// Pure helpers for Settings > Keybinds (ported 1:1 from the old page):
// pygame key names, pretty printing, browser key -> pygame name, and the
// conflict rules of keybinds.conflicts().
// キー割り当ての純粋関数（旧ページから移植）

import type { KbAction, KbBindings } from './types'

export const KB_MODS = ['ctrl', 'shift', 'alt'] as const
type Mod = (typeof KB_MODS)[number]
const isMod = (s: string): s is Mod => (KB_MODS as readonly string[]).includes(s)

const KB_PRETTY: Record<string, string> = {
  up: '↑ Up', down: '↓ Down', left: '← Left', right: '→ Right',
  space: 'Space', tab: 'Tab', return: 'Enter', enter: 'Num Enter', escape: 'Esc',
  backspace: 'Backspace', delete: 'Delete', insert: 'Insert', home: 'Home', end: 'End',
  'page up': 'PgUp', 'page down': 'PgDn', 'left shift': 'L-Shift', 'right shift': 'R-Shift',
  'left ctrl': 'L-Ctrl', 'right ctrl': 'R-Ctrl', 'left alt': 'L-Alt', 'right alt': 'R-Alt',
  'caps lock': 'Caps Lock',
}

/** browser KeyboardEvent.code -> pygame key name / ブラウザのキー → pygame のキー名 */
const KB_CODE: Record<string, string> = {
  ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right',
  Space: 'space', Tab: 'tab', Enter: 'return', NumpadEnter: 'enter', Escape: 'escape',
  Backspace: 'backspace', Delete: 'delete', Insert: 'insert', Home: 'home', End: 'end',
  PageUp: 'page up', PageDown: 'page down', BracketLeft: '[', BracketRight: ']', Comma: ',',
  Period: '.', Slash: '/', Backslash: '\\', IntlRo: '\\', IntlYen: '\\', Semicolon: ';', Quote: "'",
  Backquote: '`', Minus: '-', Equal: '=', ShiftLeft: 'left shift', ShiftRight: 'right shift',
  ControlLeft: 'left ctrl', ControlRight: 'right ctrl', AltLeft: 'left alt', AltRight: 'right alt',
  CapsLock: 'caps lock', NumpadAdd: '[+]', NumpadSubtract: '[-]', NumpadMultiply: '[*]',
  NumpadDivide: '[/]', NumpadDecimal: '[.]',
}

export const KB_MODKEYS = new Set(['left shift', 'right shift', 'left ctrl', 'right ctrl', 'left alt', 'right alt'])

export function kbParse(spec: string): { mods: Mod[]; key: string } {
  const parts = String(spec).split('+')
  const mods: Mod[] = []
  while (parts.length > 1 && isMod(parts[0])) mods.push(parts.shift() as Mod)
  return { mods, key: parts.join('+') }
}

export function kbPretty(spec: string): string {
  const { mods, key } = kbParse(spec)
  let k = KB_PRETTY[key]
  if (!k) k = key.length === 1 || /^f\d+$/.test(key) ? key.toUpperCase()
    : /^\[.+\]$/.test(key) ? `Num ${key.slice(1, -1)}` : key
  return KB_MODS.filter((m) => mods.includes(m)).map((m) => m[0].toUpperCase() + m.slice(1)).concat([k]).join('+')
}

/** pygame name of a pressed key, or null for keys pygame bindings can't use. */
export function kbName(e: KeyboardEvent): string | null {
  const c = e.code || ''
  if (/^Key[A-Z]$/.test(c)) return c.slice(3).toLowerCase()
  if (/^Digit\d$/.test(c)) return c.slice(5)
  if (/^F([1-9]|1[0-5])$/.test(c)) return c.toLowerCase()
  if (/^Numpad\d$/.test(c)) return `[${c.slice(6)}]`
  return KB_CODE[c] ?? null
}

/** The modifiers held during a key event, for a non-hold action. */
export const eventMods = (e: KeyboardEvent): Mod[] =>
  KB_MODS.filter((m) => (m === 'ctrl' ? e.ctrlKey : m === 'shift' ? e.shiftKey : e.altKey))

export interface Conflicts { keys: Record<string, string[]>; wheel: Record<string, string[]> }

/** Same rules as keybinds.conflicts(): a hold action clashes on its base key
 *  whatever the modifiers. Actions targeting different windows (driving vs carla)
 *  do not clash for keys. / 押し続ける操作は修飾キーに関係なく衝突する。別ウィンドウは衝突除外 */
export function kbConflicts(actions: KbAction[], b: KbBindings): Conflicts {
  const keyUsers: Record<string, { id: string; mods: string; hold: boolean; s: string; win: string }[]> = {}
  const wheelUsers: Record<string, string[]> = {}
  const keys: Record<string, string[]> = {}, wheel: Record<string, string[]> = {}
  for (const a of actions) {
    const win = a.window || 'driving'
    for (const s of b[a.id]?.keys ?? []) {
      const p = kbParse(s)
      ;(keyUsers[p.key] ??= []).push({ id: a.id, mods: [...p.mods].sort().join('+'), hold: a.kind === 'hold', s, win })
    }
    for (const w of b[a.id]?.wheel ?? []) (wheelUsers[w] ??= []).push(a.id)
  }
  for (const users of Object.values(keyUsers))
    users.forEach((u, i) => users.slice(i + 1).forEach((v) => {
      if (u.id === v.id || u.win !== v.win || !(u.hold || v.hold || u.mods === v.mods)) return
      for (const s of [u.s, v.s]) {
        const l = (keys[s] ??= [])
        for (const x of [u.id, v.id]) if (!l.includes(x)) l.push(x)
      }
    }))
  for (const [w, u] of Object.entries(wheelUsers)) {
    const set = [...new Set(u)]
    if (set.length > 1) wheel[w] = set
  }
  return { keys, wheel }
}

/** Order of wheel specs: buttons by number, then hats by number and direction. */
export function wheelOrder(s: string): number {
  const m = s.split(':')
  return (m[0] === 'button' ? 0 : 1000) + Number(m[1]) * 10 + ['up', 'down', 'left', 'right'].indexOf(m[2] || 'up')
}

/** Put ``spec`` at index ``i`` of ``list`` (i == length adds). A spec already
 *  in the list elsewhere is not duplicated: editing an existing chip into it
 *  removes that chip. / 重複は作らない */
export function placeSpec(list: string[], i: number, spec: string): string[] {
  const out = [...list]
  const dup = out.indexOf(spec)
  if (dup >= 0 && dup !== i) { if (i < out.length) out.splice(i, 1) }
  else out[i] = spec
  return out
}
