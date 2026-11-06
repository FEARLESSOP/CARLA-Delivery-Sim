import { useCallback, useEffect, useRef, useState, type ReactNode } from 'react'
import { AlertTriangle, Check, Keyboard, Plus, RotateCcw, Save, ScanSearch, Square, X } from 'lucide-react'
import { getJSON, postJSON } from '@/lib/api'
import { useLang, useT } from '@/lib/i18n'
import { cn } from '@/lib/utils'
import { Badge, Button, Card, EmptyState, Select, TextInput } from '@/components/ui'
import { KB_MODKEYS, eventMods, kbConflicts, kbName, kbPretty, placeSpec, wheelOrder, type Conflicts } from './keybinds-lib'
import type { KbAction, KbBindings, KbData, LabelPost, LearnReply, WheelLast, WheelPoll } from './types'

// Settings > Keybinds: one row per action of the driving window with a
// Keyboard and a Wheel column. Wheel inputs are always shown by name
// (g923_buttons.json), never by number. Binding / naming a wheel button
// listens for the press the driving window writes to wheel_last.json
// (/keybinds/learn + /keybinds/wheel every 250 ms): it reads the wheel, not
// the browser. Edits are a draft until "save" (POST /keybinds).
// 設定 > キー割り当て。ホイールは名前で表示し、入力は運転ウインドウが
// 書いたものを待ち受ける。保存するまでは下書き。

type Col = 'keys' | 'wheel'
interface Listen {
  mode: 'bind' | 'identify'
  id?: string
  i?: number
  since: number       // server time of the last press already seen
  started: number
  renew: number       // when to extend the learn window again
  deadline: number    // bind mode gives up after 20 s
}
type Msg = { key: string; extra?: string } | null

const clone = <T,>(o: T): T => JSON.parse(JSON.stringify(o)) as T
const learn = (seconds: number) => postJSON<LearnReply>('/keybinds/learn', { seconds }).catch(() => null)

// The draft survives switching sections (the old page kept it in memory).
// セクションを切り替えても下書きを保持
let draft: KbBindings | null = null

/* ------------------------------------------------------------- chips */
function KeyChip({ text, conflict, unknown, wheel, onEdit, onRemove, editLabel, removeLabel }: {
  text: string; conflict: boolean; unknown?: boolean; wheel?: boolean
  onEdit: () => void; onRemove: () => void; editLabel: string; removeLabel: string
}) {
  return (
    <span className={cn('inline-flex h-7 items-stretch overflow-hidden rounded-md border text-xs',
      conflict ? 'border-warn/60 bg-warn/10 text-warn' : wheel ? 'border-info/35 bg-info/10 text-fg' : 'border-line-strong bg-card-2 text-fg',
      unknown && !conflict && 'border-dashed text-fg-mute')}>
      <button type="button" onClick={onEdit} title={editLabel}
        className={cn('inline-flex items-center gap-1 px-2 hover:bg-muted', !wheel && 'num font-medium')}>
        {conflict && <AlertTriangle className="h-3 w-3" aria-hidden />}{text}
      </button>
      <button type="button" onClick={onRemove} aria-label={removeLabel} title={removeLabel}
        className="grid w-6 place-items-center border-l border-inherit text-fg-mute hover:bg-danger/20 hover:text-danger">
        <X className="h-3 w-3" aria-hidden />
      </button>
    </span>
  )
}

function CaptureChip({ text }: { text: string }) {
  return (
    <span className="inline-flex h-7 items-center gap-1.5 rounded-md border border-accent bg-accent/15 px-2 text-xs font-medium text-accent motion-safe:animate-pulse">
      <i className="h-1.5 w-1.5 rounded-full bg-accent" aria-hidden />{text}
    </span>
  )
}

/* ------------------------------------------------- identify "now" box */
function IdentNow({ spec, data, binds, onConfirm, onRename }: {
  spec: string; data: KbData; binds: KbBindings
  onConfirm: () => void; onRename: (name: string) => void
}) {
  const t = useT()
  const { lang } = useLang()
  const raw = data.labels[spec]
  const name = raw ? (lang === 'ja' ? data.label_ja[raw] || raw : raw) : null
  const ok = !!name && data.confirmed.includes(spec)
  const uses = data.actions.filter((a) => (binds[a.id]?.wheel ?? []).includes(spec)).map((a) => (lang === 'ja' ? a.ja : a.en))
  const choices = [...new Set(data.stock_order.concat(Object.values(data.labels)))]
  const [sel, setSel] = useState<string>(raw || '__custom')
  const [txt, setTxt] = useState('')
  const value = sel === '__custom' ? txt.trim() : sel
  return (
    <div className="space-y-3 rounded-xl border border-line bg-surface/70 p-4">
      <div className="flex flex-wrap items-center gap-2">
        <span className="text-xs text-fg-mute">{t('kb_now')}</span>
        <span className={cn('text-lg font-semibold', name ? 'text-fg' : 'text-warn')}>{name ?? t('kb_unknown')}</span>
        <span className="num text-xs text-fg-mute">{spec}</span>
        {name && <Badge tone={ok ? 'good' : 'warn'} dot>{t(ok ? 'kb_id_confirmed' : 'kb_id_unconfirmed')}</Badge>}
      </div>
      <div className="text-sm text-fg-2">{uses.length ? t('kb_id_bound', { a: uses.join(', ') }) : t('kb_id_free')}</div>
      <div className="flex flex-wrap items-end gap-2">
        {name && !ok && <Button variant="primary" className="mb-1" onClick={onConfirm}><Check className="h-4 w-4" aria-hidden />{t('kb_id_correct')}</Button>}
        <div className="min-w-44 flex-1">
          <Select label={t('kb_id_rename')} value={sel} onChange={setSel}
            options={choices.map((c) => ({ value: c, label: lang === 'ja' ? data.label_ja[c] || c : c }))
              .concat([{ value: '__custom', label: t('kb_id_custom') }])} />
        </div>
        {sel === '__custom' && (
          <div className="min-w-44 flex-1">
            <TextInput label={t('kb_id_custom_ph')} value={txt} onChange={(v) => setTxt(v.slice(0, 40))} />
          </div>
        )}
        <Button className="mb-1" disabled={!value || value === raw} onClick={() => value && onRename(value)}>{t('kb_id_rename')}</Button>
      </div>
    </div>
  )
}

/* -------------------------------------------------------------- page */
export default function KeybindsSection() {
  const t = useT()
  const { lang } = useLang()
  const [data, setData] = useState<KbData | null>(null)
  const [binds, setBindsState] = useState<KbBindings | null>(draft)
  const [dirty, setDirty] = useState(draft !== null)
  const [loadErr, setLoadErr] = useState(false)
  const [cap, setCap] = useState<{ id: string; i: number } | null>(null)
  const [listen, setListenState] = useState<Listen | null>(null)
  const listenRef = useRef<Listen | null>(null)
  const [ident, setIdentState] = useState(false)
  const identRef = useRef(false)
  const [last, setLast] = useState<WheelLast | null>(null)
  const [msg, setMsg] = useState<Msg>(null)
  const [labArm, setLabArm] = useState(false)
  const [saving, setSaving] = useState(false)
  const [, setTick] = useState(0)

  const setListen = (l: Listen | null) => { listenRef.current = l; setListenState(l) }
  const setIdent = (v: boolean) => { identRef.current = v; setIdentState(v) }
  /** change the draft / 下書きを変更 */
  const edit = useCallback((fn: (b: KbBindings) => KbBindings) => {
    setBindsState((b) => { if (!b) return b; const n = fn(b); draft = n; return n })
    setDirty(true)
  }, [])

  const actName = (a: KbAction | undefined) => (a ? (lang === 'ja' ? a.ja : a.en) : '')
  const act = (id: string) => data?.actions.find((a) => a.id === id)
  const wheelLabel = (spec: string) => {
    const n = data?.labels[spec]
    return n ? (lang === 'ja' ? data!.label_ja[n] || n : n) : null
  }
  const wheelText = (spec: string) => wheelLabel(spec) ?? t('kb_unknown')

  /* load (keeps an unsaved draft) / 読み込み（未保存の下書きは残す） */
  useEffect(() => {
    let alive = true
    getJSON<KbData>('/keybinds').then((j) => {
      if (!alive) return
      setData(j)
      if (!draft) { setBindsState(clone(j.bindings)); setDirty(false) }
    }).catch(() => alive && setLoadErr(true))
    return () => { alive = false }
  }, [])

  /* listening: start / stop / poll / ホイール入力の待ち受け */
  const startListen = async (mode: Listen['mode'], id?: string, i?: number) => {
    setCap(null)
    const j = await learn(30)
    const now = Date.now()
    setListen({ mode, id, i, since: j ? j.now : now / 1000, started: now, renew: now + 10000, deadline: now + 20000 })
    if (mode === 'identify') setIdent(true)
  }
  const stopListen = useCallback(() => {
    if (!listenRef.current && !identRef.current) return
    listenRef.current = null; setListenState(null)
    identRef.current = false; setIdentState(false)
    void learn(0)
  }, [])

  useEffect(() => {
    if (!listen) return
    let busy = false
    const poll = async () => {
      const L = listenRef.current
      if (!L || busy) return
      if (L.mode === 'bind' && Date.now() > L.deadline) {
        setListen(null); void learn(0)
        setMsg({ key: 'kb_no_driver' })
        return
      }
      if (Date.now() > L.renew) { L.renew = Date.now() + 10000; void learn(30) }
      busy = true
      let j: WheelPoll | null = null
      try { j = await getJSON<WheelPoll>('/keybinds/wheel') } catch { /* keep listening */ } finally { busy = false }
      if (!j || listenRef.current !== L) return
      const lp = j.last
      if (lp && lp.t > L.since) {
        L.since = lp.t
        setLast(lp)
        if (lp.label !== undefined) {
          setData((d) => {
            if (!d || d.labels[lp.spec] === (lp.label || undefined)) return d
            const labels = { ...d.labels }
            if (lp.label) labels[lp.spec] = lp.label; else delete labels[lp.spec]
            return { ...d, labels }
          })
        }
        if (L.mode === 'bind' && L.id !== undefined && L.i !== undefined) {
          const { id, i } = L
          setListen(null); void learn(0)
          edit((b) => ({ ...b, [id]: { ...b[id], wheel: placeSpec(b[id].wheel, i, lp.spec) } }))
          return
        }
      }
      setTick((x) => x + 1)
    }
    const h = window.setInterval(poll, 250)
    return () => window.clearInterval(h)
    // the interval reads the listen ref; restart only when listening starts/stops
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [listen !== null, edit])

  /* stop listening when the section closes or the page unloads */
  useEffect(() => {
    const onUnload = () => {
      if (listenRef.current || identRef.current)
        navigator.sendBeacon?.('/keybinds/learn', new Blob([JSON.stringify({ seconds: 0 })], { type: 'application/json' }))
    }
    window.addEventListener('beforeunload', onUnload)
    return () => { window.removeEventListener('beforeunload', onUnload); stopListen() }
  }, [stopListen])

  /* keyboard capture / キー入力の取り込み */
  useEffect(() => {
    if (!cap || !data) return
    const hold = data.actions.find((a) => a.id === cap.id)?.kind === 'hold'
    let pendingMod: string | null = null
    const finish = (spec: string) => {
      setCap(null)
      edit((b) => ({ ...b, [cap.id]: { ...b[cap.id], keys: placeSpec(b[cap.id].keys, cap.i, spec) } }))
    }
    const down = (e: KeyboardEvent) => {
      e.preventDefault(); e.stopPropagation()
      const name = kbName(e)
      if (!name) return
      if (KB_MODKEYS.has(name)) { pendingMod = name; return }
      finish([...(hold ? [] : eventMods(e)), name].join('+'))
    }
    const up = (e: KeyboardEvent) => {
      e.preventDefault(); e.stopPropagation()
      const name = kbName(e)
      if (name && name === pendingMod) finish(name)      // a modifier on its own
    }
    document.addEventListener('keydown', down, true)
    document.addEventListener('keyup', up, true)
    return () => { document.removeEventListener('keydown', down, true); document.removeEventListener('keyup', up, true) }
  }, [cap, data, edit])

  /* lab reset arm (4 s) */
  useEffect(() => { if (!labArm) return; const id = setTimeout(() => setLabArm(false), 4000); return () => clearTimeout(id) }, [labArm])

  if (!data || !binds) return <EmptyState>{loadErr ? t('s_offline') : t('s_loading')}</EmptyState>

  /* ---------------------------------------------------------- actions */
  const startEdit = (id: string, col: Col, i: number) => {
    setMsg(null)
    if (listenRef.current) { setListen(null); setIdent(false); void learn(0) }
    if (col === 'keys') setCap({ id, i })
    else void startListen('bind', id, i)
  }
  const cancelCap = () => {
    setCap(null)
    if (listenRef.current?.mode === 'bind') { setListen(null); void learn(0) }
  }
  const save = async () => {
    setSaving(true)
    try {
      const j = await postJSON<KbData & { error?: string }>('/keybinds', { bindings: binds })
      if (j.error) throw new Error(j.error)
      setData(j); draft = null; setBindsState(clone(j.bindings)); setDirty(false)
      setMsg({ key: 'kb_saved' })
    } catch (e) {
      setMsg({ key: 'kb_save_fail', extra: e instanceof Error ? e.message : String(e) })
    } finally { setSaving(false) }
  }
  const labelPost = async (body: LabelPost) => {
    try {
      const j = await postJSON<KbData>('/keybinds/label', body)
      if (j.labels) setData((d) => (d ? { ...d, labels: j.labels, confirmed: j.confirmed } : d))
    } catch { /* keep the old names */ }
  }

  /* ----------------------------------------------------------- render */
  const conf: Conflicts = kbConflicts(data.actions, binds)
  const L = listen
  const capOn = (id: string, col: Col, i: number) =>
    (col === 'keys' && cap?.id === id && cap.i === i) || (col === 'wheel' && L?.mode === 'bind' && L.id === id && L.i === i)

  let capMsg: string | null = null
  if (cap) {
    const a = act(cap.id)
    capMsg = t(a?.kind === 'hold' ? 'kb_press_key_hold' : 'kb_press_key', { a: actName(a) })
  } else if (L?.mode === 'bind') {
    capMsg = t('kb_press_wheel', { a: actName(act(L.id ?? '')), s: Math.max(0, Math.ceil((L.deadline - Date.now()) / 1000)) })
  }

  const chips = (a: KbAction, col: Col): ReactNode => {
    const list = binds[a.id]?.[col] ?? []
    const colName = t(col === 'keys' ? 'kb_col_keys' : 'kb_col_wheel')
    return (
      <div className="flex flex-wrap items-center gap-1.5">
        {list.map((spec, i) => {
          const txt = col === 'keys' ? kbPretty(spec) : wheelText(spec)
          return capOn(a.id, col, i)
            ? <CaptureChip key={`${spec}-${i}`} text={t(col === 'keys' ? 'kb_cap_key' : 'kb_cap_wheel')} />
            : <KeyChip key={`${spec}-${i}`} text={txt} wheel={col === 'wheel'} unknown={col === 'wheel' && !wheelLabel(spec)}
                conflict={col === 'keys' ? !!conf.keys[spec] : !!conf.wheel[spec]}
                editLabel={`${t('kb_change')}: ${txt}`} removeLabel={`${t('kb_remove')} ${txt}`}
                onEdit={() => startEdit(a.id, col, i)}
                onRemove={() => { setMsg(null); edit((b) => ({ ...b, [a.id]: { ...b[a.id], [col]: b[a.id][col].filter((_, k) => k !== i) } })) }} />
        })}
        {capOn(a.id, col, list.length) && <CaptureChip text={t(col === 'keys' ? 'kb_cap_key' : 'kb_cap_wheel')} />}
        {!list.length && !capOn(a.id, col, list.length) && <span className="text-xs text-fg-mute/70">{t('kb_none')}</span>}
        {list.length < 4 && !capOn(a.id, col, list.length) && (
          <button type="button" onClick={() => startEdit(a.id, col, list.length)} title={t('kb_add')}
            aria-label={`${t('kb_add')}: ${actName(a)} (${colName})`}
            className="grid h-7 w-7 place-items-center rounded-md border border-dashed border-line-strong text-fg-mute hover:border-accent hover:text-accent">
            <Plus className="h-3.5 w-3.5" aria-hidden />
          </button>
        )}
      </div>
    )
  }

  const confLines: ReactNode[] = [
    ...Object.entries(conf.keys).map(([s, ids]) => <li key={`k${s}`}><b className="num">{kbPretty(s)}</b>: {ids.map((i) => actName(act(i))).join(', ')}</li>),
    ...Object.entries(conf.wheel).map(([w, ids]) => <li key={`w${w}`}><b>{wheelText(w)}</b>: {ids.map((i) => actName(act(i))).join(', ')}</li>),
  ]

  // identify panel data / 識別パネル
  const specs = [...new Set(Object.keys(data.stock_labels).concat(Object.keys(data.labels), data.confirmed, last ? [last.spec] : []))]
    .sort((a, b) => wheelOrder(a) - wheelOrder(b))
  const named = specs.filter((s) => data.labels[s])
  const nconf = named.filter((s) => data.confirmed.includes(s)).length
  const grid = 'md:grid md:grid-cols-[minmax(11rem,1.1fr)_minmax(0,1.6fr)_minmax(0,1.6fr)_2.25rem] md:items-center md:gap-3'

  return (
    <div className="space-y-4">
      <Card title={t('kb_card')}
        actions={<>
          {dirty ? <Badge tone="warn" dot>{t('kb_unsaved')}</Badge> : <Badge tone="good"><Check className="h-3 w-3" aria-hidden />{t('s_saved')}</Badge>}
          <Button size="sm" variant="ghost" onClick={() => edit((b) => Object.fromEntries(Object.keys(b).map((k) => [k, clone(data.defaults[k] ?? b[k])])))}>
            <RotateCcw className="h-3.5 w-3.5" aria-hidden />{t('kb_reset_all')}
          </Button>
          <Button size="sm" variant="primary" disabled={!dirty} busy={saving} onClick={save}><Save className="h-3.5 w-3.5" aria-hidden />{t('kb_save')}</Button>
        </>}
        footer={t('kb_hint')}>
        {confLines.length > 0 && (
          <div role="alert" className="rounded-lg border border-warn/40 bg-warn/10 p-3 text-sm text-warn">
            <div className="mb-1 flex items-center gap-2 font-medium"><AlertTriangle className="h-4 w-4" aria-hidden />{t('kb_conflict')}</div>
            <ul className="space-y-0.5 pl-6 text-fg-2">{confLines}</ul>
          </div>
        )}
        <div role="status" aria-live="polite">
          {capMsg && (
            <div className="sticky top-16 z-10 flex flex-wrap items-center justify-between gap-3 rounded-lg border border-accent/50 bg-accent/10 p-3 text-sm text-fg">
              <span className="inline-flex items-center gap-2">
                {cap ? <Keyboard className="h-4 w-4 text-accent" aria-hidden /> : <span className="h-3 w-3 rounded-full bg-accent motion-safe:animate-ping" aria-hidden />}
                {capMsg}
              </span>
              <Button size="sm" onClick={cancelCap}>{t('kb_cancel')}</Button>
            </div>
          )}
          {msg && !capMsg && (
            <div className={cn('text-sm', msg.key === 'kb_saved' ? 'text-accent' : 'text-warn')}>{t(msg.key)}{msg.extra ?? ''}</div>
          )}
        </div>

        <div role="table" aria-label={t('kb_card')} className="text-sm">
          <div role="row" className={cn('hidden border-b border-line pb-2 text-xs font-medium text-fg-mute', grid)}>
            <span role="columnheader">{t('kb_col_action')}</span>
            <span role="columnheader">{t('kb_col_keys')}</span>
            <span role="columnheader">{t('kb_col_wheel')}</span>
            <span role="columnheader" className="sr-only">{t('kb_reset_row')}</span>
          </div>
          {data.groups.map((g) => {
            const acts = data.actions.filter((a) => a.group === g.key)
            if (!acts.length) return null
            return (
              <div key={g.key} role="rowgroup">
                <div role="row" className="mt-3 border-b border-line/60 pb-1 text-[11px] font-semibold uppercase tracking-[0.12em] text-fg-mute">
                  <span role="rowheader">{lang === 'ja' ? g.ja || g.en : g.en}</span>
                </div>
                {acts.map((a) => {
                  const changed = JSON.stringify(binds[a.id]) !== JSON.stringify(data.defaults[a.id])
                  return (
                    <div key={a.id} role="row" className={cn('space-y-1.5 border-b border-line/40 py-2 md:space-y-0', grid)}>
                      <div role="cell" className="flex items-center justify-between gap-2 md:block">
                        <span className="text-fg">{actName(a)}</span>
                        <span className="ml-2 text-[11px] text-fg-mute">{t(a.kind === 'hold' ? 'kb_hold' : 'kb_press')}</span>
                        {changed && (
                          <button type="button" onClick={() => edit((b) => ({ ...b, [a.id]: clone(data.defaults[a.id]) }))}
                            aria-label={`${t('kb_reset_row')}: ${actName(a)}`} title={t('kb_reset_row')}
                            className="grid h-7 w-7 place-items-center rounded-md border border-accent/40 text-accent hover:bg-accent/15 md:hidden">
                            <RotateCcw className="h-3.5 w-3.5" aria-hidden />
                          </button>
                        )}
                      </div>
                      <div role="cell" className="flex items-start gap-2">
                        <span className="w-16 shrink-0 pt-1.5 text-[11px] text-fg-mute md:hidden">{t('kb_col_keys')}</span>
                        {chips(a, 'keys')}
                      </div>
                      <div role="cell" className="flex items-start gap-2">
                        <span className="w-16 shrink-0 pt-1.5 text-[11px] text-fg-mute md:hidden">{t('kb_col_wheel')}</span>
                        {a.axis ? <span className="pt-1 text-xs italic text-fg-mute">{t('kb_axis')}</span> : chips(a, 'wheel')}
                      </div>
                      <div role="cell" className="hidden md:block">
                        {changed && (
                          <button type="button" onClick={() => edit((b) => ({ ...b, [a.id]: clone(data.defaults[a.id]) }))}
                            aria-label={`${t('kb_reset_row')}: ${actName(a)}`} title={t('kb_reset_row')}
                            className="grid h-7 w-7 place-items-center rounded-md border border-accent/40 text-accent hover:bg-accent/15">
                            <RotateCcw className="h-3.5 w-3.5" aria-hidden />
                          </button>
                        )}
                      </div>
                    </div>
                  )
                })}
              </div>
            )
          })}
        </div>
      </Card>

      <Card title={t('kb_id_card')}
        actions={<>
          <Button size="sm" variant={labArm ? 'danger' : 'ghost'} onClick={() => { if (labArm) { setLabArm(false); void labelPost({ reset: true }) } else setLabArm(true) }}>
            {t(labArm ? 'kb_lab_reset_sure' : 'kb_lab_reset')}
          </Button>
          <Button size="sm" variant={ident ? 'danger' : 'primary'}
            onClick={() => { if (ident) stopListen(); else { setLast(null); void startListen('identify') } }}>
            {ident ? <Square className="h-3.5 w-3.5" aria-hidden /> : <ScanSearch className="h-3.5 w-3.5" aria-hidden />}
            {t(ident ? 'kb_id_stop' : 'kb_id_start')}
          </Button>
        </>}
        footer={t('kb_id_shifter')}>
        <p className="-mt-1 text-xs text-fg-mute">{t('kb_id_hint')}</p>
        <div aria-live="polite">
          {!ident ? (
            <div className="rounded-xl border border-dashed border-line p-4 text-sm text-fg-mute">{t('kb_id_off')}</div>
          ) : !last ? (
            <div className="flex flex-wrap items-center gap-3 rounded-xl border border-accent/40 bg-accent/5 p-4 text-sm">
              <span className="h-3 w-3 rounded-full bg-accent motion-safe:animate-ping" aria-hidden />
              <span className="text-fg">{t('kb_id_wait')}</span>
              {L && Date.now() - L.started > 8000 && <span className="text-warn">{t('kb_no_driver')}</span>}
            </div>
          ) : (
            <IdentNow key={`${last.spec}|${data.labels[last.spec] ?? ''}|${data.confirmed.includes(last.spec)}|${lang}`}
              spec={last.spec} data={data} binds={binds}
              onConfirm={() => void labelPost({ spec: last.spec, confirm: true })}
              onRename={(name) => void labelPost({ spec: last.spec, label: name })} />
          )}
        </div>
        <div>
          <div className="mb-2 flex items-center justify-between gap-2 text-xs text-fg-mute">
            <span>{t('kb_all_buttons')}</span>
            <span className="num">{t('kb_id_progress', { n: nconf, m: named.length })}</span>
          </div>
          <div className="h-1.5 overflow-hidden rounded-full bg-muted" aria-hidden>
            <div className="h-full rounded-full bg-accent transition-[width] duration-300" style={{ width: `${named.length ? (nconf / named.length) * 100 : 0}%` }} />
          </div>
          <ul className="mt-3 flex flex-wrap gap-1.5">
            {specs.map((s) => {
              const ok = data.confirmed.includes(s) && !!data.labels[s]
              const now = ident && s === last?.spec
              return (
                <li key={s} title={s}
                  className={cn('inline-flex items-center gap-1 rounded-md border px-2 py-1 text-xs',
                    now ? 'border-accent bg-accent/20 text-fg ring-1 ring-accent' : ok ? 'border-accent/40 bg-accent/10 text-fg' : 'border-line text-fg-mute',
                    !data.labels[s] && 'border-dashed text-warn')}>
                  {ok && <Check className="h-3 w-3 text-accent" aria-label={t('kb_id_confirmed')} />}
                  {wheelLabel(s) ?? t('kb_wheel_unknown')}
                </li>
              )
            })}
          </ul>
        </div>
      </Card>
    </div>
  )
}
