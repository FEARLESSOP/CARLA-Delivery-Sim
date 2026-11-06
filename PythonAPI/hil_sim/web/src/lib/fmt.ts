export const fmt = (n: unknown, d = 2): string => (Number(n) || 0).toFixed(d)
export const clamp = (v: number, a: number, b: number): number => Math.max(a, Math.min(b, v))
export const round = (v: number, d = 0): number => { const k = 10 ** d; return Math.round(v * k) / k }

export function fmtDur(sec: number): string {
  const s = Math.max(0, Math.round(sec))
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), r = s % 60
  return h ? `${h}:${String(m).padStart(2, '0')}:${String(r).padStart(2, '0')}` : `${m}:${String(r).padStart(2, '0')}`
}

export function ago(t: number | null | undefined): string {
  if (!t) return '–'
  const s = Math.max(0, Date.now() / 1000 - t)
  return s < 60 ? `${Math.round(s)} s ago` : s < 3600 ? `${Math.round(s / 60)} min ago` : `${Math.round(s / 3600)} h ago`
}
