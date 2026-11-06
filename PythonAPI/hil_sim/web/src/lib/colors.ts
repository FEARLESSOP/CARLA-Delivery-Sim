// Colours for canvas drawing (CSS variables cannot be used inside a canvas
// directly). Keep in step with the @theme tokens in index.css.
// キャンバス用の色（index.css のトークンと同じ値）
export const C = {
  bg: '#0b1120', card: '#151e31', line: '#2a3650', grid: '#1f2a40', axis: '#3a4a66',
  fg: '#f1f5f9', fg2: '#cbd5e1', mute: '#94a3b8',
  accent: '#22c55e', info: '#3b82f6', warn: '#f59e0b', danger: '#ef4444', violet: '#8b5cf6', cyan: '#22d3ee',
  s1: '#60a5fa', s2: '#f87171', s3: '#fbbf24', s4: '#34d399', s5: '#a78bfa',
} as const

export const FONT_SANS = '"Fira Sans", system-ui, sans-serif'
export const FONT_MONO = '"Fira Code", ui-monospace, monospace'
