import { createContext, useCallback, useContext, useMemo, useState, type ReactNode } from 'react'

// English / Japanese. Each view registers its own strings with registerDict()
// (src/i18n/<view>.ts); t(key, {var}) fills {var} placeholders and falls
// back to English, then to the key itself.
// 英語・日本語。各ビューが自分の文言を登録する。

export type Lang = 'en' | 'ja'
type Dict = Record<string, string>
const DICT: Record<Lang, Dict> = { en: {}, ja: {} }

export function registerDict(en: Dict, ja: Dict) {
  Object.assign(DICT.en, en)
  Object.assign(DICT.ja, ja)
}

export type TFunc = (key: string, vars?: Record<string, string | number>) => string

function translate(lang: Lang, key: string, vars?: Record<string, string | number>): string {
  let s = DICT[lang][key] ?? DICT.en[key] ?? key
  if (vars) for (const [k, v] of Object.entries(vars)) s = s.split(`{${k}}`).join(String(v))
  return s
}

interface Ctx { lang: Lang; setLang: (l: Lang) => void; t: TFunc }
const LangCtx = createContext<Ctx>({ lang: 'en', setLang: () => {}, t: (k) => k })

function initialLang(): Lang {
  try { return localStorage.getItem('carla_lang') === 'ja' ? 'ja' : 'en' } catch { return 'en' }
}

export function LangProvider({ children }: { children: ReactNode }) {
  const [lang, setLangState] = useState<Lang>(initialLang)
  const setLang = useCallback((l: Lang) => {
    setLangState(l)
    try { localStorage.setItem('carla_lang', l) } catch { /* private mode */ }
    document.documentElement.lang = l
  }, [])
  const t = useCallback<TFunc>((k, v) => translate(lang, k, v), [lang])
  const value = useMemo(() => ({ lang, setLang, t }), [lang, setLang, t])
  return <LangCtx.Provider value={value}>{children}</LangCtx.Provider>
}

export const useLang = () => useContext(LangCtx)
export const useT = () => useContext(LangCtx).t
