// Entry of the Paper 1 results page (paper1.html, served at /paper1).
// Paper 1 結果ページの入口
import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import '../index.css'
import { LangProvider } from '@/lib/i18n'
import ResultsPage from '@/views/paper1/ResultsPage'

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <LangProvider>
      <ResultsPage />
    </LangProvider>
  </StrictMode>,
)
