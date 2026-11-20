import path from 'node:path'
import tailwindcss from '@tailwindcss/vite'
import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'

// The dashboard (dashboard.py, Flask on :5000) serves the build from web/dist.
// `npm run dev` proxies every API route to it for live development.
const API = ['/api', '/telemetry', '/delivery', '/tuning', '/ffb', '/vehicle_tuning', '/season',
  '/daycycle', '/windows', '/drones', '/shutdown', '/signal', '/keybinds', '/sessions', '/download', '/rl']

export default defineConfig({
  plugins: [react(), tailwindcss()],
  resolve: { alias: { '@': path.resolve(__dirname, './src') } },
  server: { proxy: Object.fromEntries(API.map((p) => [p, 'http://localhost:5000'])) },
  // two pages: the dashboard and the Paper 1 results page (served at /paper1)
  // 2 ページ：ダッシュボードと Paper 1 結果ページ
  build: { outDir: 'dist', emptyOutDir: true, chunkSizeWarningLimit: 1500,
    rollupOptions: { input: { main: path.resolve(__dirname, 'index.html'), paper1: path.resolve(__dirname, 'paper1.html') } } },
})
