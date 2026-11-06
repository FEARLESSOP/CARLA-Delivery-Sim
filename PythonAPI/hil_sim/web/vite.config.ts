import path from 'node:path'
import tailwindcss from '@tailwindcss/vite'
import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'

// The dashboard (dashboard.py, Flask on :5000) serves the build from web/dist.
// `npm run dev` proxies every API route to it for live development.
const API = ['/api', '/telemetry', '/delivery', '/tuning', '/ffb', '/vehicle_tuning', '/season',
  '/daycycle', '/windows', '/shutdown', '/signal', '/keybinds', '/sessions', '/download']

export default defineConfig({
  plugins: [react(), tailwindcss()],
  resolve: { alias: { '@': path.resolve(__dirname, './src') } },
  server: { proxy: Object.fromEntries(API.map((p) => [p, 'http://localhost:5000'])) },
  build: { outDir: 'dist', emptyOutDir: true, chunkSizeWarningLimit: 1500 },
})
