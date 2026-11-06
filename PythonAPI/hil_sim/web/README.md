# CARLA HIL dashboard (web)

The rig's dashboard UI: React 19 + TypeScript + Vite + Tailwind v4, served by
`dashboard.py` from `dist/` (committed). Developer guide, layout and rules:
[DEV.md](DEV.md).

```bash
export PATH=~/.local/node/bin:$PATH
npm install          # once
npm run build        # -> dist/ (what Flask serves)
npx vite --port 5173 # live development, API proxied to :5000
```
