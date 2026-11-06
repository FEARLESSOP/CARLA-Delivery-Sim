# CARLA HIL dashboard — web frontend (React + TypeScript)

The dashboard UI. `dashboard.py` (Flask, port 5000) serves the build
(`web/dist`) at `/` and keeps every JSON route; this app only talks to those
routes. It replaces the old single-file page that lived in `dashboard.py`
(`HTML = r"""..."""`).

## Commands

Node lives in `~/.local/node` (not on the PATH by default):

```bash
export PATH=~/.local/node/bin:$PATH
cd ~/Downloads/CARLA_0.9.16/PythonAPI/hil_sim/web
npm run build                      # tsc -b && vite build -> dist/ (what Flask serves)
npx tsc -b                         # type-check only
npx vite --port 5173               # live dev server, API proxied to :5000
```

## Layout

| Path | What |
|---|---|
| `src/App.tsx` | shell: header, nav (hash routes `#drive`, `#settings/ffb`), status, language |
| `src/lib/api.ts` | `getJSON` / `postJSON`, launcher + control types |
| `src/lib/poll.ts` | `usePoll(url, ms, enabled)`: poll a route while visible |
| `src/lib/telemetry.ts` | `/telemetry` at 5 Hz for the whole app: `useTelemetry()`, `onTelemetry(cb)` (history that keeps recording while a view is hidden), `num(s, key, default)` |
| `src/lib/sim.ts` | launcher/pipeline status: `useSim()`, `simPhase()` |
| `src/lib/i18n.tsx` | `registerDict(en, ja)`, `useT()` → `t(key, {var})`, `useLang()` |
| `src/lib/canvas.ts` | `useCanvas(draw, deps)` (sized to CSS box, DPR aware, reallocates only on resize), `Series` ring buffer |
| `src/lib/colors.ts` | colours for canvas code (`C.s1..s5` chart series, `C.accent/warn/danger/info`, `FONT_MONO`) |
| `src/lib/fmt.ts` | `fmt(n, d)`, `clamp`, `fmtDur`, `ago` |
| `src/components/ui` | **the UI kit**: `Card Grid SectionTitle Stat KV Bar Badge Indicator EmptyState Button Toggle Slider Stepper Select Segmented TextInput` |
| `src/components/charts/StreamChart.tsx` | live line chart with legend + hover readout |
| `src/components/bits/` | React Bits components (CountUp, SpotlightCard, ShinyText, GradientText, StarBorder, Aurora, AnimatedList, FadeContent) |
| `src/views/<view>/` | one folder per page; `index.tsx` default-exports the page |
| `src/i18n/<view>.ts` | that page's EN/JA strings (`registerDict`) |

## Rules

1. **Feature parity first.** Every card, control, readout, tooltip, alert and
   endpoint of the old page keeps working. Read the old HTML/JS in
   `dashboard.py` for the exact behaviour (routes, payloads, intervals,
   thresholds, colours of states). Nothing is dropped silently.
2. **Use the kit.** Build from `src/components/ui`; no ad-hoc buttons, inputs or
   card styles. Tokens only (Tailwind classes `bg-card text-fg text-fg-mute
   border-line text-accent text-warn text-danger text-info`, or `C.*` in canvas
   code) — never raw hex in JSX.
3. **TypeScript strict.** No `any`; type every API response you use (an
   interface in your view folder; loosely typed fields read with defaults).
4. **Performance.** Canvas for anything drawn per frame (charts, maps);
   `useCanvas` handles sizing. Poll with `usePoll` (it pauses on hidden tabs)
   or `useTelemetry`. Expensive drawing: cache static layers (see the old
   delivery map: Path2D in world units + oversized cached image).
5. **Accessibility/UX** (ui-ux-pro-max): visible labels, keyboard reachable,
   `aria-label` on icon-only buttons, never colour as the only cue (add text or
   shape), tabular numbers (`num` class) for values, empty/loading states,
   respect `prefers-reduced-motion` (global CSS already does).
6. **i18n.** Every visible string goes through `t()`; copy the old EN/JA texts
   (old `I18N` object in `dashboard.py`) into `src/i18n/<view>.ts`.
7. **Stay in your folder.** Only edit `src/views/<yours>/` and
   `src/i18n/<yours>.ts`. If the kit lacks something, build it locally in your
   folder (note it in your report) — do not edit shared files.
8. **Design**: dark slate, green accent, dense but scannable. Cards in a
   responsive `Grid`, most important readouts biggest (speed, status), group
   related controls, short helper text under controls. Use React Bits sparingly
   (e.g. `CountUp` for totals, `AnimatedList` for event feeds) — never on
   values that change at 5 Hz.
