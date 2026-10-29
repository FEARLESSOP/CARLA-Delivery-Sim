"""Headless test of the force-feedback settings and the centring behaviour.

Checks ffb_settings.py (load / save / clamp / presets) and, with a simple
wheel model (rim inertia + the hardware damper and friction), that the
wheel let go at half lock returns to centre smoothly: no overshoot past
centre and no force step near zero. No CARLA server or wheel needed (the
carla and evdev modules must import).
ハンドルを離したときにセンターへ滑らかに戻るか（行き過ぎ・段差なし）を簡易
モデルで確かめる。CARLA サーバとハンドルは不要。
"""
import json
import math
import os
import sys
import tempfile

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)

import ffb_settings
import carla_ffb
from carla_ffb import T

FAILS = []
def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)


def settings():
    print("settings")
    path = os.path.join(tempfile.mkdtemp(), "ffb_tuning.json")
    check("missing file gives the defaults", ffb_settings.load(path) == ffb_settings.DEFAULTS)
    saved = ffb_settings.save({"center_spring": 9.0, "invert": 0.4, "nonsense": 1}, path)
    check("save clamps to the range", saved["center_spring"] == 1.5, str(saved))
    check("save drops unknown keys", "nonsense" not in saved)
    check("invert is an integer", saved["invert"] == 0)
    v = ffb_settings.load(path)
    check("load overlays the file on the defaults",
          v["center_spring"] == 1.5 and v["damper_base"] == ffb_settings.DEFAULTS["damper_base"])
    with open(path, "w") as f:
        f.write("{broken")
    check("a broken file gives the defaults", ffb_settings.load(path) == ffb_settings.DEFAULTS)
    for name, diff in ffb_settings.PRESETS.items():
        check("preset %s only names known keys" % name, set(diff) <= set(ffb_settings.DEFAULTS))
    check("every default is inside its range",
          all(lo <= ffb_settings.DEFAULTS[k] <= hi for k, (lo, hi) in ffb_settings.LIMITS.items()))
    check("every parameter is used by carla_ffb.py",
          all('T["%s"]' % k in open(carla_ffb.__file__).read() for k in ffb_settings.DEFAULTS),
          str([k for k in ffb_settings.DEFAULTS
               if 'T["%s"]' % k not in open(carla_ffb.__file__).read()]))


def presets():
    """User presets: save, load, replace, delete, built-ins protected; and the
    dashboard's /ffb route driving them (Flask test client, temp files).
    ユーザープリセットの保存・読込・上書き・削除と /ffb ルート。"""
    print("user presets")
    d = tempfile.mkdtemp()
    path = os.path.join(d, "ffb_presets.json")
    vals = dict(ffb_settings.DEFAULTS, center_spring=0.9, kerb_jolt=0.8)
    name = ffb_settings.save_preset("  My   kerb  setup ", vals, path)
    check("name is tidied", name == "My kerb setup", repr(name))
    check("preset loads back with its values",
          ffb_settings.preset(name, path) == vals)
    ffb_settings.save_preset(name, dict(vals, kerb_jolt=0.5), path)
    check("saving the same name replaces it",
          ffb_settings.preset(name, path)["kerb_jolt"] == 0.5
          and len(ffb_settings.user_presets(path)) == 1)
    for bad in ("forza_horizon", "   "):
        try:
            ffb_settings.save_preset(bad, vals, path)
            check("refuses %r" % bad, False)
        except ValueError:
            check("refuses %r" % bad, True)
    with open(path) as f:
        data = json.load(f)
    data["old"] = {"center_spring": 0.2}          # saved before newer settings existed
    with open(path, "w") as f:
        json.dump(data, f)
    old = ffb_settings.preset("old", path)
    check("an older preset gets defaults for missing settings",
          old["center_spring"] == 0.2 and old["return_damper"] == ffb_settings.DEFAULTS["return_damper"])
    check("unknown preset is None", ffb_settings.preset("nope", path) is None)
    ffb_settings.delete_preset(name, path)
    check("delete removes it", name not in ffb_settings.user_presets(path))

    import dashboard
    saved = (ffb_settings.TUNING_PATH, ffb_settings.USER_PRESETS_PATH)
    ffb_settings.TUNING_PATH = os.path.join(d, "ffb_tuning.json")
    ffb_settings.USER_PRESETS_PATH = os.path.join(d, "route_presets.json")
    defaults = ffb_settings.load.__defaults__, ffb_settings.save.__defaults__, \
        ffb_settings.preset.__defaults__, ffb_settings.user_presets.__defaults__, \
        ffb_settings.save_preset.__defaults__, ffb_settings.delete_preset.__defaults__
    ffb_settings.load.__defaults__ = ffb_settings.save.__defaults__ = (ffb_settings.TUNING_PATH,)
    for fn in (ffb_settings.preset, ffb_settings.user_presets, ffb_settings.save_preset,
               ffb_settings.delete_preset):
        fn.__defaults__ = (ffb_settings.USER_PRESETS_PATH,)
    try:
        c = dashboard.app.test_client()
        r = c.post("/ffb", json={"values": {"kerb_jolt": 0.9}, "save_preset": "Track"}).get_json()
        check("route saves a preset with the values sent",
              r["user_presets"] == ["Track"] and r["active"] == "Track" and r["error"] is None)
        r = c.post("/ffb", json={"preset": "heavy"}).get_json()
        check("built-in preset loads and is the active one",
              r["active"] == "heavy" and r["values"]["damper_base"] == 0.3)
        r = c.post("/ffb", json={"preset": "Track"}).get_json()
        check("user preset loads", r["values"]["kerb_jolt"] == 0.9 and r["active"] == "Track")
        r = c.post("/ffb", json={"values": {"kerb_jolt": 0.3}}).get_json()
        check("changing a slider leaves no preset active", r["active"] is None)
        r = c.post("/ffb", json={"save_preset": "light"}).get_json()
        check("route reports a built-in name as an error", bool(r["error"]))
        r = c.post("/ffb", json={"delete_preset": "Track"}).get_json()
        check("route deletes a preset", r["user_presets"] == [])
    finally:
        ffb_settings.TUNING_PATH, ffb_settings.USER_PRESETS_PATH = saved
        (ffb_settings.load.__defaults__, ffb_settings.save.__defaults__,
         ffb_settings.preset.__defaults__, ffb_settings.user_presets.__defaults__,
         ffb_settings.save_preset.__defaults__, ffb_settings.delete_preset.__defaults__) = defaults


def let_go(kmh, seconds=3.0, dt=1.0 / 250, damp=0.3, fric=0.35):
    """Wheel released at half lock, rolling at ``kmh`` in a straight line.

    Rim inertia and the motor's torque scale give a G923-like response; the
    driver's damper/friction act on the rim velocity like the hardware
    effects do (``damp`` Nm per rad/s and ``fric`` Nm at 100 %: not
    published for the G923, so the test tries a range). Returns the trace
    of (s_norm, force, rate).
    """
    J, TORQUE, DAMP, FRIC = 0.06, 2.2, damp, fric  # kg m^2, Nm at 100 %
    LOCK = math.radians(450)                     # half the 900 deg rim / 900 度の半分
    x, v, steer_f = 0.5, 0.0, None
    steering = type("S", (), {"steer": staticmethod(lambda x: 0.5 * x)})()
    trace = []
    for _ in range(int(seconds / dt)):
        s_norm = steering.steer(x) * 2.0
        target = -T["kpi_nm"] * s_norm / T["torque_full_nm"] + carla_ffb.centre_spring(s_norm, kmh)
        steer_f = target if steer_f is None else \
            steer_f + carla_ffb.clamp(target - steer_f, -T["torque_slew"] * dt, T["torque_slew"] * dt)
        rate = v / LOCK                         # s_norm per second / 全ロック単位
        f = carla_ffb.output_stage((steer_f + carla_ffb.return_damper(s_norm, rate)) * T["strength"])
        damper = min(T["damper_max"], T["damper_base"] + T["damper_per_kmh"] * kmh) + \
            carla_ffb.centre_damper(s_norm)
        friction = T["friction_column"] + T["friction_park"] * math.exp(-kmh / T["friction_park_kmh"])
        torque = TORQUE * f - DAMP * damper * v
        fr = FRIC * friction
        if v == 0.0 and abs(torque) <= fr:
            torque = 0.0                        # held by static friction / 静止摩擦で停止
        else:
            torque -= math.copysign(fr, v if v else torque)
        v_new = v + torque / J * dt
        if v and v_new * v < 0:
            v_new = 0.0                         # friction can stop, never reverse / 摩擦は反転させない
        v = v_new
        x += v * dt / LOCK
        trace.append((s_norm, f, v))
    return trace


def centring():
    print("centring (Forza Horizon defaults)")
    T.update(ffb_settings.DEFAULTS)
    xs = [i / 1000.0 - 0.1 for i in range(201)]
    steps = [abs(carla_ffb.output_stage(b) - carla_ffb.output_stage(a)) for a, b in zip(xs, xs[1:])]
    check("output has no step through zero", max(steps) < 0.004, "max step %.4f" % max(steps))
    check("spring pulls back to centre", carla_ffb.centre_spring(0.5, 60) < 0 < carla_ffb.centre_spring(-0.5, 60))
    check("spring is stronger at speed",
          abs(carla_ffb.centre_spring(0.5, 100)) > abs(carla_ffb.centre_spring(0.5, 0)))
    check("centre damper is strongest on centre",
          carla_ffb.centre_damper(0.0) > carla_ffb.centre_damper(0.03) > carla_ffb.centre_damper(0.2) == 0.0)
    # the G923's own damping isn't published: try light to heavy
    # G923 自体の減衰は非公開のため軽～重で試す
    for damp in (0.1, 0.2, 0.3):
        for kmh in (30, 80, 140):
            tr = let_go(kmh, damp=damp)
            over = max(0.0, -min(s for s, _, _ in tr))
            t = next((i / 250.0 for i, (s, _, _) in enumerate(tr) if abs(s) < 0.06), 9.0)
            df = max(abs(b[1] - a[1]) for a, b in zip(tr, tr[1:]))
            check("let go at %3d km/h, rim damping %.1f: back on centre, no overshoot" % (kmh, damp),
                  over < 0.05 and t < 3.0,
                  "overshoot %.3f, within 6%% after %.2f s, largest force step %.4f" % (over, t, df))
            check("let go at %3d km/h, rim damping %.1f: force changes smoothly" % (kmh, damp),
                  df < 0.04, "%.4f" % df)

    print("for comparison: the previous tuning / 以前の設定")
    T.update(ffb_settings.preset("previous"))
    for damp in (0.1, 0.2, 0.3):
        for kmh in (30, 80, 140):
            tr = let_go(kmh, damp=damp)
            over = max(0.0, -min(s for s, _, _ in tr))
            print("        %3d km/h, rim damping %.1f: overshoot %.3f" % (kmh, damp, over))
    T.update(ffb_settings.DEFAULTS)


settings()
presets()
centring()
print("\n%s" % ("ALL PASSED" if not FAILS else "FAILED: " + ", ".join(FAILS)))
sys.exit(1 if FAILS else 0)
