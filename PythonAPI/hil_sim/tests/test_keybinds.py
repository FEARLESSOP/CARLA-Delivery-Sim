"""Tests for keybinds.py and the driving window's use of it. No CARLA server.

1. the defaults reproduce the fixed layout the script had before keybinds.json
   (keys, wheel buttons 0-11/23, wheel_config.ini button numbers);
2. save / load / clean, and wheel_config.ini as the fallback for defaults;
3. conflict detection;
4. G923 button names: lookup, renaming, never a raw number;
5. the real DualControl: keys and wheel buttons trigger the bound actions,
   a rebind in keybinds.json is picked up, learn mode suppresses wheel actions.

keybinds.py と運転スクリプトでの利用の試験（CARLA サーバー不要）。
"""
import importlib.util
import os
import sys
import tempfile
import time
import types

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)
os.chdir(HIL)

import keybinds as kb  # noqa: E402

TMP = tempfile.mkdtemp(prefix="kb_test_")
P = {name: os.path.join(TMP, name) for name in
     ("keybinds.json", "g923_buttons.json", "wheel_last.json", "learn.json", "wheel.ini")}
kb.KEYBINDS_PATH, kb.LABELS_PATH = P["keybinds.json"], P["g923_buttons.json"]
kb.WHEEL_LAST_PATH, kb.LEARN_PATH = P["wheel_last.json"], P["learn.json"]
kb.WHEEL_INI_PATH = os.path.join(HIL, "wheel_config.ini")

FAILS = []


def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)


print("\n=== 1. defaults = the old fixed layout ===")
d = kb.defaults()
OLD_KEYS = {
    "throttle": ["w", "up"], "brake": ["s", "down"], "steer_left": ["a", "left"],
    "steer_right": ["d", "right"], "handbrake": ["space"], "reverse": ["q"],
    "gear_up": ["."], "gear_down": [","], "transmission": ["m"],
    "camera_next": ["tab"], "mirrors": ["n"], "glance_left": ["z"], "glance_right": ["x"],
    "recording": ["r"], "sensor_next": ["`"], "autopilot": ["p"],
    "car_next": ["backspace"], "car_prev": ["shift+backspace"],
    "weather_next": ["c"], "weather_prev": ["shift+c"], "map_next": ["f5"],
    "map_prev": ["shift+f5"], "throttle_up": ["]"], "throttle_down": ["["],
    "brake_up": ["shift+]"], "brake_down": ["shift+["], "hud": ["f1"],
    "help": ["h", "shift+/"], "fullscreen": ["f11"], "quit": ["escape", "ctrl+q"],
    "eye_fwd": ["ctrl+i"], "eye_back": ["ctrl+k"], "eye_left": ["ctrl+j"],
    "eye_right": ["ctrl+l"], "eye_down": ["ctrl+u"], "eye_up": ["ctrl+o"],
    "signal_left": [], "signal_right": [], "hazard": ["e"],
    "road_overlay": ["o"],          # added 2026-11-09 (road_overlay.py) / 追加
}
OLD_KEYS.update({"sensor_%d" % n: [str(n)] for n in range(1, 10)})
bad = [a for a, k in OLD_KEYS.items() if d[a]["keys"] != k]
check("every keyboard default matches the old script", not bad, ", ".join(bad))
check("every action has a default", set(d) == set(OLD_KEYS), str(set(d) ^ set(OLD_KEYS)))
OLD_WHEEL = {"car_next": 0, "hud": 1, "camera_next": 2, "weather_next": 3,
             "gear_up": 4, "gear_down": 5, "reverse": 6, "handbrake": 7,
             "transmission": 9, "signal_right": 10, "signal_left": 11, "sensor_next": 23,
             "hazard": 8}
bad = [a for a, b in OLD_WHEEL.items() if d[a]["wheel"] != ["button:%d" % b]]
check("every wheel default matches the old button numbers", not bad, ", ".join(bad))
others = [a for a in d if a not in OLD_WHEEL and d[a]["wheel"]]
check("no other action gets a wheel button by default", not others, ", ".join(others))
check("throttle/brake/steer are axes on the wheel",
      all(kb.BY_ID[a]["axis"] for a in ("throttle", "brake", "steer_left", "steer_right")))
check("no conflicts in the defaults", kb.conflicts(d) == {"keys": {}, "wheel": {}},
      str(kb.conflicts(d)))

with open(P["wheel.ini"], "w") as fh:
    fh.write("[G29 Racing Wheel]\nreverse = 12\nhandbrake = 13\ngear_up = 5\ngear_down = 4\n")
d2 = kb.defaults(P["wheel.ini"])
check("wheel_config.ini button numbers become the defaults",
      d2["reverse"]["wheel"] == ["button:12"] and d2["handbrake"]["wheel"] == ["button:13"]
      and d2["gear_up"]["wheel"] == ["button:5"] and d2["gear_down"]["wheel"] == ["button:4"]
      and d2["transmission"]["wheel"] == ["button:9"])

print("\n=== 2. save / load / clean ===")
check("load() creates keybinds.json with the defaults",
      kb.load() == d and os.path.isfile(P["keybinds.json"]))
b = kb.load()
b["throttle"]["keys"] = ["I", "Up"]                 # case normalised
b["hazard"]["wheel"] = ["hat:0:down"]
b["mirrors"]["keys"] = ["Shift+Ctrl+V", "nonsense-key", "shift+ctrl+v"]
b["throttle"]["wheel"] = ["button:3"]               # axis: dropped
b["bogus_action"] = {"keys": ["k"]}
kb.save(b)
r = kb.load()
check("saved bindings load back", r["throttle"]["keys"] == ["i", "up"]
      and r["hazard"]["wheel"] == ["hat:0:down"], str(r["throttle"]))
check("modifiers normalised, junk and duplicates dropped", r["mirrors"]["keys"] == ["ctrl+shift+v"],
      str(r["mirrors"]))
check("axis actions keep no wheel binding", r["throttle"]["wheel"] == [])
check("unknown actions dropped", "bogus_action" not in r)
with open(P["keybinds.json"], "w") as fh:
    fh.write('{"bindings": {"quit": {"keys": ["f12"]}}}')
r = kb.load()
check("missing actions fall back to the defaults", r["quit"]["keys"] == ["f12"]
      and r["throttle"]["keys"] == ["w", "up"])
with open(P["keybinds.json"], "w") as fh:
    fh.write("{ broken")
check("a broken file falls back to the defaults", kb.load() == d)
check("key spellings", kb.parse_key("ctrl+shift+[") == (frozenset({"ctrl", "shift"}), "[")
      and kb.parse_key("[+]") == (frozenset(), "[+]") and kb.parse_key("ctrl+") is None
      and kb.pretty_key("shift+backspace") == "Shift+Backspace" and kb.pretty_key("f5") == "F5")
check("wheel spellings", kb.parse_wheel("button:23") == ("button", 23, None)
      and kb.parse_wheel("hat:0:left") == ("hat", 0, "left") and kb.parse_wheel("joy 7") is None)

print("\n=== 3. conflicts ===")
b = kb.defaults()
b["gear_up"]["keys"] = ["w"]                        # w = throttle (hold) too
c = kb.conflicts(b)
check("same key on two actions is a conflict", set(c["keys"].get("w", [])) == {"throttle", "gear_up"},
      str(c))
b = kb.defaults()
b["mirrors"]["keys"] = ["shift+w"]                  # hold W reacts with any modifier
check("a modified key clashes with a hold action on the same key",
      "shift+w" in kb.conflicts(b)["keys"])
b = kb.defaults()
b["recording"]["keys"] = ["ctrl+c"]                 # c / shift+c / ctrl+c are different
check("different modifiers on press actions do not clash", kb.conflicts(b)["keys"] == {})
b["hazard"]["wheel"] = ["button:3"]                 # Triangle = next weather already
check("same wheel button on two actions is a conflict",
      set(kb.conflicts(b)["wheel"].get("button:3", [])) == {"weather_next", "hazard"})

print("\n=== 4. G923 button names ===")
lab = kb.load_labels()
check("stock names cover 25 buttons + 4 D-pad directions", len(lab["labels"]) == 29)
check("PS layout: X, Triangle, paddles, R2/L2, Options, R3/L3, Enter",
      [kb.label_for(s, lab["labels"]) for s in ("button:0", "button:3", "button:4", "button:5",
                                               "button:6", "button:7", "button:9", "button:10",
                                               "button:11", "button:23")]
      == ["Cross (X)", "Triangle", "R paddle", "L paddle", "R2", "L2", "Options", "R3", "L3", "Enter"])
check("unknown input -> 'unknown button', never a number",
      kb.label_for("button:31", lab["labels"]) == kb.UNKNOWN_EN
      and "31" not in kb.label_for("button:31", lab["labels"], "ja"))
check("Japanese names", kb.label_for("button:3", lab["labels"], "ja") == "△（三角）")
kb.set_label("button:2", "Triangle")               # the user says: this one is Triangle
lab = kb.load_labels()
check("renaming moves the name (names stay unique)",
      lab["labels"]["button:2"] == "Triangle" and "button:3" not in lab["labels"]
      and "button:2" in lab["confirmed"])
kb.confirm_label("button:0")
check("confirm marks a name right", "button:0" in kb.load_labels()["confirmed"])
kb.set_label("button:30", "Mystery")
check("a button without a stock name can be named",
      kb.load_labels()["labels"]["button:30"] == "Mystery")
rows = kb.help_rows(kb.defaults(), kb.load_labels()["labels"])
txt = " ".join(" ".join(r[1:]) for r in rows)
check("help rows name the wheel buttons", "L2" in txt and "R paddle" in txt and "Options" in txt)
check("help rows never print a raw button number", "button:" not in txt and "button 7" not in txt.lower())
kb.save_labels({"labels": dict(kb.DEFAULT_LABELS), "confirmed": []})

print("\n=== 5. the driving window uses the bindings ===")
import carla as real_carla  # noqa: E402
import pygame  # noqa: E402

spec = importlib.util.spec_from_file_location(
    "mc", os.path.join(HIL, "manual_control_steeringwheel G923.py"))
mc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mc)
mc.keybinds = kb                                    # same module, temp paths
for k in ("KEYBINDS_PATH", "LABELS_PATH", "WHEEL_LAST_PATH", "LEARN_PATH"):
    setattr(mc.keybinds, k, getattr(kb, k))
os.remove(P["keybinds.json"])

CALLS = []


class FakeVehicle(object):
    def get_velocity(self):
        return real_carla.Vector3D(0, 0, 0)

    def apply_control(self, c):
        self.last = c

    def set_autopilot(self, on, tm_port=8000):
        CALLS.append(("autopilot", on))

    def get_control(self):
        return real_carla.VehicleControl()

    def get_light_state(self):
        return real_carla.VehicleLightState.NONE

    def set_light_state(self, ls):
        CALLS.append(("lights", int(ls)))


class Rec(object):
    def __init__(self, prefix):
        self._p = prefix

    def __getattr__(self, name):
        if name.startswith("_") or name in ("look_input", "gauge"):
            raise AttributeError(name)
        return lambda *a, **k: CALLS.append((self._p + "." + name,) + a + tuple(sorted(k.items())))


class FakeHUD(Rec):
    def __init__(self):
        Rec.__init__(self, "hud")
        self.help = Rec("help")
        self.gauge = types.SimpleNamespace(gears=6)
        self.msgs = []

    def notification(self, text, seconds=2.0):
        self.msgs.append(text)


class FakeCam(Rec):
    look_input = 0.0


class FakeWorld(Rec):
    def __init__(self):
        Rec.__init__(self, "world")
        self.player = FakeVehicle()
        self.hud = FakeHUD()
        self.camera_manager = FakeCam("cam")


shim = types.ModuleType("carla")
for k in dir(real_carla):
    setattr(shim, k, getattr(real_carla, k))
shim.Vehicle = FakeVehicle
shim.Walker = type("Walker", (), {})
mc.carla = shim

pygame.init()
pygame.display.set_mode((64, 48))
MODS = [0]
EVENTS = []
mc.pygame.key.get_pressed = lambda: KS
mc.pygame.key.get_mods = lambda: MODS[0]
mc.pygame.event.get = lambda: [EVENTS.pop(0) for _ in range(len(EVENTS))]
mc.pygame.joystick.get_count = lambda: 0
PRESSED = set()


class KeyState(object):
    def __getitem__(self, k):
        return k in PRESSED


KS = KeyState()


class FakeJoy(object):
    def __init__(self):
        self.buttons, self.hat = set(), (0, 0)

    def get_numaxes(self):
        return 4

    def get_axis(self, i):
        return 1.0                                  # pedals at rest / ペダル解放

    def get_numbuttons(self):
        return 25

    def get_button(self, i):
        return 1 if i in self.buttons else 0

    def get_numhats(self):
        return 1

    def get_hat(self, i):
        return self.hat


world = FakeWorld()
ctrl = mc.DualControl(world, False)
clock = types.SimpleNamespace(get_time=lambda: 16)
check("keybinds.json is created at start", os.path.isfile(P["keybinds.json"]))


def key(k, mods=0):
    del CALLS[:]
    MODS[0] = mods
    EVENTS.append(pygame.event.Event(pygame.KEYUP, key=k, mod=mods))
    quit_ = ctrl.parse_events(world, clock)
    MODS[0] = 0
    return quit_


def button(n):
    del CALLS[:]
    EVENTS.append(pygame.event.Event(pygame.JOYBUTTONDOWN, button=n, joy=0, instance_id=0))
    return ctrl.parse_events(world, clock)


def names():
    return [c[0] for c in CALLS]


key(pygame.K_c)
check("C = next weather", ("world.next_weather",) in CALLS, str(CALLS))
key(pygame.K_c, pygame.KMOD_LSHIFT)
check("Shift+C = previous weather (most specific wins)",
      ("world.next_weather", ("reverse", True)) in CALLS, str(CALLS))
key(pygame.K_i, pygame.KMOD_LCTRL)
check("Ctrl+I = eye forward", ("cam.nudge_eye", mc.EYE_ACTIONS["eye_fwd"]) in CALLS, str(CALLS))
key(pygame.K_3)
check("3 = sensor 3", ("cam.set_sensor", 2) in CALLS, str(CALLS))
check("Ctrl+Q quits", key(pygame.K_q, pygame.KMOD_LCTRL) is True)
check("Esc quits", key(pygame.K_ESCAPE) is True)
g = ctrl._control.gear
key(pygame.K_q)
check("Q toggles reverse (and does not quit)", ctrl._control.gear == -1 and g != -1,
      "gear %d -> %d" % (g, ctrl._control.gear))
key(pygame.K_q)
key(pygame.K_RIGHTBRACKET, pygame.KMOD_LSHIFT)
check("Shift+] = brake strength", any("Brake strength" in m for m in world.hud.msgs[-1:]),
      str(world.hud.msgs[-1:]))

ctrl._joystick = FakeJoy()                          # now pretend the G923 is there
button(3)
check("Triangle (3) = next weather", ("world.next_weather",) in CALLS, str(CALLS))
button(0)
check("Cross (0) = next car", ("world.restart", ("step", 1)) in CALLS, str(CALLS))
button(2)
check("Circle (2) = next camera", ("cam.toggle_camera",) in CALLS, str(CALLS))
button(1)
check("Square (1) = HUD", ("hud.toggle_info",) in CALLS, str(CALLS))
button(23)
check("Enter (23) = next sensor", ("cam.next_sensor",) in CALLS, str(CALLS))
button(4)
check("R paddle (4) = gear up (switches to manual)", ctrl._control.manual_gear_shift
      and ctrl._control.gear >= 1)
ctrl._control.gear = 1
button(5)
check("L paddle (5) in 1st = reverse", ctrl._control.gear == -1 and ctrl._control.reverse,
      str(ctrl._control.gear))
button(5)
check("L paddle (5) in R stays in R", ctrl._control.gear == -1)
button(4)
check("R paddle (4) in R = 1st", ctrl._control.gear == 1 and not ctrl._control.reverse,
      str(ctrl._control.gear))
button(9)
check("Options (9) = back to automatic", not ctrl._control.manual_gear_shift)
button(11)
check("L3 (11) = left signal", world.hud.msgs[-1] == "Turn signal: LEFT", world.hud.msgs[-1])
button(11)
last = kb.read_wheel_last()
check("a press is reported in wheel_last.json", last and last["spec"] == "button:11"
      and time.time() - last["t"] < 5)
ctrl._joystick.buttons = {7}
ctrl.parse_events(world, clock)
check("L2 (7) held = handbrake", world.player.last.hand_brake is True)
ctrl._joystick.buttons = set()
ctrl.parse_events(world, clock)
check("L2 released = no handbrake", world.player.last.hand_brake is False)

# rebind in the file (what the dashboard does); picked up within ~1 s
b = kb.load()
b["weather_next"]["wheel"] = ["hat:0:up"]
b["hazard"]["wheel"] = ["button:3"]
b["glance_left"]["wheel"] = ["hat:0:left"]
b["mirrors"]["keys"] = ["f2"]
kb.save(b)
os.utime(P["keybinds.json"], (time.time() + 2, time.time() + 2))
for _ in range(31):
    ctrl.parse_events(world, clock)
check("keybinds.json changes are reloaded while driving",
      any("Keybinds updated" in m for m in world.hud.msgs), str(world.hud.msgs[-3:]))
button(3)
check("rebound Triangle = hazard lights", world.hud.msgs[-1] == "Hazard lights: ON", world.hud.msgs[-1])
check("hazard switches both blinkers on", ("lights", int(real_carla.VehicleLightState.LeftBlinker)
                                            | int(real_carla.VehicleLightState.RightBlinker)) in CALLS,
      str(CALLS))
button(3)
del CALLS[:]
EVENTS.append(pygame.event.Event(pygame.JOYHATMOTION, hat=0, value=(0, 1), joy=0, instance_id=0))
ctrl.parse_events(world, clock)
check("D-pad up = next weather after the rebind", ("world.next_weather",) in CALLS, str(CALLS))
del CALLS[:]
EVENTS.append(pygame.event.Event(pygame.JOYHATMOTION, hat=0, value=(1, 1), joy=0, instance_id=0))
ctrl.parse_events(world, clock)
check("holding the direction does not repeat", ("world.next_weather",) not in CALLS, str(CALLS))
ctrl._joystick.hat = (-1, 0)
ctrl.parse_events(world, clock)
check("D-pad left held = glance left", world.camera_manager.look_input == -1.0)
ctrl._joystick.hat = (0, 0)
key(pygame.K_F2)
check("F2 = mirrors after the rebind", ("cam.toggle_mirrors",) in CALLS, str(CALLS))
key(pygame.K_n)
check("N no longer toggles mirrors", ("cam.toggle_mirrors",) not in CALLS, str(CALLS))

kb.set_learning(30)
button(3)
check("learn mode: wheel press reported but not acted on",
      world.hud.msgs[-1] != "Hazard lights: ON" and kb.read_wheel_last()["spec"] == "button:3",
      world.hud.msgs[-1])
kb.set_learning(0)
button(3)
check("after learn mode the button works again", world.hud.msgs[-1] == "Hazard lights: ON",
      world.hud.msgs[-1])

hud = mc.HUD(1280, 720)
hud.help.set_rows(kb.help_rows(kb.load(), kb.load_labels()["labels"]))
check("the H help renders the bindings within the window",
      hud.help.dim[0] <= 1280 and hud.help.dim[1] <= 720, str(hud.help.dim))

pygame.quit()
print("\n%d checks failed" % len(FAILS))
for f in FAILS:
    print("  - " + f)
sys.exit(1 if FAILS else 0)
