"""Headless functional test of the WASD keyboard driving path.

Loads the real manual-control script and the real carla client types, then
drives DualControl with synthetic key state and asserts on the
carla.VehicleControl actually applied to the vehicle. No CARLA server needed.
"""
import importlib.util
import math
import os
import sys
import types

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

HIL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HIL)
os.chdir(HIL)

import carla as real_carla
import pygame

spec = importlib.util.spec_from_file_location(
    "mc", os.path.join(HIL, "manual_control_steeringwheel G923.py"))
mc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mc)

FAILS = []
def check(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (" | " + detail if detail else ""))
    if not cond:
        FAILS.append(name)


# --- fakes standing in for the CARLA server-side actors ------------------
class FakeVehicle(object):
    def __init__(self):
        self.applied = []
        self.speed = 0.0          # m/s, set by the test
    def get_velocity(self):
        return real_carla.Vector3D(self.speed, 0.0, 0.0)
    def apply_control(self, c):
        self.applied.append(c)
    def set_autopilot(self, on):
        pass
    def get_control(self):
        return real_carla.VehicleControl()

class FakeHUD(object):
    def __init__(self):
        self.msgs = []
    def notification(self, text, seconds=2.0):
        self.msgs.append(text)

class FakeCam(object):
    look_input = 0.0

class FakeWorld(object):
    def __init__(self):
        self.player = FakeVehicle()
        self.hud = FakeHUD()
        self.camera_manager = FakeCam()

# carla shim so isinstance(player, carla.Vehicle) accepts the fake
shim = types.ModuleType("carla")
for k in dir(real_carla):
    setattr(shim, k, getattr(real_carla, k))
shim.Vehicle = FakeVehicle
shim.Walker = type("Walker", (), {})
mc.carla = shim

pygame.init()
pygame.display.set_mode((64, 48))

# synthetic keyboard state
PRESSED = set()
class KeyState(object):
    def __getitem__(self, k):
        return k in PRESSED
mc.pygame.key.get_pressed = lambda: KeyState()
mc.pygame.key.get_mods = lambda: 0
mc.pygame.event.get = lambda: []

K = mc
world = FakeWorld()


def drive(frames=1, ms=16, **kw):
    """Run parse_events for `frames` frames and return the last control."""
    world.player.applied = []
    clock = types.SimpleNamespace(get_time=lambda: ms)
    for _ in range(frames):
        ctrl.parse_events(world, clock)
    return world.player.applied[-1]


print("\n=== 1. no wheel attached -> keyboard fallback, no crash ===")
try:
    ctrl = mc.DualControl(world, False)
    check("DualControl constructs without a wheel", True)
except Exception as e:
    check("DualControl constructs without a wheel", False, repr(e))
    raise SystemExit(1)
check("keyboard mode active", ctrl._keyboard is True and ctrl._joystick is None)

print("\n=== 2. W = throttle ===")
PRESSED = {K.K_w}
c = drive(frames=40)                      # ~0.64 s of held W
check("W gives throttle", c.throttle > 0.5, "throttle=%.3f" % c.throttle)
check("W applies no brake", c.brake == 0.0, "brake=%.3f" % c.brake)
check("throttle respects throttle_max cap",
      c.throttle <= ctrl._tuning['throttle_max'] + 1e-6,
      "throttle=%.3f cap=%.2f" % (c.throttle, ctrl._tuning['throttle_max']))
check("forward gear engaged", c.gear == 1 and c.reverse is False, "gear=%d" % c.gear)

print("\n=== 3. throttle ramps, does not slam to full ===")
PRESSED = set(); drive(frames=40)         # release, settle
PRESSED = {K.K_w}
c1 = drive(frames=1)
check("one frame of W is a partial press", 0.0 < c1.throttle < 0.3,
      "throttle after 16 ms = %.3f" % c1.throttle)

print("\n=== 4. A / D = steering ===")
PRESSED = set(); drive(frames=60)
PRESSED = {K.K_a}
c = drive(frames=30)
check("A steers left (negative)", c.steer < -0.1, "steer=%.3f" % c.steer)
left = c.steer
PRESSED = {K.K_d}
c = drive(frames=60)
check("D steers right (positive)", c.steer > 0.1, "steer=%.3f" % c.steer)
PRESSED = set()
c = drive(frames=80)
check("releasing A/D re-centres the wheel", abs(c.steer) < 1e-6,
      "steer=%.3f" % c.steer)
PRESSED = {K.K_a}
c = drive(frames=400)
check("steering is clamped", c.steer >= -0.7001, "steer=%.3f" % c.steer)

print("\n=== 5. S = brake while moving, reverse once stopped ===")
PRESSED = set(); drive(frames=80)
world.player.speed = 8.0                  # 8 m/s forward
PRESSED = {K.K_s}
c = drive(frames=30)
check("S brakes while moving", c.brake > 0.3 and c.gear == 1,
      "brake=%.3f gear=%d" % (c.brake, c.gear))
check("S does not apply throttle while moving", c.throttle == 0.0)
world.player.speed = 0.0                  # now stopped
c = drive(frames=40)
check("S engages reverse once stopped", c.gear == -1 and c.reverse is True,
      "gear=%d reverse=%s" % (c.gear, c.reverse))
check("S gives throttle in reverse", c.throttle > 0.5,
      "throttle=%.3f" % c.throttle)

print("\n=== 6. W brakes while reversing, then goes forward ===")
world.player.speed = 3.0
PRESSED = {K.K_w}
c = drive(frames=20)
check("W brakes while still reversing", c.brake > 0.2 and c.gear == -1,
      "brake=%.3f gear=%d" % (c.brake, c.gear))
world.player.speed = 0.0
c = drive(frames=40)
check("W returns to forward gear", c.gear == 1 and c.throttle > 0.5,
      "gear=%d throttle=%.3f" % (c.gear, c.throttle))

print("\n=== 7. SPACE = handbrake ===")
PRESSED = {K.K_SPACE}
c = drive(frames=5)
check("SPACE sets handbrake", c.hand_brake is True)
PRESSED = set()
c = drive(frames=5)
check("releasing SPACE clears handbrake", c.hand_brake is False)

print("\n=== 8. arrow keys still work ===")
PRESSED = set(); drive(frames=80)
PRESSED = {K.K_UP}
c = drive(frames=40)
check("Up arrow = throttle", c.throttle > 0.5, "throttle=%.3f" % c.throttle)
PRESSED = {K.K_LEFT}
c = drive(frames=30)
check("Left arrow = steer left", c.steer < -0.1, "steer=%.3f" % c.steer)

print("\n=== 9. pedal strength cap is honoured (] / [ and dashboard) ===")
ctrl._tuning['throttle_max'] = 0.4
PRESSED = set(); drive(frames=80)
PRESSED = {K.K_w}
c = drive(frames=60)
check("lowering throttle_max lowers output", abs(c.throttle - 0.4) < 1e-6,
      "throttle=%.3f" % c.throttle)
ctrl._tuning['throttle_max'] = 0.8

print("\n=== 10. --keyboard forces keyboard mode ===")
w2 = FakeWorld()
ctrl2 = mc.DualControl(w2, False, keyboard=True)
check("keyboard=True selects keyboard mode",
      ctrl2._keyboard is True and ctrl2._joystick is None)

print("\n=== 11. wheel path untouched when a joystick is present ===")
import inspect
src = inspect.getsource(mc.DualControl.parse_events)
check("wheel is read only when a joystick exists",
      "if self._joystick is not None:" in src and "_parse_vehicle_wheel()" in src)

pygame.quit()
print("\n%d checks failed" % len(FAILS))
for f in FAILS:
    print("  - " + f)
sys.exit(1 if FAILS else 0)
