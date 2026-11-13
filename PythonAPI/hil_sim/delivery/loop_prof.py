"""Main-loop timing for delivery_sim.py, shared with delivery/* so a phase
can be broken down where its code lives. No carla import.
delivery_sim.py のメインループ計測。delivery/* から内訳を追加できるよう共有する。"""
import collections
import time


class LoopProfile(object):
    """Where the main loop's time goes: every ``PERIOD`` s prints the loop
    rate and the mean ms of each phase (a slow loop makes the drones, which
    this script poses, move in jumps). / メインループの各段階の所要時間を
    定期的に表示（ループが遅いとドローンがカクつく）。"""
    PERIOD = 20.0

    def __init__(self):
        self.acc, self.n = collections.OrderedDict(), 0
        self.t_last = self.t_print = time.perf_counter()

    def mark(self, name):
        now = time.perf_counter()
        self.acc[name] = self.acc.get(name, 0.0) + (now - self.t_last)
        self.t_last = now

    def add(self, name, secs):
        """Time spent inside a phase (shown in brackets, already counted in
        that phase). / 段階内の内訳（その段階の時間に含まれる）。"""
        self.acc[name] = self.acc.get(name, 0.0) + secs

    def loop_done(self):
        self.n += 1
        now = time.perf_counter()
        if now - self.t_print >= self.PERIOD and self.n:
            span = now - self.t_print
            print("[loop] %.1f Hz | %s / ループ時間" % (self.n / span, ", ".join(
                "%s %.1f ms" % (k, v * 1000.0 / self.n) for k, v in self.acc.items())),
                flush=True)
            slow = SLOW.report(span)
            if slow:
                print("[loop] blocking CARLA calls / 待ちの出る呼び出し: " + slow, flush=True)
            self.acc.clear()
            self.n, self.t_print = 0, now


LOOP_PROF = LoopProfile()


class SlowCalls(object):
    """Counts CARLA calls that block the main loop (> ``MIN_MS``) and where
    they were made, so the ``[loop]`` line can name them. In 0.9.16 any call
    that returns data from the server (cast_ray, get_weather, a fresh actor's
    bounding_box ...) waits ~30 ms when it is the first in a tick; one per tick
    keeps the loop -- and the drones it poses -- below the frame rate.
    メインループを止める CARLA 呼び出し（> MIN_MS）と呼び出し元を数え、[loop] 行に
    表示する。0.9.16 ではデータを返す呼び出しが tick の最初だと約 30 ms 待つ。"""
    MIN_MS = 5.0
    WORLD = ("cast_ray", "get_weather", "get_actors", "get_environment_objects",
             "get_level_bbs", "get_map", "get_settings", "get_spectator",
             "get_traffic_light", "get_lightmanager", "project_point",
             "ground_projection")

    def __init__(self):
        self.hits = collections.Counter()
        self.ms = collections.Counter()

    def install(self, carla, main_thread):
        import sys
        import threading

        def timed(name, fn):
            def wrapper(*a, **k):
                if threading.current_thread() is not main_thread:
                    return fn(*a, **k)
                t0 = time.perf_counter()
                try:
                    return fn(*a, **k)
                finally:
                    dt = (time.perf_counter() - t0) * 1000.0
                    if dt > self.MIN_MS:
                        f = sys._getframe(1)
                        key = "%s@%s:%d" % (name, f.f_code.co_filename.rsplit("/", 1)[-1],
                                            f.f_lineno)
                        self.hits[key] += 1
                        self.ms[key] += dt
            return wrapper

        for n in self.WORLD:
            fn = getattr(carla.World, n, None)
            if fn is not None:
                setattr(carla.World, n, timed(n, fn))
        bb = carla.Actor.bounding_box
        if isinstance(bb, property):
            carla.Actor.bounding_box = property(timed("bounding_box", bb.fget))

    def report(self, seconds):
        """Top callers since the last report, per second. / 上位の呼び出し元。"""
        out = ", ".join("%s %.1f/s %.0f ms each" % (k, self.hits[k] / seconds,
                                                    v / self.hits[k])
                        for k, v in self.ms.most_common(3))
        self.hits.clear()
        self.ms.clear()
        return out


SLOW = SlowCalls()
