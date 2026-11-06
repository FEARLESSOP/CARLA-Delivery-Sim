#!/usr/bin/env python3
"""weather_cycle.py - day/night cycle and dynamic weather.

  ../../carla-venv/bin/python weather_cycle.py      (run_pipeline.py starts it)
  ../../carla-venv/bin/python weather_cycle.py --test

A whole day runs in ``day_minutes`` real minutes (default 5). The sun follows
the clock: its altitude and azimuth come from the latitude (Oita, 33.2 N) and
today's date, so the nights are as long as they really are this season.
CARLA lights the street lamps by itself when the sun is down.

The weather comes in spells of 1.5-4 game hours: clear, partly cloudy,
cloudy, rain or storm (``rain_chance`` = share of rainy spells). Clouds,
rain and fog move smoothly towards each spell; puddles grow while it rains
and dry up after. Rain always comes through a cloudy spell. Wind is left
alone (seasons.py and delivery_sim.py set it).

Settings are read live from ``weather_cycle.json`` (the dashboard's "Day &
weather" card writes it); the state goes to ``data/weather_status.json``.
When it rains (precipitation >= ``rain_ground``), delivery_sim.py brings
every flying drone down and the pods keep theirs inside.

1 日を ``day_minutes`` 分（既定 5 分）で回す昼夜サイクルと変化する天気。
太陽高度は緯度と日付から計算。天気は 1.5〜4 時間（ゲーム内）ごとに
晴れ・薄曇り・曇り・雨・嵐が入れ替わる。風は変更しない。
設定は weather_cycle.json（ダッシュボード）、状態は data/weather_status.json。
"""

import argparse
import datetime
import json
import math
import os
import random
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SETTINGS_PATH = os.path.join(HERE, "weather_cycle.json")
STATUS_PATH = os.path.join(HERE, "data", "weather_status.json")

DEFAULTS = {
    "day_night": True,        # run the clock / 時計を進める
    "day_minutes": 5.0,       # real minutes per 24 h / 24 時間あたりの実時間（分）
    "paused": False,          # clock stopped (the sun stays) / 時計を止める
    "dynamic": True,          # weather changes by itself / 天気が変化する
    "rain_chance": 0.25,      # share of weather spells that bring rain / 雨の確率
    "force": "auto",          # auto | clear | partly | cloudy | rain | storm
    "rain_ground": 20.0,      # precipitation at which drones come down / ドローンが落ちる雨量
    "lat": 33.23,             # Oita / 大分
    "clock_cmd": None,        # {"hhmm": "06:00", "seq": n}: jump the clock once / 時刻の変更依頼
    "lamps": "auto",          # street lamps: auto (on when dark) | on | off / 街灯
    "lamp_lumens": 150000.0,  # street lamp brightness: our lamps hang at 10 m / 街灯の明るさ
}
LAMP_SUN = 3.0                # lamps on below this sun altitude (deg) / この太陽高度未満で点灯
LAMP_GLOOM = 60.0             # ... or in rain / fog this heavy / 強い雨・霧でも点灯

# cloudiness, precipitation, fog density per spell / 天気ごとの雲量・雨量・霧
STATES = {
    "clear": (10.0, 0.0, 2.0),
    "partly": (40.0, 0.0, 3.0),
    "cloudy": (80.0, 0.0, 6.0),
    "rain": (90.0, 55.0, 12.0),
    "storm": (100.0, 90.0, 18.0),
}
RAINY = ("rain", "storm")
SPELL_H = (1.5, 4.0)                    # game hours per spell / 1 回の天気の長さ
RATE = {"cloud": 60.0, "rain": 90.0, "fog": 20.0}     # change per game hour / 1 時間あたりの変化量
PUDDLE_UP, PUDDLE_DRY = 0.9, 30.0       # deposits: x precipitation / h up, per h down


def load_settings(path=SETTINGS_PATH):
    s = dict(DEFAULTS)
    try:
        with open(path) as fh:
            s.update({k: v for k, v in json.load(fh).items() if k in DEFAULTS})
    except (OSError, ValueError):
        pass
    s["day_minutes"] = max(1.0, min(1440.0, float(s["day_minutes"])))
    s["rain_chance"] = max(0.0, min(1.0, float(s["rain_chance"])))
    if s["force"] not in ("auto",) + tuple(STATES):
        s["force"] = "auto"
    if s["lamps"] not in ("auto", "on", "off"):
        s["lamps"] = "auto"
    s["lamp_lumens"] = max(5000.0, min(500000.0, float(s["lamp_lumens"])))
    return s


def lamps_wanted(s, sun_alt, rain, fog):
    """Street lamps on? Dusk to dawn, or heavy rain / fog, unless forced.
    街灯を点けるか（夕方から朝、強い雨・霧）。"""
    if s["lamps"] != "auto":
        return s["lamps"] == "on"
    return sun_alt < LAMP_SUN or rain >= LAMP_GLOOM or fog >= LAMP_GLOOM


def save_settings(s, path=SETTINGS_PATH):
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump({k: s.get(k, v) for k, v in DEFAULTS.items()}, fh, indent=2)
    os.replace(tmp, path)


def sun(hours, lat_deg, day_of_year):
    """(altitude, azimuth) in degrees for local solar time ``hours``.
    Azimuth from north, clockwise. / 太陽高度と方位（北から時計回り）。"""
    decl = math.radians(23.44) * math.sin(2 * math.pi * (284 + day_of_year) / 365.0)
    lat = math.radians(lat_deg)
    h = math.radians(15.0 * (hours - 12.0))
    alt = math.asin(math.sin(lat) * math.sin(decl) + math.cos(lat) * math.cos(decl) * math.cos(h))
    az = math.atan2(math.sin(h), math.cos(h) * math.sin(lat) - math.tan(decl) * math.cos(lat))
    return math.degrees(alt), (math.degrees(az) + 180.0) % 360.0


def approach(v, target, rate, dt_h):
    step = rate * dt_h
    return target if abs(target - v) <= step else v + math.copysign(step, target - v)


class Cycle(object):
    """The game clock and the weather, without CARLA (testable).
    時計と天気（CARLA 無しで動く）。"""

    def __init__(self, hours=8.0, state="clear", rng=None, day_of_year=None):
        self.hours = hours % 24.0
        self.state = self.target = state
        self.cloud, self.rain, self.fog = STATES[state]
        self.puddles = 0.0
        self.spell_left = 2.0
        self.rng = rng or random.Random()
        self.doy = day_of_year or datetime.date.today().timetuple().tm_yday
        self.clock_seq = None

    def next_spell(self, s):
        """Pick the next weather. Rain never follows clear directly.
        次の天気。晴れから直接雨にはならない。"""
        if self.rng.random() < s["rain_chance"]:
            nxt = "storm" if self.rng.random() < 0.2 else "rain"
            if self.target not in ("cloudy",) + RAINY:
                nxt = "cloudy"                   # clouds gather first / まず曇る
                self.spell_left = 0.7
                return nxt
        else:
            nxt = self.rng.choice(("clear", "clear", "partly", "cloudy"))
        self.spell_left = self.rng.uniform(*SPELL_H)
        return nxt

    def step(self, dt_real, s):
        """Advance ``dt_real`` real seconds under settings ``s``.
        Returns the game hours that passed. / 実時間 dt_real 秒進める。"""
        cmd = s.get("clock_cmd")
        if isinstance(cmd, dict) and cmd.get("seq") != self.clock_seq:
            self.clock_seq = cmd.get("seq")
            try:
                hh, mm = str(cmd.get("hhmm", "")).split(":")
                self.hours = (int(hh) + int(mm) / 60.0) % 24.0
            except ValueError:
                pass
        dt_h = dt_real * 24.0 / (s["day_minutes"] * 60.0)
        if s["day_night"] and not s["paused"]:
            self.hours = (self.hours + dt_h) % 24.0
        if s["force"] != "auto":
            self.target = s["force"]
        elif s["dynamic"]:
            self.spell_left -= dt_h
            if self.spell_left <= 0.0:
                self.target = self.next_spell(s)
        tc, tr, tf = STATES[self.target]
        # rain stops before the clouds clear / 雲より先に雨が止む
        self.rain = approach(self.rain, tr, RATE["rain"], dt_h)
        if tr > 0.0 or self.rain <= 0.0:
            self.cloud = approach(self.cloud, tc, RATE["cloud"], dt_h)
        self.fog = approach(self.fog, tf, RATE["fog"], dt_h)
        if self.rain > 0.0:
            self.puddles = min(100.0, self.puddles + self.rain * PUDDLE_UP * dt_h)
        else:
            self.puddles = max(0.0, self.puddles - PUDDLE_DRY * dt_h)
        self.state = self.target if abs(self.rain - tr) < 1.0 and abs(self.cloud - tc) < 2.0 else self.state
        return dt_h

    def apply(self, w, s):
        """Write the clock and weather into a carla.WeatherParameters (wind
        untouched). / 天気パラメータに書き込む（風はそのまま）。"""
        if s["day_night"]:
            alt, az = sun(self.hours, s["lat"], self.doy)
            w.sun_altitude_angle = alt
            w.sun_azimuth_angle = az
        if s["dynamic"] or s["force"] != "auto":
            w.cloudiness = self.cloud
            w.precipitation = self.rain
            w.precipitation_deposits = self.puddles
            w.wetness = max(self.rain, self.puddles * 0.6)
            w.fog_density = self.fog
            w.fog_distance = 60.0 if self.rain > 30 else 0.0
        return w

    def status(self, s):
        alt, _ = sun(self.hours, s["lat"], self.doy)
        h = int(self.hours)
        return {
            "clock": "%02d:%02d" % (h, int((self.hours - h) * 60)),
            "hours": round(self.hours, 3), "sun_alt": round(alt, 1), "night": alt < 0.0,
            "state": self.state, "target": self.target,
            "cloud": round(self.cloud, 1), "rain": round(self.rain, 1),
            "puddles": round(self.puddles, 1), "fog": round(self.fog, 1),
            "raining": self.rain >= s["rain_ground"],
            "day_night": s["day_night"], "dynamic": s["dynamic"], "paused": s["paused"],
            "day_minutes": s["day_minutes"], "rain_chance": s["rain_chance"], "force": s["force"],
            "lamps": s["lamps"],
            "t": time.time(),
        }


def switch_lamps(world, lamps, want, lumens=None):
    """Street lamps (CARLA's LightManager, group Street: our BP_JpGaito and the
    towns' own) on or off when that changes, and again when the map reloads.
    CARLA's own day/night switching is turned off so the two never disagree.
    街灯を点灯・消灯（変化時とマップ再読込時）。CARLA 側の自動切替は止める。"""
    import carla
    wid = world.id
    if lamps["world"] != wid or time.time() - lamps["t"] > 30.0:
        lm = world.get_lightmanager()
        lm.set_day_night_cycle(False)
        lamps["ids"] = list(lm.get_all_lights(carla.LightGroup.Street))
        if lamps["world"] != wid:
            lamps["on"] = lamps["lumens"] = None
        lamps["world"], lamps["t"] = wid, time.time()
    if lamps["ids"] and lumens and lumens != lamps["lumens"]:
        # CARLA's 20000 lm is for its own low lamps; ours hang at 10 m
        # CARLA の既定は低い街灯向け。自作の街灯は 10 m の高さ
        world.get_lightmanager().set_intensity(lamps["ids"], float(lumens))
        lamps["lumens"] = lumens
    if lamps["ids"] and want != lamps["on"]:
        lm = world.get_lightmanager()
        (lm.turn_on if want else lm.turn_off)(lamps["ids"])
        lamps["on"] = want
        print("[weather] street lamps %s (%d) / 街灯%s" % ("on" if want else "off", len(lamps["ids"]),
                                                            "点灯" if want else "消灯"))


def write_status(st, path=STATUS_PATH):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(st, fh)
    os.replace(tmp, path)


def last_clock(path=STATUS_PATH):
    """Carry the clock over a restart. / 再起動前の時刻から続ける。"""
    try:
        with open(path) as fh:
            return float(json.load(fh)["hours"])
    except (OSError, ValueError, KeyError, TypeError):
        return 8.0


def self_test():
    s = dict(DEFAULTS)
    alt_noon, az_noon = sun(12.0, 33.23, 172)
    alt_mid, _ = sun(0.0, 33.23, 172)
    assert 75 < alt_noon < 82 and abs(az_noon - 180) < 1 and alt_mid < -20, (alt_noon, az_noon, alt_mid)
    assert sun(6.0, 33.23, 80)[0] < 2 and sun(9.0, 33.23, 80)[1] < 180      # equinox sunrise, morning sun in the east
    c = Cycle(hours=23.5, rng=random.Random(1), day_of_year=172)
    c.step(5.0 * 60 / 24.0, s)                       # 1 game hour at 5 min/day / 1 時間
    assert abs(c.hours - 0.5) < 1e-6, c.hours
    p = dict(s, paused=True)
    c.step(60.0, p)
    assert abs(c.hours - 0.5) < 1e-6
    c.step(0.0, dict(s, clock_cmd={"hhmm": "06:30", "seq": 1}))
    assert abs(c.hours - 6.5) < 1e-6
    c.step(10.0, dict(s, clock_cmd={"hhmm": "01:00", "seq": 1}))   # same seq: not again / 同じ seq は無視
    assert c.hours > 6.5
    # forced rain: clouds first, rain reaches 55, puddles grow, then dry / 強制雨
    f = dict(s, force="rain")
    for _ in range(200):
        c.step(1.0, f)
    assert c.rain == 55.0 and c.cloud == 90.0 and c.puddles > 40 and c.state == "rain"
    assert c.status(f)["raining"]
    f = dict(s, force="clear")
    c.step(25.0, f)                                  # ~2 game hours / 約 2 時間
    assert c.rain == 0.0 and c.puddles < 100
    for _ in range(400):
        c.step(1.0, f)
    assert c.puddles == 0.0 and c.cloud == 10.0 and not c.status(f)["raining"]
    # dynamic: over many days some rain, never rain straight after clear
    # 長期: 雨が降ることがあり、晴れの直後に雨にならない
    c = Cycle(rng=random.Random(7), day_of_year=172)
    seen, prev, bad = set(), c.target, 0
    for _ in range(20000):
        c.step(1.0, s)
        if c.target != prev:
            bad += prev in ("clear", "partly") and c.target in RAINY
            prev = c.target
        seen.add(c.target)
    assert {"clear", "cloudy", "rain"} <= seen and bad == 0, (seen, bad)
    c = Cycle(rng=random.Random(7))
    for _ in range(20000):
        c.step(1.0, dict(s, rain_chance=0.0))
        assert c.target not in RAINY
    assert load_settings("/nonexistent")["day_minutes"] == 5.0
    assert lamps_wanted(s, -10, 0, 0) and not lamps_wanted(s, 30, 0, 0) and lamps_wanted(s, 30, 90, 0)
    assert lamps_wanted(dict(s, lamps="on"), 30, 0, 0) and not lamps_wanted(dict(s, lamps="off"), -30, 0, 0)
    print("weather_cycle self-test OK / 自己テスト OK")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--period", type=float, default=0.25, help="seconds between updates / 更新間隔")
    ap.add_argument("--test", action="store_true")
    a = ap.parse_args()
    if a.test:
        self_test()
        return
    import carla
    c = Cycle(hours=last_clock())
    cmd = load_settings().get("clock_cmd")
    c.clock_seq = cmd.get("seq") if isinstance(cmd, dict) else None   # already done before a restart / 再起動前の依頼は済み
    client = carla.Client(a.host, a.port)
    client.set_timeout(20.0)
    last, world, said = time.time(), None, None
    lamps = {"world": None, "ids": [], "on": None, "t": 0.0, "lumens": None}
    while True:
        s = load_settings()
        now = time.time()
        c.step(now - last, s)
        last = now
        try:
            if world is None:
                world = client.get_world()
            w = world.get_weather()
            if s["day_night"] or s["dynamic"] or s["force"] != "auto":
                w = c.apply(w, s)
                world.set_weather(w)
            switch_lamps(world, lamps, lamps_wanted(s, w.sun_altitude_angle, w.precipitation, w.fog_density),
                         s["lamp_lumens"])
        except RuntimeError as exc:          # server busy or loading a map / 応答なし・マップ読込中
            world = None
            print("[weather] no answer from the server (%s), retrying / 再試行" % exc)
            time.sleep(5.0)
            last = time.time()
            continue
        st = c.status(s)
        st["lamps_on"], st["lamps_n"] = lamps["on"], len(lamps["ids"])
        write_status(st)
        if st["target"] != said:
            print("[weather] %s -> %s (%s) / 天気" % (st["clock"], st["target"], "rain" if st["raining"] else "dry"))
            said = st["target"]
        time.sleep(a.period)


if __name__ == "__main__":
    main()
