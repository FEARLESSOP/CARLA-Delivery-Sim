"""Shared driving rules: driver-alert thresholds and RL reward terms.

ONE rule set for both sides of the HIL study (docs/RL_DESIGN.md §3):
the G923 script imports ALERT_RULES from here to warn the human driver,
and the Gym environment imports REWARD_RULES + step_reward() to score the
RL agent. Changing a threshold here changes both, so "bad driving" always
means the same thing for human feedback and agent training.

運転ルールの共有モジュール：運転警告のしきい値とRL報酬の定義
（docs/RL_DESIGN.md §3）。G923スクリプトは ALERT_RULES を、Gym環境は
REWARD_RULES と step_reward() をここから読み込む。しきい値をここで
変えれば人間側の警告とエージェント側の報酬が同時に変わり、
「悪い運転」の定義が常に一致する。
"""

# Driver-alert thresholds (also used as reward triggers).
# 運転警告のしきい値（報酬の発火条件としても使用）。
ALERT_RULES = {
    'speeding_kmh_over': 10.0,   # km/h above limit / 制限速度からの超過量
    'speeding_hold_s': 2.0,      # sustained before alerting / 警告までの継続秒数
    'fcw_ttc_s': 1.5,            # forward-collision TTC / 前方衝突警告のTTC
    'fcw_min_kmh': 20.0,         # FCW only above this speed / FCW有効の最低速度
    'thw_warn_s': 1.0,           # tailgating headway / 車間時間警告
}

# RL reward terms (docs/RL_DESIGN.md §3 table). Per-step terms assume the
# 20 Hz synchronous tick — rescale if fixed_delta_seconds changes.
# RL報酬項（docs/RL_DESIGN.md §3の表）。ステップ毎の項は20Hz同期Tick前提。
# fixed_delta_seconds を変えた場合は要スケール調整。
REWARD_RULES = {
    'collision': -200.0,          # + end episode / エピソード終了
    'red_light': -100.0,
    'off_road': -50.0,            # once on leaving road / コース外に出た時に1回
    'off_road_end_s': 3.0,        # end episode after this long off-road / この秒数コース外で終了
    'solid_cross': -20.0,
    'broken_cross': -5.0,
    'fcw_scale': -10.0,           # x (1.5-TTC)/1.5 per step / ステップ毎
    'thw_scale': -2.0,            # x (1.0-THW) per step / ステップ毎
    'speeding_scale': -0.5,       # x over_kmh/10 per step / ステップ毎
    'hard_brake': -2.0,           # per event (comfort) / イベント毎（乗り心地）
    'hard_brake_brake': 0.7,      # brake input threshold / ブレーキ入力しきい値
    'hard_brake_decel': 4.0,      # m/s^2 deceleration threshold / 減速度しきい値
    'jerk_scale': -0.1,           # x |jerk| per step above thresh / しきい値超過時ステップ毎
    'jerk_thresh': 5.0,           # m/s^3
    'jerk_cap': 20.0,             # clamp |jerk| for the penalty: single-tick
                                  # physics artifacts (gear shifts, contacts)
                                  # spike to ~100 m/s^3 and would dominate.
                                  # ペナルティ計算用の上限。ギアチェンジ等の
                                  # 物理アーティファクトは1Tickで約100 m/s^3
                                  # に跳ね、報酬を支配してしまうため制限する。
    'lane_keep_scale': 0.5,       # x (1-|offset|/(width/2)) per step / ステップ毎
    'progress_scale': 0.1,        # x route metres gained / 経路前進1mあたり
    'completion': 100.0,          # route finished / 完走
}


def step_reward(m, ev):
    """Compute one step's reward. Returns (reward, breakdown dict).

    `m` — continuous measurements this step / このステップの連続量:
        speed_kmh, speed_limit_kmh, ttc (s, -1 = none), thw (s, -1 = none),
        jerk (m/s^3), lane_offset (m, signed), lane_width (m), on_road (bool),
        progress_m (route metres gained this step / このステップで進んだ経路距離)
    `ev` — discrete events this step (bools) / このステップの離散イベント:
        collision, red_light, left_road (just left the road / 直前にコース外へ),
        crossed_solid, crossed_broken, hard_brake, route_done

    Episode termination is the caller's job (collision, off-road > 3 s,
    completion, timeout) — this function only scores.
    エピソード終了判定（衝突・3秒超のコース外・完走・タイムアウト）は
    呼び出し側の責務。この関数は採点のみを行う。
    """
    R = REWARD_RULES
    b = {}

    # --- Event penalties / イベントペナルティ --------------------------
    if ev.get('collision'):
        b['collision'] = R['collision']
    if ev.get('red_light'):
        b['red_light'] = R['red_light']
    if ev.get('left_road'):
        b['off_road'] = R['off_road']
    if ev.get('crossed_solid'):
        b['solid_cross'] = R['solid_cross']
    if ev.get('crossed_broken'):
        b['broken_cross'] = R['broken_cross']
    if ev.get('hard_brake'):
        b['hard_brake'] = R['hard_brake']
    if ev.get('route_done'):
        b['completion'] = R['completion']

    # --- Per-step penalties / ステップ毎ペナルティ ----------------------
    ttc = m.get('ttc', -1.0)
    if (0.0 < ttc < ALERT_RULES['fcw_ttc_s']
            and m.get('speed_kmh', 0.0) > ALERT_RULES['fcw_min_kmh']):
        b['fcw'] = R['fcw_scale'] * (
            (ALERT_RULES['fcw_ttc_s'] - ttc) / ALERT_RULES['fcw_ttc_s'])
    thw = m.get('thw', -1.0)
    if 0.0 < thw < ALERT_RULES['thw_warn_s']:
        b['thw'] = R['thw_scale'] * (ALERT_RULES['thw_warn_s'] - thw)
    over = (m.get('speed_kmh', 0.0)
            - m.get('speed_limit_kmh', 0.0) - ALERT_RULES['speeding_kmh_over'])
    if m.get('speed_limit_kmh', 0.0) > 0 and over > 0:
        b['speeding'] = R['speeding_scale'] * over / 10.0
    jerk = min(abs(m.get('jerk', 0.0)), R['jerk_cap'])
    if jerk > R['jerk_thresh']:
        b['jerk'] = R['jerk_scale'] * jerk

    # --- Per-step rewards / ステップ毎報酬 ------------------------------
    width = m.get('lane_width', 0.0)
    if m.get('on_road') and width > 0.1:
        off = min(1.0, abs(m.get('lane_offset', 0.0)) / (width / 2.0))
        b['lane_keep'] = R['lane_keep_scale'] * (1.0 - off)
    prog = m.get('progress_m', 0.0)
    if prog:
        b['progress'] = R['progress_scale'] * prog

    return sum(b.values()), b
