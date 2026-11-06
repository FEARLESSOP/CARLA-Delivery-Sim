import { registerDict } from '@/lib/i18n'

// Statistics page (current session tiles, session history chart, saved
// sessions). Keys copied from the old I18N object in dashboard.py.
// 統計ページの文言
registerDict({
  tile_crash: 'Crashes', tile_lane: 'Out of lane', tile_rlv: 'Red lights run',
  tile_hb: 'Hard brakes', tile_dist: 'Distance', tile_dur: 'Duration',
  tile_avg: 'Avg speed', tile_comp: 'Compliance',
  stats_session: 'Current session', stats_history: 'Session history',
  hist_hint: 'crashes & out-of-lane per session (latest 20)',
  stats_sessions: 'Saved sessions', stats_empty: 'No saved sessions yet',
  th_time: 'Time', th_map: 'Map', th_dur: 'Duration', th_dist: 'Distance (km)',
  th_crash: 'Crashes', th_lane: 'Out of lane', th_rlv: 'Red lights',
  th_hb: 'Hard brakes', th_comp: 'Compliance', badge_live: 'LIVE',
  // new in the React port / React 版で追加
  st_session_h: 'Live counters of the drive in progress.',
  st_no_live: 'No car connected: the counters show the last values.',
  st_sessions_h: 'Every run saved in logs/ (newest first). Refreshed every 10 s.',
  st_dl_csv: 'Download the current session CSV',
  st_kind: 'Show', st_kind_all: 'All', st_kind_drive: 'Driving', st_kind_delivery: 'Delivery',
  st_delivery: 'delivery', st_sort_by: 'Sort by {c}', st_count: '{n} sessions',
  st_loading: 'Loading sessions…', st_error: 'Could not load the sessions.',
  st_hist_empty: 'No sessions to chart yet.',
  st_total_km: 'Total distance', st_total_time: 'Total time', st_total_crash: 'Total crashes',
  st_more: 'Show {n} more', st_unknown_date: 'unknown date',
}, {
  tile_crash: '衝突回数', tile_lane: '車線逸脱回数', tile_rlv: '信号無視',
  tile_hb: '急ブレーキ', tile_dist: '走行距離', tile_dur: '走行時間',
  tile_avg: '平均速度', tile_comp: '速度遵守率',
  stats_session: '現在のセッション', stats_history: 'セッション履歴',
  hist_hint: 'セッション毎の衝突・車線逸脱（最新20件）',
  stats_sessions: '保存済みセッション', stats_empty: '保存されたセッションはまだありません',
  th_time: '時刻', th_map: 'マップ', th_dur: '時間', th_dist: '距離 (km)',
  th_crash: '衝突', th_lane: '車線逸脱', th_rlv: '信号無視',
  th_hb: '急ブレーキ', th_comp: '遵守率', badge_live: '走行中',
  st_session_h: '走行中のセッションのカウンター。',
  st_no_live: '車が接続されていません（最後の値を表示）。',
  st_sessions_h: 'logs/ に保存された走行（新しい順）。10 秒ごとに更新。',
  st_dl_csv: '現在のセッションの CSV をダウンロード',
  st_kind: '表示', st_kind_all: 'すべて', st_kind_drive: '運転', st_kind_delivery: '配送',
  st_delivery: '配送', st_sort_by: '{c} で並べ替え', st_count: '{n} 件',
  st_loading: 'セッションを読み込み中…', st_error: 'セッションを読み込めませんでした。',
  st_hist_empty: 'グラフにするセッションがまだありません。',
  st_total_km: '総走行距離', st_total_time: '総走行時間', st_total_crash: '総衝突回数',
  st_more: 'さらに {n} 件表示', st_unknown_date: '日付不明',
})
