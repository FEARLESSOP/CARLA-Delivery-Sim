import { registerDict } from '@/lib/i18n'

// Strings shared by the shell and several views. / 共通の文言
registerDict({
  nav_sim: 'Simulation', nav_drive: 'Drive', nav_vehicle: 'Vehicle', nav_delivery: 'Delivery', nav_drones: 'Drones', nav_minipods: 'Mini Pods',
  nav_stats: 'Statistics', nav_training: 'AI training', nav_settings: 'Settings',
  st_offline: 'dashboard offline', st_idle: 'not running', st_starting: 'starting…', st_running: 'running',
  st_switching: 'switching map…', st_stopping: 'stopping…', st_hero_wait: 'waiting for your car',
  btn_stop: 'Stop', btn_stop_sure: 'Click again to stop', csv: 'CSV log', lang: 'Language',
  map: 'Map', fps: 'FPS', sim_time: 'Sim time',
}, {
  nav_sim: 'シミュレーション', nav_drive: '運転', nav_vehicle: '車両', nav_delivery: '配送', nav_drones: 'ドローン', nav_minipods: 'ミニポッド',
  nav_stats: '統計', nav_training: 'AI 学習', nav_settings: '設定',
  st_offline: 'ダッシュボード未接続', st_idle: '停止中', st_starting: '起動中…', st_running: '実行中',
  st_switching: 'マップ切替中…', st_stopping: '停止中…', st_hero_wait: '自車を待機中',
  btn_stop: '停止', btn_stop_sure: 'もう一度押すと停止', csv: 'CSV ログ', lang: '言語',
  map: 'マップ', fps: 'FPS', sim_time: 'シミュレーション時間',
})
