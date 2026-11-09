import { registerDict } from '@/lib/i18n'

// Timetabled buses card (Drive page, Environment). / 路線バスのカード
registerDict({
  bus_card: 'Timetabled buses',
  bus_off: 'No timetabled buses running (OitaStation only, new road logic).',
  bus_clock: 'Timetable clock',
  bus_on_map: 'Buses on the map',
  bus_done: 'Finished / removed',
  bus_next: 'Next departures, Oita Station',
  bus_none: 'none in the next 3 h',
  bus_platform: 'Platform {p}',
  bus_kousha: 'Alighting only',
  bus_ueno: 'Ueno-no-mori exit',
  bus_kaname: 'Kanamecho (highway)',
  bus_day_weekday: 'weekday',
  bus_day_saturday: 'Saturday',
  bus_day_sunday_holiday: 'Sunday / holiday',
}, {
  bus_card: '路線バス（時刻表）',
  bus_off: '時刻表バスは走っていません（大分駅マップ・新しい道路ロジックのみ）。',
  bus_clock: '時刻表の時刻',
  bus_on_map: 'マップ上のバス',
  bus_done: '運行終了 / 削除',
  bus_next: '大分駅の次の発車',
  bus_none: '3 時間以内に無し',
  bus_platform: '{p}番のりば',
  bus_kousha: '降車専用',
  bus_ueno: '上野の森口',
  bus_kaname: '要町（高速バス）',
  bus_day_weekday: '平日',
  bus_day_saturday: '土曜',
  bus_day_sunday_holiday: '日祝',
})
