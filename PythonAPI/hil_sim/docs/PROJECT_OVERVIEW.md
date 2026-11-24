# Pods, drones and what this project is for

A short, plain introduction to the delivery part of the CARLA simulator.
配送シミュレーションの簡単な紹介（日本語は各節の後）。

## The idea in one sentence

Parcels in a town are delivered by **small self-driving delivery vehicles (pods)** that
carry **drones**: the pod drives the parcels into the neighbourhood, and the drones fly
the last few hundred metres to the doors.

町の荷物を、ドローンを積んだ小型の自動運転配送車（ポッド）で届ける。ポッドが近くまで運び、
最後の数百メートルはドローンが飛んで届ける。

## The pod

- A small **electric delivery vehicle** that drives itself on the roads, obeying the
  traffic rules (signals, stop signs, speed limits, keep left in Japan).
- It carries **all the parcels for its zone** of the town (the map is split into 3 zones,
  one pod each) and **4 drones** on its roof.
- It delivers some parcels **itself**, by dropping them in **parcel lockers** on the way.
- It **launches** drones for the other parcels from a stop with open sky, then drives on
  and **picks the drones up again** at a meeting point, where they land on its roof.
- Its battery (10 kWh) powers the driving and **recharges the drones** while they ride on
  it, so the pod must also visit **charging stations** in time.

ポッド：交通ルールを守って自動で走る小型の電気配送車。担当ゾーン（町を 3 つに分け、各 1 台）の
荷物すべてと屋根のドローン 4 機を運ぶ。一部の荷物は自分で宅配ロッカーに入れ、残りはドローンを
発進させ、合流地点で屋根に着陸させて回収する。電池でドローンも充電するため、充電ステーション
にも寄る必要がある。

## The drones

- Small **delivery quadcopters**, one parcel each.
- A drone takes off from the pod, **flies to the customer's address**, lowers the parcel,
  then flies to the **meeting point** and lands back on the pod.
- On the way it must avoid buildings and trees, stay out of **no-fly zones**, cope with
  **wind**, and come back before its battery runs out (about 25 minutes of flight).
- If a drone is low on battery it can fly to a **charging station** instead, where spare
  drones also wait; a crashed drone is replaced by a spare.

ドローン：荷物 1 個を運ぶ小型の配送用クアッドコプター。ポッドから離陸し、お客様の住所へ飛んで
荷物を降ろし、合流地点でポッドに着陸する。建物や木を避け、飛行禁止区域に入らず、風に対応し、
電池（約 25 分）が切れる前に戻る。電池が少なければ充電ステーションへ。墜落したら予備機と交代。

## Where it runs

Everything runs in **CARLA**, a realistic 3-D driving simulator, on a model of the area
around Oita Kosen in Japan, with traffic, pedestrians and weather. A web dashboard shows
the fleet live, and camera windows show what each drone sees.

すべて実写的な 3D 運転シミュレーター CARLA 上で、大分高専周辺の町のモデル（交通・歩行者・天気あり）
で動く。ダッシュボードで艦隊の様子を、カメラ画面で各ドローンの視点を見られる。

## What AI (reinforcement learning) does here

Instead of writing rules by hand, some decisions are **learned by trial and error** in a
fast practice world, then tested in CARLA:
- **Drone flying**: the drone learns to fly to the address, avoid obstacles and land on the
  moving pod's roof by itself.
- **Fleet decisions**: where the pod stops, how many drones to launch, where to meet,
  when to charge.
- **After the drop** (the research study "Paper 1"): should the drone go back to the pod,
  meet it at its next stop, or recharge at a station?

The pod's driving itself is ordinary code (CARLA's driving agent with the Japanese rules).

手で規則を書く代わりに、一部の判断を高速な練習世界で試行錯誤により学習させ、CARLA で試す：
ドローンの飛行、艦隊の判断（停車場所・発進数・合流地点・充電）、荷物を降ろした後の行き先
（研究「Paper 1」）。ポッドの運転そのものは通常のプログラム。

## Expected outcome

1. **A working simulator** of drone-assisted last-mile delivery in a real town, that people
   can watch, drive in (with a steering wheel) and use for experiments.
2. **Faster, more reliable deliveries** from the learned decisions than from simple
   fixed rules: more parcels delivered on time, drones ready again sooner, fewer failed
   deliveries and flat batteries.
   - First result (Paper 1, practice world): the proposed method delivers about 0.6 more
     parcels per hour within 30 minutes than the best other method, and drones are ready
     about 22 s sooner. In four CARLA test days customers waited less with it every day (about 1¾ min), while the number of deliveries was mixed.
3. **Drones that fly safely by themselves**: reach the address, avoid obstacles and no-fly
   zones, and land on the pod. (Avoiding no-fly zones is not solved yet.)
4. **Research results** (papers) on how learning plus planning ahead can manage a mixed
   fleet of vehicles and drones, and lessons on training such systems.

Long term, the same approach could help plan real delivery services with pods and drones
before they are built.

期待される成果：(1) 実在の町でのドローン併用ラストマイル配送のシミュレーター（観察・ハンドルで運転・
実験に使える）。(2) 学習した判断により、単純な固定ルールより速く確実な配送（時間内の配達が増え、
ドローンの準備が早く、失敗や電池切れが減る）。最初の結果では提案手法が最良の他手法より 30 分以内の
配達が毎時約 0.6 件多く、ドローンの準備が約 22 秒早い。CARLA の 4 日間の試験では毎日注文の待ち時間が短く（平均約 1 分 45 秒）、配達数は日により勝ち負けがあった。
(3) 自分で安全に飛ぶドローン（飛行禁止区域の回避は未解決）。(4) 車両とドローンの混成艦隊を学習と
先読みで運用する研究成果（論文）。将来は、実際のポッド・ドローン配送サービスを作る前の計画に役立つ。
