# Delivery KPIs: LastMile pod (LMP-4) and drones

Key performance indicators for the delivery simulation (`delivery_sim.py`,
`delivery/`), one set for the **pod** and one for the **drones**. Mini Pods
and LastMile Hubs are left out for now.

Each KPI lists:
- **Formula**: how it is calculated.
- **Source**: where the numbers come from. A field name means the value is
  already in `summary.json` / `state.json`. *New* means it must be logged first.
- **Better**: which direction counts as better (↑ higher, ↓ lower).

Targets are left blank on purpose. Set them after a few baseline runs on the
same map and seed.

Time is simulation time (`sim_seconds`). A run is one `delivery_sim` session,
and its results are in `summary.json`, with every event in `events.csv`.

---

## 1. Pod KPIs (LMP-4)

### 1.1 Delivery output

| KPI | Formula | Source | Better | Target |
|---|---|---|---|---|
| Pod deliveries | face-to-face orders delivered by the pod | `delivered_face_to_face`, `per_van[].delivered` | ↑ | |
| Pod throughput | pod deliveries ÷ sim hours | `delivered_face_to_face`, `sim_seconds` | ↑ | |
| Pod success rate | delivered ÷ (delivered + failed), face-to-face only | `per_van[].delivered`, `per_van[].failed` | ↑ | |
| Batches completed | batches finished by each pod | `per_van[].batches_completed` | ↑ | |
| Backlog at end | orders still queued when the run stops | `still_queued` | ↓ | |
| Orders cancelled (outside zones) | orders no pod zone covers | `cancelled_outside_zones` | ↓ | |

### 1.2 Time

| KPI | Formula | Source | Better | Target |
|---|---|---|---|---|
| Mean order lead time | mean(delivered_t − created_t), face-to-face orders | *New* (split `mean_wait_s` by order kind) | ↓ | |
| 95th-percentile lead time | p95 of the same | *New* | ↓ | |
| Mean time per stop | time from arrival to departure (dock, side port, locker) | *New* (stage timestamps in `events.csv`) | ↓ | |
| Pod utilisation | time not IDLE ÷ run time | *New* (time in each `state`) | ↑ (up to a healthy cap) | |
| Charging time share | time charging ÷ run time | *New* | ↓ | |
| Drone-wait time at meeting points | time drones hold until the pod arrives (seen up to ~195 s) | `events.csv` "drone wait=" | ↓ | |

### 1.3 Energy and distance

| KPI | Formula | Source | Better | Target |
|---|---|---|---|---|
| Distance driven | km per pod | `per_van[].pod_km` | — | |
| Distance per delivery | pod_km ÷ pod deliveries | `pod_km`, `delivered_face_to_face` | ↓ | |
| Energy per delivery | kWh used ÷ deliveries | *New* (sum the `PodEnergyModel` drain) | ↓ | |
| Charges per run | charging-station visits | `per_van[].ev_charges` | ↓ | |
| Battery at end | state of charge at the end of the run | `per_van[].pod_battery` | ↑ | |
| Stranded pods | pods out of battery away from a charger | `pods_stranded` | ↓ (target 0) | |

### 1.4 Driving quality and safety

| KPI | Formula | Source | Better | Target |
|---|---|---|---|---|
| Max lane offset | largest distance from the lane centre | `per_van[].max_lane_offset_m` | ↓ | |
| Reroutes | route replans by the off-road guard | `per_van[].reroutes` | ↓ | |
| Lane resets | times the pod was put back on the lane | `per_van[].lane_resets` | ↓ (target 0) | |
| Stuck recoveries | reverse-outs by `recovery.py` | *New* (count per pod) | ↓ | |
| Collisions | pod collisions with vehicles, walkers, props | *New* (collision sensor) | ↓ (target 0) | |
| Traffic-rule breaks | red lights, 止まれ, speeding | `traffic_rules` | ↓ (target 0) | |

---

## 2. Drone KPIs

### 2.1 Delivery output

| KPI | Formula | Source | Better | Target |
|---|---|---|---|---|
| Drone deliveries | drone orders delivered | `delivered_drone` | ↑ | |
| Drone throughput | drone deliveries ÷ sim hours | `delivered_drone`, `sim_seconds` | ↑ | |
| Drone success rate | delivered ÷ (delivered + failed), drone orders only | `delivered_drone`, `failed_by_reason` | ↑ | |
| Deliveries per drone | trips per unit | `per_drone[].trips` | — (check balance) | |
| Fleet availability | drones ready ÷ fleet size, averaged over the run | `status` counts (`drones.summary()`), sampled | ↑ | |

### 2.2 Time

| KPI | Formula | Source | Better | Target |
|---|---|---|---|---|
| Mean order lead time | mean(delivered_t − created_t), drone orders | *New* (split `mean_wait_s` by order kind) | ↓ | |
| Mean flight time per trip | take-off to landing | *New* (`Sortie` timestamps) | ↓ | |
| Hold time at meeting point | time hovering or parked until the pod arrives | `events.csv` "drone wait=" | ↓ | |
| Weather hold time | time grounded by wind or rain (precipitation ≥ 20) | *New* (station wind hold + `weather_cycle` rain) | ↓ | |
| Turnaround in pod | land → battery swap / reload → next take-off | *New* (crane stage timestamps) | ↓ | |

### 2.3 Energy and distance

| KPI | Formula | Source | Better | Target |
|---|---|---|---|---|
| Distance flown | km per fleet / per drone | `drone_km` | — | |
| Distance per delivery | drone_km ÷ drone deliveries | `drone_km`, `delivered_drone` | ↓ | |
| Battery swaps | swaps in the pod or at the station | `swaps` | ↓ | |
| Charge trips | flights to the charge station | `charge_trips` | ↓ | |
| Battery at landing (min) | lowest state of charge at any landing | *New* (log SoC on landing) | ↑ (safety margin) | |
| Mean flight altitude | mean cruise height (per-leg rule: 8 m over obstacles, min 15 m) | *New* | ↓ (within safety) | |

### 2.4 Safety and reliability

| KPI | Formula | Source | Better | Target |
|---|---|---|---|---|
| Crashes | crashed drones | `crashes` | ↓ (target 0) | |
| Crash rate | crashes ÷ 100 trips | `crashes`, `trips` | ↓ | |
| Failures by cause | collision, blown off course, battery died, motor failure, parcel torn off, no safe drop, no drone | `failed_by_reason` | ↓ | |
| Repairs | drones sent for repair | `repairs` | ↓ | |
| No-fly zone breaches | flight samples inside an active no-fly zone | *New* (check against `zones.ACTIVE`) | ↓ (target 0) | |
| Min obstacle clearance | smallest distance to an obstacle during a flight | *New* (`SkyCheck` / per-tick `cast_ray`) | ↑ | |
| Wind gusts met | gusts during flights | `gusts` | — (context) | |

---

## 3. How to compare runs

- Compare runs on the **same map, seed, fleet size and traffic**. The seed is
  printed at start; replay it with `--seed`.
- Report pod and drone KPIs **side by side but never added together**. Lead
  times only compare fairly within the same order kind.
- Rates (per hour, per 100 trips, per delivery) are fairer than raw counts
  when run lengths differ.
- The *New* KPIs need logging added to `delivery_sim.py` / `delivery/`
  (mostly timestamps per stage and per state) before they can be measured.
