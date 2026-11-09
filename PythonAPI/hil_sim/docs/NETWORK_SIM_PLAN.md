# Wireless network co-simulation plan (ns-3 + CARLA)

Plan for adding VANET, 5G (with network slicing) and 6G research links to the
CARLA HIL rig. Status: **plan only, nothing built yet** (2026-11-08).

## 1. Goal

Simulate the radio network the LastMile fleet would really use, inside the
running simulation:

- vehicles, LMP-4 pods, LastMile Hubs, Mini Pods and drones talk to each
  other (V2V), to roadside units (V2I) and to 5G/6G base stations (gNBs);
- every message sees realistic latency, loss and throughput, blocked by the
  map's real buildings;
- the network results feed back into the sim (late control commands, lost
  camera frames, a drone that loses its link) and appear on the dashboard;
- network slices give each traffic type its own guaranteed share.

## 2. Architecture: co-simulation

CARLA stays the physical world; ns-3 simulates the radio and the network.
They exchange data every tick through a bridge.

```
CARLA 0.9.16 (tick master, 30 Hz)              ns-3 (C++, 5G-LENA, WAVE, ...)
  vehicles, pods, hubs, mini pods,   ──pos──►   nodes moved each tick
  drones, RSUs, gNBs                            PHY/MAC/RLC/PDCP, gNBs, core, slices
  buildings (buildings.csv, meshes)  ──once─►   Buildings module: LOS / NLOS
  apps (telemetry, video, teleop)    ◄─pkts──   delivered packets: latency, loss, rate
        │                                        per node, per slice, per link
  dashboard "Network" page: gNB map, coverage heatmap, link lines, slice KPIs
```

- **Bridge** (`hil_sim/netsim/bridge.py` <-> an ns-3 program `carla-bridge`):
  ZeroMQ (or gRPC) messages, one step per CARLA tick: positions/velocities in,
  delivered packets and link metrics out. ns-3 runs to the same sim time
  (lockstep) or a little behind (loose coupling, no wait on CARLA).
- **Where it runs**: ns-3 on the second PC (offloads this one, which already
  runs CARLA at ~8 FPS), or on this PC (20 cores, 62 GB) for small scenes.
- **Clock**: CARLA sim time is the master; ns-3 never runs ahead of it.

## 3. Phases

| Phase | What | Result | Effort |
|---|---|---|---|
| 1 | CARLA's own V2X sensors (`sensor.other.v2x` CAM, `v2x_custom`) on the fleet; charging stations, box lockers and signals as roadside units | CAM messages and loss on the dashboard; no ns-3 yet | days |
| 2 | ns-3 bridge + VANET: 802.11p (WAVE) and/or C-V2X / NR-V2X sidelink (5G-LENA Mode 2); ETSI CAM/DENM via ms-van3t | real V2V/V2I network, building blockage | 1-2 weeks |
| 3 | 5G: gNBs on the map (section 4), UEs = every LastMile node, 5G core; network slicing (section 5) | per-slice latency/throughput, handovers, KPIs on the dashboard | 2-4 weeks |
| 4 | Feedback into the HIL rig: URLLC latency applied to remote driving / pod control, eMBB loss to drone camera windows, link loss grounds a drone | study how the network affects driving and delivery | 1-2 weeks |
| 5 | 6G research: sub-THz/THz links, RIS (reconfigurable surfaces), non-terrestrial (satellite/HAPS) links for drones | experimental (no standard 6G stack exists yet) | open |

## 4. gNB (base station) positions on the map

Where the gNBs stand decides coverage, handovers and every result, so their
placement is a first-class setting, saved per map, chosen on the dashboard
before a launch, like the charging stations.

### 4.1 Placement modes

| Mode | Who places them | Use |
|---|---|---|
| **Manual** (fixed) | you, on the Delivery/Network map: click to add, drag, set height, azimuths, band | repeatable experiments; a real operator's site list |
| **Random** | the sim, `N` gNBs, seeded (same seed = same layout) | variety, Monte-Carlo runs |
| **Grid / hexagonal** | the sim, a 3GPP-style layout with a set inter-site distance | textbook comparisons (TR 38.901 scenarios) |
| **Auto-coverage** | the sim picks sites to cover the roads and delivery areas with the fewest gNBs | "how many do I need?" |
| **Imported** | a CSV of real sites (lat/lon -> map frame, `maps/osm_city/geo.py`) | real-world operator data (where available) |

Saved like the charging stations: `data/network_sites.json`, per map:

```json
{"maps": {"OitaKosen": {
   "gnb": [{"id": "g1", "x": 106.8, "y": 55.9, "z_m": 25.0, "mount": "rooftop",
            "band": "n78", "freq_ghz": 3.5, "bw_mhz": 100, "tx_dbm": 46,
            "sectors": [0, 120, 240], "tilt_deg": 6, "source": "manual"}],
   "rsu": [{"id": "r1", "x": ..., "y": ..., "z_m": 6.0, "tech": "802.11p"}],
   "mode": "manual", "random": {"n": 4, "seed": 7, "isd_m": 200}}}}
```

### 4.2 Rules every gNB position must pass (manual is checked, random/auto obey)

- **Not on a road or pavement**, not inside a building's footprint at ground
  level: use the map's surface grid (`maps/build/<Map>/surface_grid.npz`,
  made by `osm_city/surface_grid.py`) and `buildings.csv`.
- **Mount type and height** (3GPP TR 38.901):
  - *rooftop macro* (UMa): on a building roof, antenna about 25 m above
    ground, or roof height + 3 m mast; the building must exist in
    `buildings.csv` at that spot;
  - *street small cell* (UMi street canyon): on a lamp/utility pole beside
    a road, about 10 m; snap to an existing pole (environment objects) or a
    kerb point with free pavement;
  - *standalone tower* on free lawn (like the charging stations' free-ground
    check), 25-35 m.
- **Spacing**: at least the scenario's minimum inter-site distance
  (UMi ~200 m, UMa ~500 m; configurable); random placement uses Poisson-disc
  sampling so sites never bunch.
- **Terrain** (hilly maps like OitaKosen): the antenna height is above the
  local ground (`cast_ray`), and hilltops are preferred for macro sites.
- **Line of sight**: each candidate is scored by how many road / delivery-zone
  sample points it sees (LOS through the building blockage model); the
  dashboard shows a coverage heatmap before the launch.
- **Sector azimuths**: 3 x 120 deg by default; manual can rotate them; auto
  mode points the main lobe at the most road length.
- **Keep-out**: not inside a charging station, box locker, landmark site or
  no-fly zone footprint (drones fly there; a tower would be an obstacle:
  register it in the drone obstacle list).

### 4.3 Random mode

1. Candidate sites: roofs (from `buildings.csv`), poles (environment
   objects), free lawn (surface grid), filtered by the rules above.
2. Poisson-disc sample `N` of them with the minimum inter-site distance,
   seeded.
3. Reject the layout if any road segment or delivery zone has no coverage
   (SINR below a threshold in the quick LOS check); retry with the next seed
   offset (report the coverage %).

### 4.4 Auto-coverage mode

Greedy set cover over road / delivery-zone sample points (every 10 m):
repeatedly add the candidate that covers the most still-uncovered points
(weighted: delivery zones and the ring road first) until coverage >= the
target (e.g. 95 %) or `N` is reached. Output: the sites, the coverage %, the
heatmap.

### 4.5 Dashboard

- **Launch page, "Network" card**: technology (V2X only / 5G / 5G + V2X),
  gNB mode (Manual / Random / Grid / Auto / Imported), count, seed, minimum
  inter-site distance, band (n77/n78/n79 FR1, n257 28 GHz FR2).
- **Map editor mode "Base stations"**: add / drag / delete gNBs and RSUs; set
  mount, height, azimuths, tilt, band; live rule check (red marker + reason);
  coverage heatmap preview (quick LOS model) before the launch.
- **Live "Network" page**: gNBs with sector wedges; each UE's serving cell
  (coloured line), handovers, SINR / RSRP, per-slice latency and throughput
  charts, packet loss per app; click a vehicle to see its link history.

### 4.6 In CARLA

gNB models as props (a rooftop antenna array, a street small cell on a pole, a
lattice tower), spawned at their sites like the charging stations; optional
debug sector wedges (black lines only: other colours bloom).

## 5. Network slicing

| Slice | Type | Traffic in the rig | Target |
|---|---|---|---|
| S1 | URLLC | remote driving / pod teleoperation, emergency stop, drone control | < 10 ms, 99.999 % |
| S2 | eMBB | drone and pod camera streams, map updates | 20-100 Mbit/s per stream |
| S3 | mMTC | box lockers, Mini Pod telemetry, charging-station status | many small packets |
| S4 | V2X (sidelink) | CAM/DENM between vehicles, cooperative perception | 100 ms, 10 Hz |

5G-LENA: one bandwidth part (BWP) per slice or QoS flows (5QI) on a shared
BWP; schedulers per slice; slice shares set on the dashboard (e.g. 20 / 60 / 10
/ 10 %), changeable live to show the effect on each app.

## 6. Traffic sources (who sends what)

| Node | Uplink | Downlink |
|---|---|---|
| LMP-4 pod | telemetry 10 Hz, 4 drone links, 1 camera | route, orders, teleop |
| LastMile Hub | telemetry, 3 Mini Pod relays | orders, teleop |
| Mini Pod | telemetry 10 Hz, lidar summary, camera | route, locker codes |
| Drone | telemetry 20 Hz, FPV video | flight commands |
| Hero car (G923) | CAM, sensor sharing | warnings (DENM) |
| Charging station / locker | status (mMTC) | commands |

## 7. Tools (to verify at install time)

- ns-3 (current release) with **5G-LENA** (NR, NR-V2X sidelink, BWPs),
  **WAVE** (802.11p), **Buildings**, **3GPP channel** (TR 38.901) modules;
  **ms-van3t** (ETSI C-ITS on ns-3, has CARLA coupling) as a reference.
- Bridge: ZeroMQ (`pyzmq` / `cppzmq`) or ns3-ai.
- Alternative stack: OMNeT++ + Simu5G / Veins / Artery (e.g. CARLANeT).
- 6G: sub-THz / THz and RIS research modules, NTN channel models (check
  versions; experimental).

## 8. Performance and accuracy

- Lockstep makes CARLA wait for ns-3: keep the node count to tens (the fleet
  + RSUs), or run loosely coupled; run ns-3 on the second PC.
- Statistical 3GPP channels + building blockage are the default; ray tracing
  (e.g. Sionna RT on the map meshes) is a later, heavier option.
- Reproducibility: seeds for the placement, ns-3 and the delivery sim.

## 9. Files (planned)

| File | What |
|---|---|
| `netsim/bridge.py` | CARLA side: node registry, positions out, packets in |
| `netsim/sites.py` | gNB / RSU placement: rules, random, grid, auto, import (pure, tested) |
| `netsim/ns3/carla-bridge.cc` | ns-3 program: nodes, stacks, slices, ZeroMQ |
| `netsim/props/` | gNB / small-cell / tower models (asset build like the LMH props) |
| `data/network_sites.json` | sites per map |
| `web/src/views/network/` | Network page + map editor mode |
| `tests/test_net_sites.py`, `tests/test_bridge.py` | placement rules, bridge protocol |

## 10. Open questions

- One technology at a time or VANET + 5G together?
- Which band(s): FR1 (3.5 GHz) only, or FR2 (28 GHz) small cells too?
- Real operator site data for Oita, or synthetic layouts only?
- Lockstep (exact, slower) or loose coupling (faster, approximate)?
- Which experiments first: teleoperation latency, drone video, platooning?
