---
name: carla-car-importer
description: "Turn a downloaded car or bus model (glb/FBX/OBJ) into a CARLA 0.9.16 vehicle for the user's HIL rig, or fix an imported one (white body, opaque glass, wheel not turning, wrong seat, blinkers, LHD->RHD, heavy NPC vehicles): survey, cars.json entry, cars/build_cars.sh, live verification. Use for any work under hil_sim/cars."
---

# CARLA Car Importer (Antigravity entry point)

This is a pointer. The real skill is Claude Code's, kept in one place so both agents share it:

1. Read `/home/inouelab/.claude/skills/carla-car-importer/SKILL.md` completely and follow it
   (the user's rules for every car, files, workflows, "Common failures" table).
2. Read `/home/inouelab/.claude/skills/carla-car-importer/car_guide/README.md` and the chapter
   for your task first; record new lessons there (change log ch. 11).
3. Roles: `agents/car-surveyor.md` -> `agents/car-checker.md` (gate) -> `agents/car-builder.md`
   -> `agents/car-checker.md` (gate), all under
   `/home/inouelab/.claude/skills/carla-car-importer/`. Play each role yourself or as a
   sub-agent with the role file as its prompt; the gate checks files and images, not summaries.
4. Build with `bash cars/build_cars.sh` and NO keys (the install replaces the whole UserCars
   package); it needs CARLA stopped: ask the user first.
5. Method, checks and traps: `/home/inouelab/Downloads/CARLA_0.9.16/AGENTS.md`.
