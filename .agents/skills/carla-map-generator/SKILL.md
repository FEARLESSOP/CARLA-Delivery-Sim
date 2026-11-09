---
name: carla-map-generator
description: "Build, import, fix or verify a CARLA 0.9.16 city map for the user's HIL rig (OpenStreetMap -> road logic -> generated 3-D model -> import), e.g. OitaStation, OitaKosen, PasirRis: roads, markings, buildings, landmarks, signs and signals (JARTIC), trees, embedded models, buses, live checks. Use for any work under hil_sim/maps."
---

# CARLA Map Generator (Antigravity entry point)

This is a pointer. The real skill is Claude Code's, kept in one place so both agents share it:

1. Read `/home/inouelab/.claude/skills/carla-map-generator/SKILL.md` completely and follow it
   (the user's rules, files, the new-map and edit workflows, the stage/gate order, the
   "Common failures" table).
2. Read the guide index `/home/inouelab/.claude/skills/carla-map-generator/map_guide/README.md`
   and the chapter for your task before acting (chapters stay under 100 lines; add every new
   lesson to the right chapter and a line to the change log, ch. 38).
3. Where the skill says "agent" or "spawn <name>", read the role file
   `/home/inouelab/.claude/skills/carla-map-generator/agents/<name>.md` and play that role
   yourself, or give its text as the prompt of an Antigravity sub-agent. After every stage run
   the agent-checker gate (`agents/agent-checker.md`) against the files themselves:
   APPROVED or REJECTED with reasons; nothing moves on without APPROVED.
4. "Claude in Chrome" in the skill = any browser tool you have (photo-getter's imagery).
5. The method, the checks with their pass criteria and the expensive traps are in
   `/home/inouelab/Downloads/CARLA_0.9.16/AGENTS.md`.
