# FTC 2025–2026 Season: DECODE™ Presented by RTX
> Competition Manual TU32 — Context for Claude Code

This file provides Claude Code with full context about the DECODE game so it can assist with robot code, autonomous routines, scoring logic, and strategy.

---

## Game Summary

Two ALLIANCES of 2 teams each compete on a 12×12 ft field. Each alliance scores colored **ARTIFACTS** (polypropylene balls) into their **GOAL**, builds a **PATTERN** on their **RAMP**, and returns to their **BASE** before time runs out. A randomly selected **MOTIF** (revealed by the **OBELISK**) defines the required PATTERN color sequence.

**Match duration:** 30-second AUTO + 8-second transition + 2-minute TELEOP = ~2:30 total

---

## Field Layout

- **Field size:** ~144 × 144 in. (12 ft × 12 ft), foam TILE surface
- **TILES:** 36 interlocking 24×24×0.59 in. soft foam tiles, arranged in a 6×6 grid (columns A–F, rows 1–6)
  - Columns A, B, C = Blue side; Columns D, E, F = Red side (relevant during AUTO)
- **One CLASSIFIER per alliance:** consists of SQUARE (top scoring gate), RAMP (holds up to 9 CLASSIFIED ARTIFACTS), and GATE (push-to-open, robot-activated)
- **One GOAL per alliance:** ~27×27×54 in. tall, open top, 38.75 in. lip height; robots LAUNCH ARTIFACTS through the open top
- **One OBELISK:** triangular prism placed outside the field on the GOAL side, shows one of 3 MOTIFs via AprilTag face

### Key Zones

| Zone | Description |
|------|-------------|
| **LAUNCH ZONE** | Triangular zones where robots must be to LAUNCH — audience-side (2×1 tiles) and GOAL-side (6×3 tiles) |
| **BASE ZONE** | 18×18 in. alliance-colored zone; robots return here at end of match |
| **LOADING ZONE** | ~23×23 in. zone near field perimeter; human players load ARTIFACTS here during TELEOP |
| **GATE ZONE** | 2.75×10 in. zone adjacent to GATE; protected from opponent contact |
| **SECRET TUNNEL ZONE** | ~46.5 in. long zone between opponent GATE and LOADING ZONE; overflow ARTIFACTS exit here |
| **DEPOT** | White tape at base of GOAL; ARTIFACTS resting here score DEPOT points |
| **ALLIANCE AREA** | 96×54 in. area outside field where drive team stands |
| **SPIKE MARKS** | 6 white tape marks (10 in. long) on field, marking pre-staged ARTIFACT positions |

---

## Scoring Elements (ARTIFACTS)

- **Purple (P):** 24 total per match
- **Green (G):** 12 total per match
- **Size:** ~5 in. (nominal 4.9 ± 0.25 in.) Gopher ResisDent™ polypropylene balls (am-3376a)
- **Not perfectly spherical** — design intake mechanisms to accommodate variation

### Pre-Match Staging

| Location | ARTIFACTS |
|----------|-----------|
| Near SPIKE MARK (audience side) | GPP |
| Middle SPIKE MARK | PGP |
| Far SPIKE MARK (GOAL side) | PPG |
| Each LOADING ZONE (×2) | PGP (3 total: 2P, 1G) |
| Each ALLIANCE AREA (×2) | 6 ARTIFACTS (4P, 2G), no set order |
| Each robot pre-load | Up to 3 ARTIFACTS from alliance area |

---

## MOTIF & PATTERN System

The **OBELISK** is randomized after drive teams set up. It shows one of 3 faces, each with an AprilTag:

| MOTIF | AprilTag ID | RAMP Pattern (indices 1→9, GATE→SQUARE) |
|-------|------------|------------------------------------------|
| GPP | 21 | G-P-P-G-P-P-G-P-P |
| PGP | 22 | P-G-P-P-G-P-P-G-P |
| PPG | 23 | P-P-G-P-P-G-P-P-G |

- RAMP holds up to **9 ARTIFACTS** (CLASSIFIED)
- ARTIFACTS enter GOAL through open top → exit archway → through SQUARE → onto RAMP
- CLASSIFIED = lands directly on RAMP; OVERFLOW = skips over the 9th slot or rolls over other ARTIFACTS
- PATTERN points are earned when an ARTIFACT's color at a given RAMP index matches the MOTIF color for that index

### AprilTag Reference

| Tag ID | Location | Use |
|--------|----------|-----|
| 20 | Blue GOAL (front face) | Navigation / targeting |
| 21 | OBELISK face 1 | MOTIF: GPP |
| 22 | OBELISK face 2 | MOTIF: PGP |
| 23 | OBELISK face 3 | MOTIF: PPG |
| 24 | Red GOAL (front face) | Navigation / targeting |

> Note: OBELISK placement is not deterministic relative to field coordinates — do not use for navigation.

---

## Match Periods

### AUTO (0:00–0:30)
- Robots operate fully autonomously — no driver input allowed
- Robots can read the OBELISK AprilTag to determine MOTIF
- ROBOTS start in STARTING CONFIGURATION within their ALLIANCE's half
- Key AUTO scoring: LEAVE, CLASSIFIED/OVERFLOW ARTIFACTS, AUTO PATTERN

### Transition (0:30–0:38)
- 8-second gap; no powered robot movement allowed
- Drivers may press INIT on Driver Station app; do NOT cause any actuator movement

### TELEOP (0:38–2:38)
- Drivers remotely operate robots
- Human players can load ARTIFACTS into robots via the LOADING ZONE
- Key TELEOP scoring: CLASSIFIED/OVERFLOW, TELEOP PATTERN, DEPOT, BASE return

---

## Scoring Point Values

### Match Points

| Achievement | AUTO | TELEOP |
|-------------|------|--------|
| LEAVE (robot no longer over LAUNCH LINE at end of AUTO) | 3 | — |
| CLASSIFIED ARTIFACT | 3 | 3 |
| OVERFLOW ARTIFACT | 1 | 1 |
| DEPOT (ARTIFACT over DEPOT at match end) | — | 1 |
| PATTERN (each ARTIFACT matching MOTIF color at correct index) | 2 | 2 |
| Partially returned to BASE | — | 5 |
| Fully returned to BASE | — | 10 |
| **Bonus: Both robots fully returned to BASE** | — | **+10** |

### Ranking Points (RP)

| RP | Description | Threshold (Most Events) |
|----|-------------|------------------------|
| MOVEMENT RP | Combined LEAVE + BASE points ≥ threshold | 16 |
| GOAL RP | ARTIFACTS scored through SQUARE ≥ threshold | 36 |
| PATTERN RP | PATTERN points ≥ threshold | 18 |
| WIN | More MATCH points than opponent | 3 |
| TIE | Same MATCH points as opponent | 1 |

> Thresholds are higher at Regional Championships (42/22/21) and FIRST Championship (67/22/21).

---

## ARTIFACT Scoring Flow

1. Robot is in a LAUNCH ZONE (or overlapping a LAUNCH LINE)
2. Robot LAUNCHES ARTIFACT through the **open top** of the GOAL
3. ARTIFACT exits the archway and passes through the **SQUARE** (diverter at top of RAMP)
4. If it lands directly onto RAMP → **CLASSIFIED** (3 pts AUTO, 3 pts TELEOP)
5. If it skips or passes over → **OVERFLOW** (1 pt)
6. CLASSIFIED ARTIFACTS are retained by the **GATE** until the robot opens it
7. To open GATE: robot pushes the GATE lever (approximately 2 in. horizontal displacement)
8. GATE height range: 3.75–5.5 in. closed; ~3 in. open above TILE surface
9. When GATE is opened, CLASSIFIED ARTIFACTS roll out into the opponent's SECRET TUNNEL ZONE
10. OVERFLOW ARTIFACTS pass over the GATE top when the RAMP is full

> Design tip: Use a large vertical surface at the front of the robot to ensure consistent GATE contact from 0–5.5 in. height.

---

## BASE Return (Endgame)

At the end of TELEOP:
- **Partially returned to BASE:** robot is partially supported by the TILE in BASE ZONE → **5 pts**
- **Fully returned to BASE:** robot only supported by TILE in BASE ZONE → **10 pts**
- **Bonus:** Both alliance robots fully returned to BASE → **additional 10 pts** (20 pts total for the pair)
- Vertical expansion up to **38 in.** is allowed in the **final 20 seconds** when NOT in a LAUNCH ZONE

---

## Robot Sizing Rules

### Starting Configuration
- Must fit within an **18 × 18 × 18 in. cube** (including electronics, battery, etc.)
- Pre-loaded ARTIFACTS may extend outside this cube
- Robot must be fully self-supported (no force on sizing tool walls or top)

### Expansion During Match
| Expansion | Limit | Notes |
|-----------|-------|-------|
| Horizontal | 18 × 18 in. maximum (fixed) | Must be physically constrained — software limits alone are NOT sufficient |
| Vertical (normal) | 18 in. max | Software or physical constraint OK |
| Vertical (final 20 sec, outside LAUNCH ZONES) | Up to 38 in. | Software or physical constraint OK |

---

## Key Game Rules Summary

### ROBOT Interaction Rules
- **G408:** Max **3 ARTIFACTS** controlled simultaneously (MINOR FOUL per extra)
- **G416:** Can only LAUNCH when inside a LAUNCH ZONE or overlapping a LAUNCH LINE
- **G417:** May NOT contact opponent's GATE; may NOT apply closing force to any GATE
- **G418:** May NOT contact ARTIFACTS on any RAMP (own or opponent); only remove own ARTIFACTS by operating own GATE
- **G419:** Must LAUNCH into own GOAL's open top — no direct placement onto RAMP or scoring into opponent GOAL
- **G414/G415:** Must comply with horizontal/vertical expansion limits during match

### AUTO-Specific Rules
- **G401:** No driver/human interaction with robot or OPERATOR CONSOLE from OBELISK randomization until end of AUTO (except pressing start/stop buttons)
- **G402:** During AUTO, robot may NOT contact an opponent robot fully in its own side, or disrupt pre-staged ARTIFACTS on the opponent's side
  - Columns A-B-C = Blue side; D-E-F = Red side

### TELEOP-Specific Rules
- **G403:** No powered movement during the 8-second AUTO→TELEOP transition
- **G404:** Robots must stop at end of TELEOP (MAJOR FOUL if robot contacts GATE or LAUNCHES ARTIFACT into GOAL after time)

### Human Player Rules
- **G432:** DRIVE TEAM members may only introduce/remove/move ARTIFACTS in the **LOADING ZONE**, only during TELEOP, only by hand (no tools), and only the ARTIFACT being CONTROLLED by a robot may exit the LOADING ZONE
- **G434:** Max **6 ARTIFACTS stored out-of-play** in ALLIANCE AREA during TELEOP (prevents field starvation)
- **G430:** DRIVE COACHES may NOT contact ARTIFACTS

### Contact/Defense Rules
- **G422:** No PINNING opponent robot for more than 3 seconds (MINOR FOUL + additional per 3 sec)
- **G424:** GATE ZONE is protected — no contact with opponent robot while either robot is in opponent's GATE ZONE
- **G425:** No contact with opponent robot while own robot is in opponent's SECRET TUNNEL ZONE
- **G426:** LOADING ZONE is protected — no contact with opponent while either robot is in opponent's LOADING ZONE
- **G427:** BASE ZONE protected during **last 20 seconds** of match (MAJOR FOUL + automatic BASE points awarded)
- **G420:** No deliberate functional impairment of opponent robot (MAJOR FOUL + YELLOW/RED CARD)
- **G421:** No deliberate tipping or entangling opponent robot

### Penalties
| Penalty | Points |
|---------|--------|
| MINOR FOUL | 5 pts to opponent |
| MAJOR FOUL | 15 pts to opponent |
| YELLOW CARD | Warning; 2nd YELLOW = RED CARD |
| RED CARD | DISQUALIFIED (0 MATCH pts + 0 RP) |

---

## Programming Notes

### MOTIF Detection (AUTO)
- Read AprilTag on the OBELISK face pointing toward the field
  - ID 21 → MOTIF = GPP
  - ID 22 → MOTIF = PGP
  - ID 23 → MOTIF = PPG
- OBELISK is outside the field and position varies — use only for MOTIF detection, not navigation
- GOAL AprilTags (ID 20 = blue, ID 24 = red) are reliable for targeting

### AUTO Strategy Priorities
1. Move off LAUNCH LINE → +3 pts LEAVE
2. Detect MOTIF via OBELISK AprilTag
3. Pre-loaded ARTIFACTS: up to 3 ARTIFACTs from alliance area
4. LAUNCH ARTIFACTS matching MOTIF colors into GOAL → CLASSIFIED = +3 pts each
5. Build correct PATTERN sequence on RAMP → +2 pts per matching index
6. Do NOT cross into opponent's columns (A-C for red, D-F for blue) or disrupt their pre-staged ARTIFACTS

### TELEOP Strategy Priorities
1. Collect ARTIFACTS from field (SPIKE MARKS, LOADING ZONE, OVERFLOW from own GATE)
2. LAUNCH from LAUNCH ZONE → CLASSIFY and build PATTERN
3. Coordinate with HUMAN PLAYER to receive ARTIFACTS via LOADING ZONE
4. Open own GATE when RAMP fills to receive more ARTIFACTS (and release previous batch)
5. In final 20 seconds: return to BASE ZONE; vertical expansion allowed up to 38 in.

### OpMode Requirements (per G305)
- Must select OpMode on DRIVER STATION app and press INIT before match
- If running AUTO: select AUTO OpMode with 30-second timer enabled
- Otherwise: select TELEOP OpMode
- Recommended: have AUTO OpMode automatically queue TELEOP OpMode after completing

### Expansion Safety (Code)
- Horizontal limits are **physical constraints only** (18×18 in.) — software limits alone are insufficient per R105.A
- Vertical limits (18 in. normal, 38 in. final 20 sec / not in LAUNCH ZONE) may be software or physical
- Track match timer to allow >18 in. vertical extension only in final 20 seconds and when outside LAUNCH ZONES

---

## Common Sensor Use Cases

| Task | Sensor Approach |
|------|----------------|
| Detect MOTIF | AprilTag camera (OBELISK: IDs 21, 22, 23) |
| Navigate to GOAL for launching | AprilTag camera (GOAL IDs 20/24) |
| Detect ARTIFACT color (P vs G) | Color sensor |
| LEAVE detection | IMU / encoder odometry (crossed LAUNCH LINE) |
| BASE return | Encoders / odometry |
| GATE operation | Touch sensor or limit switch on GATE arm |
| RAMP fill level | Not directly sensed — track count of CLASSIFIED ARTIFACTS launched |

---

## Field Element Dimensions (Quick Reference)

| Element | Key Dimension |
|---------|--------------|
| Field | 144 × 144 in. (~12 ft × 12 ft) |
| GOAL opening | ~26.5 in. wide × 18.3 in. deep |
| GOAL lip height | 38.75 in. from TILE surface |
| GOAL backboard height | 15 in. above GOAL opening top |
| RAMP capacity | 9 CLASSIFIED ARTIFACTS |
| GATE (closed) | 3.75–5.5 in. above TILE surface |
| GATE (open) | ~3 in. above TILE surface |
| GATE travel | ~2 in. horizontal displacement to open |
| BASE ZONE | 18 × 18 in. |
| LOADING ZONE | ~23 × 23 in. |
| OBELISK height | 23 in. tall, 11 in. wide faces |
| ARTIFACT diameter | ~5 in. (4.9 ± 0.25 in.) |
| TILE | 24 × 24 × 0.59 in. |

---

*Source: FTC 2025–2026 DECODE Competition Manual, Team Update 32 (April 16, 2026)*
