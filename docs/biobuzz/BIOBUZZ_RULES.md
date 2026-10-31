# BIOBUZZ (FTC 2026-27): detailed rules summary

Summarized in our own words from the **BIOBUZZ Competition Manual, version TU07
(updated Oct 29, 2026)**:
https://ftc-resources.firstinspires.org/ftc/game/manual

This is a study aid, not the official text. Rules change through Team Updates, and
the official Q&A board answers rule questions. Check the current manual before
relying on any exact number. Rule numbers (G4xx, R1xx...) match the manual so you can
look them up.

---

## 1. The game in one paragraph

Two alliances of two robots each. Robots collect yellow **POLLEN** and alliance-colored
**NECTAR** balls and **LAUNCH** them into their alliance's **HIVE**. Enough balls in
the upward-facing CELL makes the HIVE **TIP** (20 points each time), which also lets
the human player bring one more NECTAR onto the field. In the last minute, alliances
race to place balls into the 4 **FLOWERS** on the walls. The alliance whose NECTAR is
**on top** of a FLOWER **owns** it and scores every ball inside. Robots end in their
**LOADING ZONE**.

## 2. Match timing

| Period | Length | Field timer | Notes |
|---|---|---|---|
| AUTO | 30 s | 2:30 → 2:00 | Robots run pre-programmed code only. |
| Transition | 8 s | — | No powered robot movement (G403). Drivers pick up controllers. |
| TELEOP | 2:00 | 2:00 → 0:00 | Drivers control robots. |
| FLOWER / NECTAR unlock | — | **1:00 left** | FLOWER scoring allowed and all remaining NECTAR may enter (chime). |
| Endgame cue | — | 0:20 left | Train whistle. |

Audio cues are a courtesy; the **visual field timer is the authority**.

## 3. The field (ARENA)

- **Field:** ~144 × 144 in, 36 foam tiles (24 × 24 in). Red alliance area is on the
  **left** as seen from the audience. Red side = tile columns **A, B, C**; blue side =
  **D, E, F**.
- **Official CAD** is the reference for dimensions. General tolerance is **±1 in**,
  and fields vary by venue, so design for it.
- **Alliance area:** ~97 in wide × 54 in deep, outside the field, where the drive team
  stands.
- **LOADING ZONE:** ~23 × 11 in taped box against the wall, one per alliance next to
  its alliance area. The human player enters NECTAR here, and robots PARK here.
- **GARDEN:** ~23 × 2 in taped strip in opposite field corners, one red and one blue.
  It scores 1 point per ball.

### HIVE Structure (center of field)

- One frame with **2 HIVES**, red and blue, **25.5 in apart** center to center.
- The frame is ~49.5 in wide × 39 in deep at the base. Each HIVE pivots on an axis
  **43.95 in** above the tiles.
- **Each HIVE has 2 CELLS**, one at each end of a rocker, ~18.8 in apart. It is
  **bi-stable**: one CELL faces up and the other down, tilted 30°.
- **CELL opening:** ~20 in wide × 14 in tall × 12 in deep. The up-facing opening spans
  ~**53.5 in to 65.6 in** above the tiles. The lowest point of the HIVE is ~30.6 in
  above the tiles.
- A HIVE stays put until enough POLLEN/NECTAR is launched into the **upward** CELL,
  then it flips to the other state (a **TIP**).
- **Setup:** the CELL that "points at" a FLOWER starts tilted down.

### FLOWERS (4, mounted on the perimeter walls)

- Top opening **~4 in diameter**, **~21.5 in** above the tiles, with a 1.25 in backstop
  on top to guide balls in.
- **Retrieval opening** at the bottom (~3.55 in tall), where robots may pull
  **POLLEN** (not NECTAR) out.
- A lower ring on the floor holds a POLLEN.
- **Scoring volume:** between the top ring and the middle ring.

### Scoring elements

| Element | Size | Count per match |
|---|---|---|
| POLLEN (yellow) | ~2.8 in (7.1 cm) | 40 |
| NECTAR (red / blue) | ~3.6 in (9.1 cm) | 8 red + 8 blue |

Balls are not perfectly round and vary in size, so design for it.

### AprilTags

- 36h11 family, 3.25 in squares, in **clusters of 4** on one sticker. Each tag has a
  printed ID label.
- One cluster is on the **bottom face of each CELL, facing down at the tiles**, with
  the bottom edge toward field center. **They move when the HIVE tips.**
- **IDs:** red far-side CELL **30–33**, red audience-side CELL **34–37**, blue
  audience-side CELL **38–41**, blue far-side CELL **42–45**.
- Reference holes on the sticker locate the cluster against the field CAD.

## 4. Pre-match setup

**Scoring elements are staged by field staff:**
- POLLEN:
  - 4 in each of the 4 FLOWERS (16).
  - 4 in a line in each GARDEN (8), starting from the corner nearest that
    alliance's area.
  - **4 pre-loaded per robot** (16).
- NECTAR:
  - **3 in each alliance's upward-facing CELL** (6).
  - **5 held in each alliance area** (10) for the human player.
- A no-show robot's 4 POLLEN go in the middle of its LOADING ZONE against the wall.

**Robot placement (G304), all required:**
- Fully on its own side (red A–C, blue D–F) and not overhanging the wall.
- **Touching the perimeter wall.**
- Not in the LOADING ZONE, and not touching or inside a FLOWER's scoring volume.
- Inside its **18 in cube** starting configuration.
- Touching **exactly 4 pre-load POLLEN**, held in or on the robot or on the tiles
  touching it, on its own side.
- **Fully motionless after OpMode INIT.**
- Not attached to or entangled with any field element.

If placement order matters, the referee alternates: red 1st, blue 1st, red 2nd,
blue 2nd.

**Driver Station (G305):** every team must select and INIT an OpMode. An AUTO OpMode
must have the 30 s timer enabled. Teams with no auto should still run a basic AUTO that
auto-loads their TELEOP.

**Drive team:**
- Up to 4 people: 1 DRIVE COACH (may be an adult), up to 3 DRIVERS (students), and a
  HUMAN PLAYER who handles NECTAR.
- Only 1 non-student is allowed, and badges must be visible.

## 5. Scoring

### Point values (Table 10-2)

| Action | AUTO | TELEOP | When it's judged |
|---|---|---|---|
| **LEAVE**: robot no longer touching the perimeter wall | 3 | — | End of AUTO |
| **PARK**: at least partly in your LOADING ZONE | 5 | 5 | End of AUTO / end of match |
| **HIVE TIP** | **20** | **20** | Live, all match. Tips finished before TELEOP starts count as AUTO. |
| Ball left in an **upward-facing CELL** | — | 2 each | After everything stops at match end |
| **Bottom NECTAR Bonus** (per FLOWER) | — | 5 | Live, final check at end |
| Ball in a FLOWER **you own** | — | 2 each | Live, final check at end |
| Ball in your **GARDEN** | — | 1 each | End of match |

### How each scoring type works

- **HIVE TIP:** the HIVE moves from one stable state to the other, so the old down CELL
  is now up, and the damper starts touching the frame. **Launching into the up-facing
  CELL is the only legal way to tip.** Shots aimed at the down-facing CELL while it is
  tipping can stop the tip, so pause and make it obvious.
- **Each TIP also unlocks NECTAR:** the human player may bring in **1 more NECTAR per
  tip** of their own HIVE, or all remaining NECTAR once 60 s are left, whichever comes
  first.
- **FLOWER scoring:**
  - A ball counts if it is at least partly in the volume between the top and middle
    rings.
  - **Only allowed in the last 60 s.** Earlier placements still count but draw
    penalties (G410: MAJOR FOUL per NECTAR).
  - Balls must enter **through the top**.
  - **Bottom NECTAR Bonus:** the alliance with the **lowest** NECTAR of its color in a
    FLOWER gets 5.
  - **FLOWER Owner:** the alliance with the **top-most** NECTAR of its color owns the
    FLOWER and scores **2 per ball** in it, including balls the other alliance put
    there.
- **GARDEN:** any POLLEN, or NECTAR of either color, at least partly in a GARDEN scores
  1 for the **GARDEN's** color, no matter who put it there. GARDENS are not protected:
  either alliance may take balls out.

### Ranking points (RP)

| RP | Requirement (regular events) |
|---|---|
| **WIN** | 3 RP |
| **TIE** | 1 RP |
| **SWARM** | 1 RP for LEAVE + PARK points ≥ **16** |
| **POLLINATOR 1** | 1 RP for ≥ **4 TIPS** |
| **POLLINATOR 2** | 1 RP for ≥ **7 TIPS** |

Regional and World Championship thresholds will come in Team Updates. Premier events
set their own.

Max SWARM points is 2 robots × (3 LEAVE + 5 AUTO PARK + 5 TELEOP PARK) = 26. Both
robots leaving (6) plus both parking at the end (10) hits exactly 16.

**Qualification ranking order:**
1. Ranking Score (average RP).
2. Average match points without fouls.
3. **Average number of TIPS.**
4. Average AUTO points.
5. Random.

## 6. Penalties

| Penalty | Effect |
|---|---|
| VERBAL WARNING | Warning only |
| **MINOR FOUL** | **+5** points to the opponent |
| **MAJOR FOUL** | **+20** points to the opponent |
| YELLOW CARD | Warning. A second yellow in the same tournament phase becomes a RED. |
| RED CARD | Disqualified for that match |
| DISABLED | Robot must stop for the rest of the match |
| DISQUALIFIED | 0 match points and 0 RP (Quals), or 0 points for the alliance (Playoffs) |

**Terms:**
- **MOMENTARY** ≈ under 3 s.
- **CONTINUOUS** ≈ over 10 s.
- **REPEATED** = more than once in a match.
- **STRATEGIC** = done to gain an advantage. Deliberate or reckless acts count;
  accidents don't, but accidents you then exploit do.

**Cards:**
- Cards carry over into later matches as a yellow card.
- Yellows and G301 warnings clear after Practice, after Qualifications, and after
  division Playoffs.
- In Playoffs, cards apply to the **whole alliance**.

## 7. Game rules (G)

### Safety and conduct
- **G101:** Humans stay off the field during a match. Leaning or pointing is fine.
  Warning.
- **G102:** Don't climb on, hang from, bend or damage field elements. You may brace the
  wall but not move it. Warning.
- **G201:** Be civil and respectful. Warning, then yellow if it happens again.
- **G202:** Follow the Competition Integrity Contract:
  - No throwing matches or deliberately missing RPs to manipulate rankings.
  - No asking teams to no-show, and no taunting or disrupting.
  - Warning, then yellow or red.
- **G203:** An inspected team must send at least 1 drive team member to every Qual
  match. Otherwise: disqualified for that match.
- **G204:** Don't force opponents into penalties. MAJOR FOUL per instance, plus a
  yellow if REPEATED. The victim isn't penalized.
  Example: shoving an opponent so it drops NECTAR into a FLOWER early.
- **G205:** The head referee can give yellow or red cards for egregious behavior.
  Taking a penalty on purpose because it's "worth it" counts as egregious.

### Pre-match
- **G301:** Don't delay matches.
  - First, a warning. Then a MAJOR FOUL for a repeat in the same phase.
  - If you're still not ready 2 minutes later with no good-faith effort, you're
    DISABLED.
  - A quick "wiggle test" is OK.
- **G302:** Only bring what fits in your alliance area, nothing over 6 ft 6 in tall,
  and nothing that communicates outside the arena.
  - **No mimicking field AprilTags** and no shining lights or lasers onto the field.
  - Alignment jigs are fine.
- **G303:** Robot must be safe and inspected, the only team item on the field, and show
  correct-color robot signs. A non-inspected robot playing means a RED CARD.
- **G304:** Starting position rules (see §4).
- **G305:** An OpMode must be selected and INIT pressed.

### AUTO
- **G401:** No touching the robot or driver station during AUTO, except to press start
  (within a moment of match start), press stop, or for safety. Warning; MAJOR FOUL plus
  yellow if strategic.
- **G402:** **No interfering with the other alliance's AUTO.** Each alliance has
  priority on its own half (red A–C, blue D–F).
  - Don't control their balls on their side, and don't hit their robots on their side.
  - **MAJOR FOUL per match**, plus a yellow if strategic. Crossing over is risky.

### TELEOP / timing
- **G403:** **No powered movement during the 8 s AUTO→TELEOP transition.** Coasting or
  gravity is fine. If your TELEOP INIT moves anything, wait until TELEOP starts to
  press INIT. Warning; MAJOR FOUL plus yellow if strategic.
- **G404:** No powered movement after the match ends. Press stop and put controllers
  down. Warning; MAJOR FOUL plus yellow if strategic.

### Scoring elements
- **G405:** Don't deliberately launch balls out of the field.
  **MAJOR FOUL per ball.** Missed shots and robot collisions are not deliberate.
- **G406:** Don't damage or mess up the arena, including gouging or tearing balls.
  Warning; MAJOR FOUL plus yellow if strategic; DISABLED if more damage is likely.
- **G407:** **Max 4 scoring elements controlled at once.** Warning; MAJOR FOUL plus
  yellow if strategic. Controlling 6+, or 5+ repeatedly, looks strategic, even with a
  "keep 4, dump the rest" sorter.
  - Not control: bulldozing, deflecting, and balls already launched.
  - Design guards so you can't accidentally hold a 5th.
- **G408:** **Don't control the opponent's NECTAR.** Warning; yellow if strategic.
  Repeated pickups count even if your sorter spits them back out.
- **G409:** **Don't catch balls falling from a tipped HIVE** before they touch
  something else. Warning; yellow if strategic.
  - Camping under the HIVE or using an open-top catcher looks strategic.
  - A couple landing on top while driving by is fine.
- **G410:** **NECTAR into a FLOWER only in the last 60 s.**
  **MAJOR FOUL per NECTAR.** The field timer is the cue.
- **G411:** **No hoarding.** Don't corral balls or block the other alliance from
  reaching them. **MAJOR FOUL plus yellow per match.** Using your own balls to score
  is fine.

### Robot
- **G412:** Robot must be under control and must not hit people or things outside the
  field. **DISABLED plus warning**; a yellow for later violations.
- **G413:** When the referee says stop, press stop and set the controllers down.
  Warning; RED if strategic.
- **G414:** Team number and alliance color must stay readable.
- **G415:** Don't grab, grasp, attach to, entangle with or hang from field elements.
  Balls are the exception. A concave shape that wraps partly around a FLOWER for
  alignment is OK.
- **G416:** Stay within the expansion limit (R105) and don't deliberately drop parts.
  Warning; MAJOR FOUL per instance if strategic.
- **G417:** **Don't meddle with the HIVE.** The only allowed way to move it is
  launching into the **up-facing CELL**. Warning; MAJOR FOUL plus yellow if strategic.
  - Strategic: ramming the frame, deliberately shooting at a CELL's outside
    bottom/sides/top, launching at the **opponent's** HIVE to block their tip, or
    touching the HIVE directly or through a held ball.
  - Not strategic: accidental bumps while collecting, and honest misses that hit a
    CELL's outside.
- **G418:** **Don't meddle with FLOWERS.** Balls go **in through the top only**, and
  only **POLLEN** comes out the bottom. Warning; MAJOR FOUL plus yellow if strategic.
  - Strategic: pulling balls through the side, shaking the FLOWER, pushing NECTAR out
    the middle ring, slamming the wall to knock balls out, or holding NECTAR against
    the pipes so it "counts."
  - Fine: bumping it while scoring, or knocking one ball out while scoring another.

### Opponent interaction
BIOBUZZ is high-contact and defense is legal, so build robust. G419 and G420 don't
stack; only the worst penalty applies.

- **G419:** Don't damage or functionally impair an opponent robot. Warning; MAJOR FOUL
  plus yellow if strategic; MAJOR FOUL plus **RED** if strategic and they can't drive.
  - Damage to exposed wiring, power switches or delicate intakes in normal play
    usually isn't a violation.
- **G420:** Don't tip, attach to or entangle opponents. Same scale; red if continuous
  or they can't drive.
  - Wedges and hitting a robot while it's trying to right itself look strategic.
  - A single normal hit that tips someone isn't strategic.
- **G421:** **3-second pin limit.** A pin is blocking an opponent's movement directly
  or against a field element. **MAJOR FOUL plus another every 3 s.** The pin ends
  when:
  - the robots are separated by 2 ft for 3 s, or
  - either robot has moved 2 ft from where the pin started, for 3 s, or
  - the pinner gets pinned.

### Humans
- **G422:** Drive team stays in the alliance area. You may grab your own NECTAR that
  left the field if you can reach it from there.
- **G423:** Only drivers touch the gamepads. The coach may hold or troubleshoot the
  driver station and press INIT, start or stop.
- **G424:** Drive coach doesn't touch balls. Minor foul if strategic.
- **G425:** Once the match starts, no touching the robot, balls on tiles or robots,
  or field elements, and no disrupting scoring (like shaking the wall). Warning; MAJOR
  FOUL plus yellow if strategic.
- **G426:** **NECTAR entry limit:** 1 per own-HIVE TIP, or all once 60 s are left.
  **MINOR FOUL per NECTAR.** You don't have to enter it right after a tip.
- **G427:** Human players enter NECTAR **only in their LOADING ZONE**, by hand with no
  tool, and it must **touch the tile in the LOADING ZONE before touching a robot or
  field element**. Minor foul per NECTAR. Humans may never score directly into a HIVE
  or FLOWER. Hand any POLLEN or opponent NECTAR that leaves the field to field staff.
- **G428:** Humans don't remove balls from the field. Minor foul per ball.

### Out-of-field balls
- POLLEN that leaves the field is put back by field staff near where it left.
- NECTAR that leaves the field is returned to that alliance's drive team to re-enter
  through the LOADING ZONE.

## 8. Robot construction rules (R): the ones that matter most

### Size and build
- **R102:** Starting configuration fits in an **18 × 18 × 18 in** cube, fully still.
  Pre-loaded balls may stick out.
- **R103:** You may hold starting configuration mechanically, or by an INIT OpMode
  holding motors and servos still. Avoid stalled motors cooking during long waits.
- **R104:** **No weight limit.**
- **R105:** After the start, the robot may expand but must always fit in
  **18 × 24 in footprint × 29 in tall**.
  - The limit must be **physically enforced; software limits don't count.**
  - It applies at every point of a rotating mechanism's travel, which is relevant for a
    turret.
  - Two separate mechanisms extending out opposite sides at once must fit together.
  - Don't deliberately detach parts.
- **R201–R204:**
  - Nothing that damages the field or makes a mess, and nothing hazardous.
  - The robot must be removable without power, with balls removable from it.
  - **No grabbing the floor or suction** for downforce.
- **R301–R305:**
  - COTS mechanisms built to do a whole game task are not allowed.
  - COTS parts must be single degree of freedom.
  - Legal COTS parts and raw materials may be modified.
  - **Past seasons' code and designs may be reused.**
  - Real or replica scoring elements can't be used as robot parts.

### Robot signs
- **R401–R403:** at least 2 robot signs on surfaces at least 90° apart, each with a
  solid red or blue panel at least 6.5 × 2.5 in, with the team number shown.

### Motors and servos
- **R501:** Legal motors include:
  - goBILDA Yellow Jacket 5201/5202/5203/5204 and the 5000 series
  - REV HD Hex and Core Hex
  - AndyMark NeveRest
  - Studica Maverick
  - SWYFT Spike
  - TETRIX MAX TorqueNADO
  - WATTOS Stingray
  - NFR Yuksel

  The full list is in Table 12-1.
- **R502:** Servos must meet the stall-current and mechanical-output-power limits at 6 V
  (Table 12-2), checked with FIRST's online calculator or the Inspection Quick
  Reference.
- **R503:** **Max 8 motors + 8 servos**, counting every mechanism used in any
  configuration at the event.
- **R504:** Don't modify motors or servos, except mounting, output shafts and pinions,
  lead length and connectors, and manufacturer-approved settings.
- **R505–R506:** Actuators must be controlled through approved devices. No relays,
  electromagnets or solenoids.

### Power
- **R601:** One 12 V NiMH main battery.
- **R602:** Other batteries only for self-contained peripherals and LEDs.
- **R603:** One main power switch.
- **R604–R613:** Fuses as directed; frame not used as a current path; inspectable
  electronics; wire sizes and colors; and so on.

### Control and vision
- **R701:** One controller: a REV Control Hub, or an Android phone plus Expansion Hub.
  Up to 1 extra Expansion Hub.
- **R702:** You can't reprogram coprocessors, **except the Limelight 3A, which is
  allowed and reprogrammable.** The Limelight 3G, OpenMV and OAK-1 are **prohibited**.
- **R704:** No other wireless; only the official robot Wi-Fi.
- **R707–R708:** **USB is only for vision devices** (UVC webcams, Limelight 3A),
  USB hubs or switches, and an Expansion Hub. No stereo cameras.
- **R709:** Recording cameras like a GoPro are OK with wireless off.
- **R710:** Lasers only as part of a sensor, Class I, non-visible.
- **R801:** No pneumatics you charge yourself, no blowers or vacuums.

### Driver station
- **R901–R904:**
  - One approved Android driver station.
  - Touch screen reachable.
  - Console no bigger than 3 ft × 1.5 ft × 2 ft.
  - No wireless besides the robot connection.

## 9. Things easy to miss

1. **INIT must not move the robot** (G304.H), and nothing may move in the 8 s
   transition (G403).
2. The **tags face down** from ~4 ft up and **move** with each TIP. The visible cluster
   for your color tells you which CELL is up.
3. **4-ball control limit** (G407), and opponent NECTAR is off limits (G408).
4. **No NECTAR in FLOWERS before 1:00** (MAJOR per NECTAR), and no extra NECTAR entry
   before 1:00 except 1 per TIP (minor per NECTAR).
5. **Only shoot at your own up-facing CELL.** Shooting the opponent's HIVE to block a
   tip is a strategic G417 foul.
6. **Don't camp under the HIVE** to catch spilled balls (G409).
7. **Robot size limits must be mechanical**, including a turret's full rotation (R105).
8. **SWARM RP (16 points)** needs both robots to LEAVE and PARK.
   **POLLINATOR RPs** need 4 and 7 TIPS per match.
