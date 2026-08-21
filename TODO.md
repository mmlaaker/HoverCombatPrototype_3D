# TODO

**Everything unfinished, and nothing else.** Verification debt, blockers, known traps, pending
decisions, unimplemented features.

**When an item here is finished it LEAVES this file.** Its outcome goes to `CLAUDE.md` (what the
system now does and why), its measurements go to `TuningLog.md` (how the number was arrived at), and
its number is retired at the bottom of this file. Nothing closed is kept here, however good the
reasoning was. This file is read to find work; anything that is not work is cost.

## The documents

| Question | Document |
|---|---|
| **What is not done?** | **`TODO.md`** (this file) |
| What exists, how does it work, why is it built that way? | `CLAUDE.md` |
| Which unfinished things gate the next milestone, and in what order? | `ROADMAP.md` |
| How do I measure this without fooling myself? | `Measuring.md` |
| How was a shipped number arrived at, and what was rejected on the way? | `TuningLog.md` |
| What should this be? Why? | `GameDesignDocument.md` |
| Physics derivations and method, frozen at `4a34f21` | `PhysicsAudit.md` |

Two things are deliberately absent from this file and live in `CLAUDE.md`: **consciously accepted
limitations**, and **theories disproved by measurement**. Read that list before investigating any
symptom that sounds familiar; it exists specifically to stop a session being spent on a dead end.

## Rules that apply to everything below

- **Live values come from `Assets/Data/VTP_Default.asset` and `Assets/Data/WD_*.asset`, never from a
  doc.** Numbers quoted here are the ones an item was written against and may already be stale. Items
  quote a value only where the value IS the defect.
- **Pair every derivation with one cheap runtime check.** An entire audit section was once
  arithmetically perfect and described a branch that had never executed.
- **Never trust a zero you have not proven can be non-zero.**
- **Weight owner feel-reports above both the docs and the derived readouts, and check the instrument
  before checking the game.** The owner's instinct has beaten the record four times.

**Item numbers are permanent IDs and are never reused.** Gaps are retired or renumbered items, both
listed at the bottom. `ROADMAP.md` points at these numbers, so a number that moves tier gets a new
one rather than inheriting someone else's.

---

## How work is tiered

| Tier | What belongs in it |
|---|---|
| **0** | **Movement and camera. TOP PRIORITY, and specifically what gets worked next.** |
| **1** | Blocks the combat loop |
| **2** | Feel and feedback that is not movement, **or is movement blocked on art or VFX** |
| **3** | Traps and code quality. Not currently visible, will be misdiagnosed when they are |
| **4** | Verification debt and instrumentation. Nothing here blocks play or development |
| **5** | Design decisions with no engineering blocker. Needs an owner call, not code |
| **6** | Unimplemented features |

**The sorting rule, set by the owner 2026-08-16: anything movement or camera related belongs in
Tier 0, unless it is blocked by art or VFX, in which case it defers to Tier 2.** Weapons come after
movement, so weapon-subject work stays out of Tier 0 even when the symptom is a camera one.

| Tier | Open |
|---|---|
| **0** | 0.12 |
| **1** | 1.1 · 1.2 · 1.3 · 1.4 · 1.5 · 1.6 |
| **2** | 2.2 · 2.4 · 2.5 · 2.7 · 2.9 · 2.11 · 2.12 · 2.13 · 2.14 · 2.15 · 2.16 · 2.17 · 2.18 · 2.19 |
| **3** | 3.1 · 3.2 · 3.3 · 3.5 · 3.6 · 3.7 · 3.8 · 3.9 · 3.10 · 3.11 · 3.15 · 3.16 · 3.17 |
| **4** | 4.1 · 4.2 · 4.3 · 4.4 · 4.5 |
| **5** | 5.1 · 5.2 · 5.3 · 5.4 · 5.5 · 5.7 · 5.8 · 5.11 · 5.12 · 5.13 · 5.14 · 5.15 |
| **6** | 6.1 · 6.2 · 6.3 · 6.4 · 6.5 · 6.6 |

### The agreed working order for Tier 0

**Set with the owner 2026-08-16 and approved. Bugs violating a stated hard requirement outrank
tuning and features.** Struck items are done.

1. ~~**0.12**~~ resolved by tuning rather than by the code fix, see the item
2. ~~**0.22**~~ shipped 2026-08-17, outcome in `TuningLog.md` > Aiming and the hover sensors
3. ~~**0.14**~~ shipped and closed 2026-08-17. Revisit deferred to pre-alpha 2 as **5.12**
4. ~~**0.23**~~ closed 2026-08-17. **It was a suppression bug, not a missing cue**, and no cue was
   built. The drift VFX half remains as 2.12
5. ~~**0.20**~~ shipped 2026-08-17, outcome in `TuningLog.md` > The boost jolt at a standstill.
   **The mechanism this list recorded was wrong** and three runs disproved it; see the entry
6. ~~**0.24**~~ **retired 2026-08-17 without being built.** Deferred behind the thruster replacement
   by owner decision; the successor carrying the measurements is **2.14**
7. ~~**0.21**~~ shipped and **judged good 2026-08-17** at `extraFallGravity` 35, outcome in
   `TuningLog.md` > Fall gravity and airtime. The owner drove it and supplied the acceptance test:
   two barrel rolls still land
8. ~~**0.13**~~ shipped and **judged good 2026-08-17**. The player swings the camera around a downed
   craft; the handback was closed by the owner on play evidence against the assistant's measurements
9. ~~**0.18**~~ shipped 2026-08-17. **The threshold was never the defect** -- the arming clock now
   decays instead of resetting, and the threshold stayed at 2. Outcome in `TuningLog.md` > The
   downed window > Closed as 0.18
10. ~~**0.16**~~ **retired 2026-08-17 without being judged.** The scoping that hides it is intentional
    and confirmed by the owner; the successor is **2.15**
11. ~~**0.19**~~ shipped 2026-08-17. Three preview states that break the nose-follows-travel
    assumption; the two defects that hid in that gap are now judgeable with the game stopped
12. ~~**0.26**~~ fixed and **judged good 2026-08-17**, outcome in `TuningLog.md` > The boost gate was
    a step against reversing. The engage cost it carried was played and accepted
13. ~~**0.25**~~ shipped 2026-08-17. The ride height is now the charge meter. **The design question
    this item carried was answered by the owner: the squat EXPRESSES the charge, it never stores
    it.** Outcome in `TuningLog.md` > The charge squat

**Added after that list was set, and it continues it. 2026-08-19 ran `0.29`, then `0.30`, and both
closed the same day**; `0.29` was taken ahead of `0.28` because it was the live complaint, and `0.30`
is what `0.29`'s fixes uncovered once the craft stopped fighting itself.

**The pre-alpha 1 movement playtest, four testers, 2026-08-19, re-set this list.** The owner's order,
approved 2026-08-20, is **`0.27`, then `0.33`, then `0.32`, then `0.36`, with `0.12` last.** Those five
are the whole of Tier 0 and the whole of what stands between here and PRE-ALPHA 1: **the owner's
completion criterion is those items built AND judged good in play.** **All four of the built items cleared that bar on 2026-08-20** — `0.27` in the morning, then `0.33`,
`0.32` and `0.36` together in one drive. **Only `0.12` remains, and it is a decision rather than a
build.**

14. ~~**0.27**~~ shipped and **judged good in play 2026-08-20**, the same day it was taken. The owner
    drove both repro cases and could not get stuck in either. Outcome in `TuningLog.md` > Closed as
    0.27; the design rules it leaves behind are in `CLAUDE.md` > Modules > Foundation and > Standing
    Decisions. **The milestone blocker is closed and `0.33` is next.**
15. ~~**0.33**~~, ~~**0.32**~~, ~~**0.36**~~ — **all three shipped and judged good in ONE drive,
    2026-08-20.** Owner: *"genuinely better than the playtest build ... less arcade-y."* The
    build-three-then-judge-once sequencing worked exactly as intended and is the reason they are one
    entry rather than three. Outcome in `TuningLog.md` > The speed pass; acceptance criteria in
    `CLAUDE.md` > Judged good 2026-08-20. **`0.12` is now the only Tier 0 item left, and the
    milestone turns on a single decision rather than on any build.**

**What the closing pass actually found, because it contradicts what `0.33` was written on.** The
complaint was the CEILING, not the ramp. `topSpeed` 80 to **105** with `maxForwardAccel` untouched
delivered both halves of the owner's ask at once — the three-second ramp they asked for AND a quicker
approach to every speed below it — because `accelCurve` is read as a fraction of the cap, so the cap
is the ramp's horizontal scale. **`0.33` had explicitly ruled `topSpeed` out of scope** on the
grounds that no tester asked for more speed. That inference was the error, and it is now recorded as
a standing rule in `CLAUDE.md`: **absence of a request is not a bound.**

**`0.33` deliberately precedes `0.32` even though `0.32` had two independent votes and `0.33` had one.**
They are the same subsystem, and the acceleration shape decides how much time a player spends at the
speeds where a steering fade would be felt. Tuning the fade first means re-tuning it after the curve
lands.

**Five Tier 0 numbers were retired to lower tiers in the same pass**, by the sorting rule: `0.28` and
`0.31` were open, and `0.34`, `0.35` and `0.37` were issued during the playtest intake and never
written here. See Retired numbers. **What is left in this tier is gated on nothing.**

### Before the next playtest build

Cheap things that make a long session worth more, or that stop it producing an unreadable result:

- **The marker key and the pad both work in a build, confirmed across the four-tester session
  2026-08-19.** This closes the half of **4.4** that was about the marker, and it closes the
  `PlaytestSession` question below it: Options, hold `P` and hold Share were all driven by a physical
  button in a build for the first time. **What 4.4 still wants is the allocation figure under
  sustained weapon fire**, which a movement playtest could never supply.
- **Markers are only as good as the instruction to press them.** The 2026-08-19 session dropped none,
  by choice: the owner was watching every run and diagnosed the soft locks live. That worked at four
  testers with one observer and it does not scale. **If anyone is ever tested without the owner
  watching, they have to be told to press `M`**, or the trace is a wall of undifferentiated rows.
- **Watch:** the speed look-ahead is now rate-limited to 8 m/s of look-point travel (`TuningLog.md` >
  The boost jolt at a standstill). If the framing feels like it lags behind hard acceleration, that
  is the first number to suspect. It also changes how the aim point retracts after a collision at
  speed, which nobody has judged yet.
- **Watch:** the two-barrel-roll landing margin is **0.09s at `extraFallGravity` 35, down from 0.12s
  at 30.** The owner judged 35 good on the explicit condition that two rolls still land, so this is
  now an acceptance criterion rather than a caution. **The specific new risk:** at 30 the 50-degree
  landing roll tolerance was wide enough to cover the whole margin, so you could roll straight into
  the ground and still pass. At 35 it no longer is, so a failure will read as "I finished the roll
  and it still didn't count" — suspect this, not the trick scorer (`TuningLog.md` > Fall gravity and
  airtime).
- **Watch:** aiming no longer moves the craft at all (`CLAUDE.md` > Standing Decisions), so the nose
  can now approach terrain the hover sensors do not look at. An unexplained scrape while aiming down
  over a rise is that, and `TuningLog.md` records the middle option that fixes it.

---

## Tier 0 — Movement and camera. TOP PRIORITY

**This is the working set.** Fifteen movement behaviours were judged good in the movement playtest and
are recorded in `CLAUDE.md` > Standing Decisions, joined 2026-08-16 by boost drive mode, speed and
turning, drift, and the strafe/drive speed ratio. **Read those before changing anything here:** most
items below are bounded by something already confirmed to feel right, and those confirmations are the
acceptance criteria.

**THIS TIER IS EMPTY OF WORK. PRE-ALPHA 1 IS COMPLETE**, owner's decision 2026-08-20. `0.33`, `0.32`
and `0.36` closed together, and **`0.12` was deferred as trivial by the owner** rather than fixed.
See `ROADMAP.md`.

**`0.12` stays written down, because deferred is not closed.** It still technically violates a hard
requirement — a sliver of bumper clips on the pitch-up, for a fraction of a second. The owner has now
declined it twice, most recently as "trivial" when it was the last thing standing between the project
and the milestone, **which is a stronger signal than the earlier "no big deal right now"**: it was
weighed against shipping and lost. Do not re-raise it on its own. **Fold it into the next camera
framing work if that ever happens**, and treat the hard requirement as having an owner-granted
exception rather than as an open bug.

**The two-barrel-roll acceptance test was re-driven and HOLDS**, owner 2026-08-20, after the speed
pass. That closes the last outstanding criterion on `0.21` and confirms what the argument predicted:
the pass moved horizontal caps only, and `extraFallGravity`, `airRollTorque` and the three jump
impulses were untouched, so trick margin was never in the path.

### 0.12 A sliver of bumper still clips on the pitch-up

**CONFIRMED STILL PRESENT 2026-08-17.** Owner, asked to glance at it after four separate changes to
the drive camera: *"0.12 is still true"*. So the residual is not something the framing rework, the
look-ahead rate limit, the boost gate slew or the downed orbit happened to fix along the way, and it
should stop being treated as probably-already-gone. It remains accepted rather than fixed.

**LARGELY RESOLVED 2026-08-17 by raising `minFrameMargin` to 5**, which is a PREFAB-VARIANT override
stored in `HoverCar_Prototype.prefab` (committed at `137b6c7`), not a scene override and not a change
to the base `HoverCar_PlayerController.prefab`, which still reads 2. Checked 2026-08-17 after this
file described it as a scene override and a scene-wide search found nothing: the live value is 5. The severe form is gone: what was measured at
-0.0404 and -0.1090 of viewport (craft fully off screen) is now, in the owner's words, "the bumper
just barely" for "like a fraction of a second". **Owner has accepted it as-is: "no big deal right
now."** Left open only because it still technically violates the hard requirement.

**Three causes are now eliminated, which is most of the value of this entry.**

1. **Chassis attitude.** The residual reproduces PARKED and LEVEL, where the craft has no attitude to
   contribute, so it cannot be the cause of what remains. An oriented-box fix was written, measured
   and backed out by the owner in favour of the tuning change; the A/B is worth knowing about anyway,
   because it showed the whole preview sweep bit-identical except `Inverted`, which had been graded
   identically to `Resting` and moved to 6.458 / 27.225.
2. **The framing guard.** Swept the parked pitch-up at every elevation from neutral to full stick:
   `guardScale` 1.000 and `guardRemoved` 0.000 in **every** row. It never fires here, so it cannot be
   mis-tuned into causing this.
3. **Hull geometry.** The modelled box, the collider union and the mesh renderers were read live and
   agree to three decimals: centre (0.000, 0.877, 0.732), extents (1.907, 1.174, 3.439).

**What is left is the camera lag blind spot, and it is now the only candidate.** The solved margin
never drops below 4.325 degrees during a parked pitch-up, which is 0.067 of viewport height against a
recorded settled margin of +0.0806. The rendered frame only has to fall about 7% of screen height
behind the solved one to clip, and the rig is documented as trailing its request by up to 2.498m
during a fast stick-up. **Three levers if it is ever worth more:** reduce `pitchLookAhead` /
`pitchLookLift`, raise `minFrameMargin` further, or fix the lag itself with a rate limit on elevation
change. The third is the honest one and deserves its own session.

**`Measure` IS STILL ATTITUDE-BLIND, and `minFrameMargin` 5 is partly absorbing that error.** Measured
2026-08-16: the model over-reports vertical margin by up to 2.5 degrees at 25 degrees of roll, against
a budget of 5. **If the attitude blindness is ever fixed, 5 becomes over-conservative and must be
re-judged**, not kept.

*Historical note: this item previously stated `Measure` used a bounding SPHERE. It does not and did
not; the eight-corner box loop was already there. The stale wording came from this file's own doc
comments, since corrected. The real defect is that the drive rig runs on two transforms (Follow is
the yaw-only heading proxy, LookAt is the vehicle, and `targetOffset` lives in the vehicle's space)
while `Measure` treats both as one frame.*

**Two theories are dead. Do not re-run them.**

1. **Orbit geometry.** The settled pose at full stick-up fits comfortably, so `pitchUpDistanceGain` was
   never the cause. It would have bought margin without touching the mechanism.
2. **Camera lag.** The rig really does trail its solved pose by up to 2.498m during a fast stick-up, so
   the guard genuinely is certifying a frame nobody is looking through. Feeding it the rig's REAL
   position was implemented, measured and backed out: it changed the worst margin by -0.0009 at 60 m/s,
   and at 45 m/s the guard performed slightly WORSE than being switched off entirely. Correcting the
   position while still assuming an ideal aim makes the model less self-consistent, not more.

**The cause is localised and the remaining work is a shape change in `Measure`, not tuning.** The clip
reproduces readily on real terrain and **does not reproduce at all on flat ground with the craft held
level, at any speed, with the guard on, off or modified.** So the missing variable is chassis attitude.
`Measure` treats the hull as a bounding SPHERE, which is attitude-independent, while the thing that
actually leaves frame is the rear-bottom corner of an oriented box. Two independent confirmations:

- **The sphere model is provably attitude-blind.** `EvaluateState` grades `Inverted` identically to
  `Resting` to three decimals: `guardScale` 1.000 both, `guardRemoved` 0.000 both, vertical margin 7.53
  both, horizontal 27.32 both. Turning the craft upside down changes nothing the framing check can see.
- **The real silhouette swings, and asymmetrically.** Sweeping the measured hull box through a full
  rotation against a fixed camera: **pitch overflows the BOTTOM edge by up to 24.3% of screen height**,
  clipping continuously from roughly 105 through 210 degrees, while **roll overflows by at most 7.7%**.
  Same camera, same craft, same box. The bottom edge matches the terrain measurements exactly.

*Caveat on those percentages: the sweep held the camera at the resting pose with the craft parked, so
the absolute figures are not the in-flight condition. The pitch-versus-roll comparison at a fixed
camera is the valid part, and it is a 3:1 difference the model cannot represent.*

**Constraint before starting:** `Measure` is deliberately shared by `EvaluateState` and the framing
guard so the inspector cannot promise a margin the runtime does not deliver. Any change lands on both
at once and the readout must be re-judged with it.

**Verification harness worth reusing:** the oriented-box viewport check written for this investigation.
Eight corners of the measured collider union through `Camera.WorldToViewportPoint`, reporting worst
margin and which edge broke. It is the only instrument here that measures the frame the player sees
rather than the frame the solver intended.

**Test discipline, and it cost three bad conclusions:** the first runs let the craft accelerate from
wherever the previous test left it, so no two were comparable and one apparent regression was pure
starting-state noise. **Pin position, rotation and velocity before any camera framing test.**

---

## Tier 1 — Blocks the combat loop

The checklist item "Basic combat loop (primary fire + pickups)" is blocked by 1.1 through 1.4 together.
**None of them is independently sufficient.**

### 1.1 Death is terminal; respawn is a stub

`VehicleHealth.HandleDeath()` sets `IsAlive = false`, fires `OnDeath`, and calls
`gameObject.SetActive(false)`. Once disabled the vehicle's own MonoBehaviours stop, so it cannot revive
itself; something external must hold the reference and call in. **Nothing does.** There is no match
manager, round system or spawn controller.

`VehicleHealth.Respawn()` exists but has no callers, and is incomplete even if called. It sets health,
`IsAlive` and re-enables the object. It does NOT:

- reset position or velocity, so you respawn where you died at whatever velocity you had, possibly
  inside geometry
- fire `OnDamaged`, so the HUD health bar stays at 0 until the next damage event
- reset energy or ammo
- grant invulnerability (see 1.2)
- reset `AIHoverInput._state`, which `HandleDeath` latches to `Dead` and nothing clears. **A respawned
  AI would be inert forever**, its `Update` hitting `case AIState.Dead: ZeroAllOutputs()`.

`EnemyHealthBar` is the only thing already respawn-safe.

**Done looks like:** a decision on who owns respawn (match manager vs the vehicle itself), spawn point
selection, and a `Respawn` that resets the full vehicle state including AI FSM state and HUD.

**`PlaytestReset` does NOT advance this despite looking adjacent.** It is a held-button escape hatch that
restores pose, velocity and energy, and deliberately nothing else: health, shield, ammo and weapon slot
are all left alone, which is exactly what keeps it short of being a respawn. (Energy joined the list
2026-08-18 as an owner call for the playtest, behind a `restoreEnergy` tickbox; the reasoning is in
`CLAUDE.md` > Instrumentation.) It does settle
one design question by demonstration: **the position-and-velocity half of a real respawn needs
`HoverCameraController.NotifyVehicleWarped` or the camera flies across the level**, and the same is true
for every future spawn point, checkpoint or teleport.

### 1.2 Post-respawn invulnerability does not exist

`VehicleHealth.SetInvulnerable(float)` has no callers. The machinery is entirely live: the flag, the
timer, the `Update` tick, the `TakeDamage` early-out and an `[INVULN]` gizmo label. **It simply can never
be switched on.** Both `CLAUDE.md` and `VehicleHealth`'s own header describe a post-respawn grace period
that has never run.

Separate clearly: **shield invulnerability works**, through a different path. Only the timed grace is dead.

Pairs with 1.1: respawning without it is spawn-camping by construction.

### 1.3 Four of six implemented weapons deal zero damage

`WD_Missile`, `WD_SoftHomingMissile`, `WD_HardLockMissile` and `WD_Shotgun` are all at `combat.damage: 0`.
Only `WD_ChainGun` (1) and `WD_MachineGuns` (0.5) do damage.

Not cosmetic: both delivery paths are live and both are being handed zero. The combat loop can therefore
neutralise a player completely but never finish them. **That is a coherent design if chosen deliberately
and a hole if not.** The inspector now warns on any definition sitting at 0.

**Watch out when picking numbers:** for ParticleSystem weapons `damage` counts PER PARTICLE, so the
Shotgun's 30-pellet burst applies it thirty times.

**Blocks:** 1.1 is pointless without this (nothing can die), 5.11 cannot be answered without it (no combat
pressure), and the self-damage suppression built into the rocket only matters once this is non-zero.

### 1.4 Every weapon has unlimited ammo

All six definitions have `maxAmmo: 0`, which `WeaponSlot.Initialize` turns into `currentAmmo = -1`. The
design specifies limited pickup ammo for everything except the Machine Gun.

Knock-on effects while this stands:

- `RefillAmmo` is a no-op (see 3.1)
- `OnAmmoDepletedForSlot` is **unreachable**, not merely unsubscribed: both invocations sit inside
  `if (slot.currentAmmo > 0)` and `currentAmmo` is permanently -1
- pickups have nothing to restore

### 1.5 `PickupManager` is unwritten

The only planned-and-entirely-absent gameplay module. Design calls for Ammo, Energy Cell and Health Repair
pickups, with **placement as the primary mechanism for forcing engagement** (powerful pickups in exposed
positions). Depends on 1.3 and 1.4 to have anything meaningful to give.

### 1.6 The AI cannot fire anything except slot 0

`AIHoverInput` never sets `FirePressed`, so it cannot fire any `SingleShot` or `Missile` weapon, including
the Shotgun it actually carries in slot 1. It also never sets `CycleWeaponNext/Prev`, so it never leaves
slot 0 regardless. **Two independent causes; fixing one alone changes nothing.**

Net effect: the AI's entire offense is the Machine Gun at 0.5 damage per pellet, 8 pellets/sec. It carries
no Chain Gun, which is the only weapon with a reasonable time to kill.

Also deferred inside `AIHoverInput`: strafe, drift, jump, shield, EMP, all hardcoded false or zero. The
missing EMP is what makes 3.15 unreachable in play.

---

## Tier 2 — Feel and feedback

Non-movement feel, plus movement feel that is **blocked on art or VFX** and therefore cannot be worked
in Tier 0.

### 2.2 EMP launch has no acknowledgement

**Wired, feel unjudged, and NOT JUDGEABLE IN THIS BUILD**, because `empSource` is nulled on the scene
instance along with the EMP ability itself for the current movement-focused playtest (`CLAUDE.md` >
`HoverCameraImpulseRouter`). EMP is the most expensive ability in the game and empties
the meter you need in order to disengage. The projectile carries its own particle visual so the shot is
visible, but a commitment that large should confirm itself.

Shipped mechanism: its own impulse channel, a recoil thrown backward along the chassis so the launch reads
as a one-way shove.

The owner's first pass was "feels bad", and **most of that was the screen-space direction bug**
(`Measuring.md` trap 11): the craft's world heading was being applied as a screen direction, so the recoil
went sideways instead of backward. Now measured as a clean pull away from the craft, and the duration was
cut from a 0.35s wallow. **Needs re-judging after that fix.** Whether the weight is right for the game's
largest single energy commitment is still open; it currently sits at a little over half the landing punch.

*Filed here rather than Tier 0 because the subject is an ability, not movement. It becomes judgeable at
the same moment 2.15 does, when the three nulled impulse channels come back.*

### 2.4 Hard landings land with no dust

**BLOCKED ON ART.** `landingDustPrefab` and `landingDustHeavyPrefab` are both null on both vehicle
prefabs. They pointed into a VFX package that is no longer on disk, and `HoverVehicleVFX.HandleHardLanding`
null-checks and returns.

The whole hard-landing feature therefore currently delivers only the camera punch and the spring give. The
event, the severity split, the heavy/standard threshold and the ground raycast are all live and correct;
only the two prefab slots are empty.

### 2.5 Missile detonations have no explosion VFX

**BLOCKED ON ART.** `blast.explosionPrefab` is null on all three missile definitions, same removed package.
`RocketProjectile` null-checks before instantiating, so this fails silently rather than erroring.
Detonations currently produce no visual at all.

Note the dependency runs the opposite way from how it used to: the definition states `splashRadius` and the
VFX is authored to match it, not the reverse.

### 2.7 Automatic weapon fire rates are far too low

Owner note: both automatics "need to increase the firing speed a bunch to keep up with the reticle". Live
emission is **8/sec per Machine Gun barrel and 20/sec on the Chain Gun**.

Related: 3.6 records `WD_MachineGuns.combat.fireRate` at 0.01, which is a **separate value** from the
emitter rate. Both need to move together, and the definition is the single author since
`ApplyDefinitionToEmitter` pushes it into the emitter.

**Do not tune camera recoil before this item and 3.6 are fixed.** Recoil is wired, per weapon and opt-in
(`combat.recoilVelocity`, 0 on all six assets). It lands once per shot, so its right value is a function of
fire rate, and both automatics are currently misconfigured in opposite directions: the Chain Gun at 20/sec
would blur any kick into a tremor, and the Machine Guns at 0.01 would fire one kick every 100 seconds. Any
number picked now gets invalidated when the rates move. The single-shot weapons are tunable today and are
the sensible place to start.

#### Deferred: per-weapon recoil CHARACTER, not just strength

**Deliberately not built**, because the weapons are placeholder and shaping camera feel around them now is
tuning against a moving target. Recorded so the reasoning is not re-derived.

Only strength is per weapon today. Shape and duration live on a single shared impulse source, so every gun
that opts in shares one character. The owner found that insufficient immediately: **the shotgun wants a
short snap and a missile wants a longer envelope with a wind-up dip**, which is a difference in kind
rather than in size.

**Do not infer the split from `projectileMode`.** It was considered because it happens to separate the
current six correctly, and rejected: mode describes how a weapon spawns damage, not how it should feel. A
railgun would be Instantiated and want a sharp crack; a plasma stream would be ParticleSystem and want a
sustained push. Worse, the failure would be silent, and the only way to author around it would be to change
the projectile mode, which changes gameplay.

**And do NOT solve it by writing shape and duration onto the shared source before each shot.** That is the
obvious design and it is unsafe: an impulse reads its shape and duration off the live source component for
its whole life, so a still-playing impulse adopts whatever the next shot wrote. Full mechanism is
`Measuring.md` trap 14. The consequence is simply that **different characters need different sources.**

**The shape it should take when it is worth building:** a `recoilCharacter` enum next to `recoilVelocity`,
named for FEEL rather than for camera objects (`Snap`, `Swell`), with one source per character on the
camera and the router picking by name. The weapon says what kind of event it is; the camera owns how that
is expressed, so the weapon definition never learns a camera concept. Start with exactly the two that were
empirically found. `recoilVelocity` 0 stays the off switch, so the enum only ever picks character. Cost per
character is one GameObject, one serialized field and one enum value, so do not create them speculatively.

### 2.9 Taking a weapon hit produces no camera feedback at all

*The rest of this item is closed: crash shake, landing punch and bad-angle tumble were all judged good, and
the queued `levelingTorqueStrength` change is withdrawn. **Do not make that change without a fresh
complaint** -- it was the only queued reason to modify `HoverController_Foundation`.*

Being shot is currently as silent as the denied jump was. **The crash path does not and cannot cover it:**
crash shake hangs off `OnCollisionEnter`, which needs a physical contact, and weapon damage does not arrive
that way. Rockets detonate from `ProjectileSweep` before touching, and particle weapons never had a
collision to begin with.

The hook exists and has no camera subscriber: `VehicleHealth.OnDamaged(currentHealth, maxHealth)`. Note the
signature carries HP rather than damage dealt, so the router would need to difference it to get a
magnitude, or the event needs a third parameter.

**Undecided on purpose, and worth deciding before wiring:** a shake on every hit is a real design choice,
since it costs aim stability at exactly the moment the player most needs it, and some games deliberately
refuse it.

**Also still untested: vehicle-on-vehicle collision.** The AI craft is on its own layer with its own mesh
colliders and should route identically, but nobody has driven it. Easy path: park the AI and drive into it.

### 2.11 Boost presentation FX: vignette, speed lines, duration-based rumble

**BLOCKED ON VFX, deferred to PRE-ALPHA 2 by the owner**, to be picked up with the general FX quality
pass. Their reasoning: the whole FX layer is currently basic, and building one polished effect against a
bed of rough ones is the wrong order. **This is the only reason it is not in Tier 0.**

**The deferral does not withdraw the diagnosis, and the sequencing note still stands:** boost strength in
BOTH modes is being judged with half its presentation missing, so **do not retune `boostAccelMultiplier`
or `boostSpeedMultiplier` on feel until it lands.**

**This is the fix for a live complaint, not a polish item.** Owner: "my boost feels like it does next to
nothing while strafing." **The physics is not the problem and must not be touched:**

| | Speed | Time to 95% | FOV | Camera distance |
|---|---|---|---|---|
| Strafe | 53.0 -> 66.4 (**+25%**) | 0.33s | **55 -> 55** | **6.95 -> 6.95** |
| Drive | 80.1 -> ~99 (**+24%**) | 0.34s | 65 -> 69 | 9.84 -> 10.25 |

Identical relative gain, identical time to arrive. **Strafe boost delivers everything drive boost does
and shows the player nothing at all**, because the lens was removed from strafe on the owner's
no-modifiers rule. Boost's felt impact is mostly presentation, which is exactly why the same physics
reads as "next to nothing" in one mode and fine in the other.

**Do NOT resolve this by restoring the FOV kick to strafe.** It is a genuine aim modifier here: the
reticle is a projected world point rather than a centred crosshair, resting ~438px above centre, and a
projected point moves radially with the lens, so the shipped +4 degree kick drags it **35 pixels**
before the overshoot is counted. The owner's rule is correct and better founded than the comment that
originally justified it.

**So the cue must touch neither the rig nor the lens**, which is precisely what the three requested
effects do. Vignette and speed lines are screen-space overlays: no camera movement, no zoom change, no
reticle displacement, and they read as speed more strongly than an FOV kick does. That makes them the
only available answer rather than merely a nice one.

**The rumble is the one to be careful with.** It is the first camera effect requested that builds with
DURATION HELD rather than firing on an event, and the boost envelope already tracks exactly that. But
rumble moves the camera, so in strafe it would move the reticle for the same reason FOV does. Either gate
it to drive mode, or make it a screen-space shake of the rendered image rather than a camera move.

### 2.12 Drift has no VFX

**BLOCKED ON VFX**, so by the sorting rule it defers here and joins 2.11 in the pre-alpha 2 FX pass.

Owner, 2026-08-16: *"Some VFX will probably go a long way for communicating a drift too."* Said in
the same breath as judging drift itself good, so this is presentation on a mechanic that is already
right, not a fix.

**Filed separately from 0.23 on purpose.** 0.23 turned out to be a suppression bug and closed without
a cue being built (`TuningLog.md` > The drift hop was being suppressed). This is the sustained visual
for the slide itself, and it is now the whole of what remains from the owner's drift report.

### 2.13 Charge jump wind-up VFX: energy gathering under the craft

**BLOCKED ON VFX**, so it joins 2.11 and 2.12 in the pre-alpha 2 FX pass.

The visual half of 0.25. Owner's reference is **Jak 3's hoverboard high jump**, where holding the
button gathers visible energy under the board and releases it on the launch. Here it would build
under the craft while the charge squat settles it lower, and discharge on the jump.

**The movement half SHIPPED as 0.25 on 2026-08-17** and is no longer a dependency. The squat exists,
the ride height reads the charge, and the mechanism is recorded in `TuningLog.md` > The charge squat.
Same split as 0.23 and 2.12.

**The constraint 0.25 hands this item, and it is a hard one:** the squat deliberately does NOT store
energy. Releasing the springs would stack on `jumpImpulseMax`, and it was measured doing exactly
that -- an instant snap-back added 1.0m to a 20.6m jump. So a discharge effect here must read as
release without any actual push, and nothing in the FX may be wired to add impulse.

**Sequencing note that matters here:** the squat now makes the charge readable through ride height
alone, so judge it in play before authoring this. If the squat already communicates the charge, this
is polish. If it does not, this is the thing that was actually missing.

### 2.14 Thruster VFX and the boost camera fall out of step on release

**BLOCKED ON VFX, and this is the successor to 0.24**, which was retired without anything being
built. See Retired numbers for why the tier changed.

Owner, 2026-08-16: *"I'd like to see the timing of the thruster VFX to be tied to the boost blend
directly, so they grow/shrink at the same rate as the camera move."*

**The VFX is ALREADY tied to the boost blend directly, and that is the cause rather than the fix.**
`HoverVehicleVFX` reads `Propulsion.BoostLerp`. The camera does not; it integrates its own envelope.
Measured 2026-08-17 on a scripted engage-and-release:

| | plume (`BoostLerp`) | camera (`_boostHold`) |
|---|---|---|
| rise | identical, sample for sample | identical, sample for sample |
| falls below 50% after release | 0.18s | 0.39s |
| falls below 25% | 0.27s | 0.59s |
| falls below 10% | 0.32s | 0.85s |
| falls below 5% | 0.34s | **still above it a full second after release** |

So they agree perfectly going up and disagree by **2.2x, widening to 2.7x at the tail**, coming down.

**THE CONSTRAINT THAT MATTERS FOR WHOEVER AUTHORS THE REPLACEMENT**, found 2026-08-17 by reading the
live component rather than the class defaults. Of the four parameters the tiers vary, `emissionRate`
is the only one that acts on particles already in the air — and it is **40 at idle, 40 at
acceleration and 40 at boost**, so it is doing no work at all. Everything that does change under
boost (`startSize` 0.5 -> 0.8, `startSpeed` 3 -> 4, `startLifetime` 0.2 -> 0.3) applies only at the
moment a particle is born. **The rendered plume therefore cannot track any signal promptly, no
matter what drives it**, and will always trail by up to one particle lifetime. That is a requirement
on the new effects, not a defect in the old ones.

**Why this deferred rather than being wired now**, owner's decision 2026-08-17: the current
thrusters are explicitly bare-minimum placeholder and are being replaced, and the replacement may
well vary different parameters. The rewiring itself would survive that, but two things argued
against doing it early — the defect is a timing disagreement between two plume states that are
barely distinguishable on this placeholder, so no playtester would see it; and it would mean tuning
the release rates against art that is about to be thrown away.

**Done looks like:** plume and camera rise together and settle together, with the emission-time lag
above accounted for rather than ignored.

**The likely shape, recorded so it is not re-derived:** one owner for "how boosty does this look
right now". Propulsion is the right owner, since it already owns `BoostLerp` and both the camera and
the VFX are downstream of it; `releaseSpeed` and `settleSpeed` would move from `CameraBoostTuning`
into the vehicle profile with it. Two consequences worth knowing in advance. The plume would inherit
the camera's engage overshoot for free, so it would spike and settle rather than merely ramp. And
the camera's `ForwardGate` must **not** travel with the signal: the thrusters genuinely are firing
at full when you boost from a standstill, even though the camera has decided not to dramatise it.
The gate is a camera judgement, not a thruster fact.

**No longer depends on 0.20**, which closed by limiting the look-ahead and never touched the boost
package at all.

### 2.15 A denied jump gives no usable feedback

**Wired, feel unjudged, and DELIBERATELY NOT EVALUATED BEFORE THE PRE-ALPHA 1 PLAYTEST.** This is the
successor to 0.16, which was retired without being judged. See Retired numbers for why the tier
changed.

**The open question 0.16 carried is now answered.** It asked whether `jumpDeniedSource` had travelled
with EMP and weapon recoil by accident, a denied jump being a movement cue rather than a weapon or
energy one. Owner, 2026-08-17: **it is intentional**, disabled along with the shield and the EMP, and
it will not be evaluated before the pre-alpha 1 playtest. So this is scoping, not an oversight, and
nothing should be restored to make it judgeable early.

The player presses jump, nothing happens, and there is no way to tell "out of energy" from "input
didn't register" from "still in the post-land lockout". This matters more than a normal missing cue
because **energy as tempo is a design pillar**: a resource the player is meant to manage as their
primary tempo tool currently gives no feedback when it runs out.

Shipped mechanism: a camera impulse on its own channel, thrown down the chassis axis so a denial while
banked still reads as failing to rise. The air value is smaller deliberately, because the token is not
consumed and the failure is not final. Audio is out of scope, which is why this landed on the camera.

The owner's first pass was "just feels bad". Unlike the crash and EMP channels this was NOT the
screen-direction bug, so it is a genuine shape problem. Changed from a 0.12s recoil to a 0.18s bump:
0.12s is seven frames and reads as a frame hitch, and a bump returns to where it started, so it lands
as a lurch rather than a one-way shove. **Still unjudged.**

**If it still reads wrong, the next lever is direction rather than duration.** A denial is a failure to
LAUNCH, so a brief up-twitch that falls back may be more legible than a downward sag.

**Judge it by draining the meter and pressing jump, not with the test hotkey**, which fires without the
empty meter that gives it meaning. Note jump costs changed (tap is now much cheaper than a full charge),
so how often denial arrives has moved.

*Filed here rather than Tier 0 despite being a movement cue, because what gates it is the energy and
ability scoping rather than anything about movement. It becomes judgeable at the same moment 2.2 does,
when the three nulled impulse channels come back.*

---

### 2.16 No reliable altitude cue: the realtime shadow degrades exactly as altitude grows

*Was `0.28`. **Retired from Tier 0 on 2026-08-20** and renumbered, because the owner gated it behind a
more representative arena and an arena is art. Nothing below changed; the analysis was done read-only
in the editor on 2026-08-19 and stands.*

**TWO THINGS THE PLAYTEST CHANGED, and both argue for the deferral.** This item was opened on
2026-08-19 with an explicit prediction: raised deliberately BEFORE the pre-alpha 1 playtest because
the owner expected it to come back from testers.

1. **It did not come back. No tester raised altitude legibility at all**, across four players and two
   runs each. That is not proof there is no problem — new players were not pushing altitude the way the
   owner does — but the prediction this item was filed on did not land, and the file should say so
   rather than quietly dropping it.
2. **The owner's own complaint named the environment as the cause** ("the ground is a tiled texture and
   there are no props in the test env"). Building a cue to compensate for a placeholder arena risks
   tuning it against conditions that will not exist. **Judge the arena first, then judge whether the
   cue is still needed.**

**Gate: a representative arena (6.2).** Owner decision 2026-08-20, alongside 2.17 for the same reason.


**Owner playtest feedback 2026-08-19, raised deliberately BEFORE the pre-alpha 1 playtest and
deliberately NOT acted on for it.** Owner: *"it's hard to judge how high off the ground I am and how
fast I am moving sometimes because the ground is a tiled texture and there are no props in the test
env, which is fine at this stage."* Expected to come back from testers as well, which is why it was
raised early rather than after the session.

**Superseded 2026-08-20, and the original wording is kept here because it dates the decision.** This
item read *"Owner decision 2026-08-19: this is the next thing worked"* and argued itself into Tier 0
on the grounds that it is movement legibility and is NOT blocked on art or VFX, since all it needs is
a quad, an unlit transparent material and a soft radial texture. **That reasoning was correct about
the assets and wrong about the gate.** What blocks it is not the material, it is not knowing whether
a real arena makes the cue unnecessary. The build cost was never the constraint.

**Why the one cue that exists fails, measured read-only in the editor 2026-08-19.** The scene has a
single realtime directional light at euler `(43.57, 12.30, 0.00)`, so the sun sits **43.6 degrees**
above the horizon and the cast shadow displaces **1.05m horizontally per metre of altitude**, in a
**world-fixed** direction. Three consequences, and they compound:

- At hover equilibrium the belly rides about 4.7m up, so the shadow already lands roughly one craft
  length away before the player has done anything.
- At a full-charge apex (about 20m above ride height) it lands about 25m away, which is either the
  blurriest cascade or past `m_ShadowDistance: 50` in `URP_RenderAsset.asset` entirely.
- Because the offset direction is world-fixed and the craft is not, yawing swings the shadow from
  ahead of you, to behind you, to beside you, with no altitude change at all.

**So the single altitude cue in the game degrades exactly as altitude grows, which is precisely when
it is needed.** The tiled ground and the deliberately sparse arena (6.2) mean there is nothing else
to read height against.

**What to build: a fake blob straight down under the craft, present at all times.** The standard 3D
platformer fix, and the vertical range here is closer to a platformer than to a racer -- Wipeout and
F-Zero craft ride a metre or two off the track, this one rides 4.7m and can be 25m up.

**The hovercraft reframing, and it changes the brief.** In a grounded car game the shadow is a
contact confirmation: near-zero offset, effectively binary, nobody reads it as a quantity. Here
altitude is a continuously varying quantity the player must read at every instant, on a craft that is
**never planted**. Two consequences worth designing to:

1. **Do not encode height in size alone.** Over a 0 to 20m range one channel is either invisible when
   high or a useless dinner plate when low. Use a coarse channel (size and opacity falling off with
   height) plus something that resolves sharply in the last metre or two, so touchdown reads
   precisely instead of asymptotically.
2. **Calibrate the resting state, not zero.** Because the craft never lands, "zero altitude" is not
   the interesting reading. What matters is whether you are at ride height, above it, or below it. If
   the blob has a distinct recognisable look at hover equilibrium, the player learns what normal looks
   like in a minute and reads every deviation against it. That is the part a car game's shadow never
   has to do.

**It is an altimeter, NOT a landing predictor, and this item should not drift into claiming
otherwise.** Falling from a full-charge apex takes about 0.74s at the live fall gravity, and
`topSpeed` is 80, so the craft can travel up to ~59m horizontally during the descent. A real "where
will I land" cue is a ballistic marker along the velocity arc, which is a different and much larger
feature. Do not build it as part of this.

**What this does NOT solve: the speed half of the same feel report.** Speed is read from optical
flow, meaning features fixed in the world passing you. A cue that travels with the craft supplies
none, so the blob will do nothing for speed and should not be sold as if it will. The project's
recorded position on speed readability is 6.2 (props are NOT coming to fix it, and the environment
doubling halved how fast the world goes past at a given speed) and 2.11 (screen-space speed lines,
deferred to PRE-ALPHA 2). **The one thing this item leaves behind for it:** the downward raycast
below is exactly the ground anchor a world-anchored ground wash would need, so a wash spawned at the
ground point and left behind in world space becomes cheap once this exists. 2.4's landing-dust
raycast is already live and correct with only the two prefab slots empty, so that hook is waiting
too.

**The double-shadow contradiction has to be resolved, not ignored.** A blob under the craft plus the
existing realtime cast shadow puts two shadows in two different places. **Recommended: blob only.**
Turn Cast Shadows off on the five vehicle mesh renderers (`3D/HoverCar/{Body, Rim_FL, Rim_FR, Rim_RL,
Rim_RR}`), which is a prefab tick with no code and also saves shadowmap cost. The alternatives, not
rejected, just not recommended: keep both and accept the disagreement, or keep the cast shadow and
build a ground-effect ring instead of a shadow, which suits a hovercraft and dodges the contradiction
entirely.

**Free A/B to run first, before any of this.** Rotate the light toward overhead and drive it. It is
**not** a fix -- it flattens the shading that makes greybox slopes readable, and at 20m the shadow is
still metres away -- but it costs nothing and it bounds how much of the complaint is the sun angle
rather than the absence of a cue.

**Implementation traps, all project-specific and all cheap to trip:**

- **It needs its own downward raycast.** The hover sensors cannot serve: `sensorRange` 9.5 minus
  `hoverHeight` 7 tops them out at 2.5m of clearance, so they are blind from any jump.
- **Cast and place it in `LateUpdate`, not `FixedUpdate`.** Both vehicle prefabs use Rigidbody
  interpolation against a 0.01 fixed timestep on a 165Hz display, so the physics pose is not the
  drawn pose. Placing a cosmetic in `FixedUpdate` makes it jitter against the craft it is attached to.
- **Mask out the vehicle layers.** Self-hits are the documented mechanism behind the drift flip.
- **Drop the ray from the hull centre, not the transform origin.** The collider union centre is
  `(0.000, 0.877, 0.732)` in vehicle-local space; that 0.732m forward offset has already caused one
  false reading, in the camera framing check.
- **Yaw only.** Align the quad to the hit normal, never pitch or roll it with the craft.
- **Hide it when nothing is in range** rather than pinning it at max distance. Loops, the screwtower
  and ledge edges will all produce no hit.
- **Author it on the prefab, not as a scene instance override** (3.16 and 3.17). `HoverCar_AI` is
  currently inactive in the scene, so only the player needs it to work for now, but the prefab is
  still where it belongs.

**Rendering: an unlit transparent quad aligned to the hit normal, NOT a URP Decal Projector, at least
first.** `URP_Renderer.asset` is `m_RenderingMode: 2` (deferred) with SSAO as its only renderer
feature, so adding a Decal Renderer Feature is a project-wide render change with deferred-path
caveats and it would invalidate the 6.07ms / 0.39ms perf baseline. Revisit decals only if quad
intersection artifacts on the loop, the mountains and ledge edges prove annoying in play.

**Done looks like:** the player can tell, without looking away from the craft, whether they are above
or below ride height, and can call a landing before it happens. **The owner supplies the acceptance
test**, as with 0.21 and 0.13.


### 2.17 The camera should be more active and automatic, with manual override

**Pre-alpha 1 playtest, 2026-08-19, two testers.** Tester 1 suggested a more active or automatic
camera that can still be overridden by hand. Tester 4 independently said they would prefer the same
thing, and **explicitly was not put off by controlling camera pitch themselves** — they called drive
mode basically perfect and the controls standard for the genre in the same breath.

**Gate: a representative arena (6.2).** Owner decision 2026-08-20: *the camera is good enough until we
have a better test arena to see if camera changes are necessary in that type of environment.* The
reasoning is the same as 2.16's. An automatic camera earns its keep by anticipating terrain and
framing what matters, and the current arena is an RVP demo city being driven as greybox. **Nothing
about how a camera behaves in a real arena can be learned in this one.**

**What already exists, so this is not a from-scratch feature.** `HoverCameraController` already solves
framing per state, already has drive, strafe, boost, reverse-boost, downed and nose-across-travel
states with an inspector preview (`0.19`), and already runs look-ahead with a rate limit. The question
here is authority, not capability: how much the rig decides on its own and how the player takes it
back.

**Do not confuse this with the framing revisit.** `5.12` revisits the AIM framing once there is
something to aim at. This is about the drive camera acting on its own. They will probably want judging
in the same session and they are different questions.

**Related, and it is why 2.18 sits next to this one:** the third tester's difficulty was the right
stick carrying both steering and camera pitch. That is a binding question, not a camera-behaviour one.

### 2.18 Control scheme: three testers reached for bindings the game does not have

**Pre-alpha 1 playtest, 2026-08-19.** Three separate mismapping reports, none of which cost anyone a
run, all of which say the same thing about expectations:

- One player reached for **Cross for gas**, old-school racing convention. Cross is `Jump`.
- One player reached for **R2 for gas**. R2 is `Fire`.
- One player expected **all steering on the left stick and all camera on the right**. The live scheme
  is the reverse for steering: `Turn` is right stick X, throttle is left stick Y.
- Tester 2 struggled with **camera pitch sharing the right stick with steering**, which is the same
  expectation expressed as a difficulty rather than a guess.

**The scheme survived contact, and that is the more important finding.** Players tried to invent
alternatives and **failed to produce one**, and reported the twin sticks as contextualized once they
understood the drive/aim mode switch. Recorded as judged good in `CLAUDE.md`. **This item is about
placement, not about the model.**

**Gate: the full control set exists.** Owner decision 2026-08-20: *bindings are easy enough to swap
but we should wait until we have all of the controls in before deciding if they are placed in the most
optimized or ergonomic spots.* Seven of thirteen weapons are unimplemented (6.1), pickups do not exist
(1.5), and three impulse channels are off. **Optimising a layout against a partial control set means
doing it twice.**

**One concrete ergonomic collision to carry forward, since it is the only one with a mechanism rather
than an expectation:** `Boost` is L3, a CLICK of the left stick, while air control reads that same
stick's DEFLECTION for pitch and roll. Boosting mid-trick asks one thumb to press and steer the same
input. Tester 3's report of boosting through tricks is this. See 2.19.

**Do not renumber the actions to fix a doc.** `HoverControls.inputactions` is the source of truth for
the control table in `GameDesignDocument.md` section 6, settled by `3.18`.

### 2.19 Air control and tricks need to be taught, not discovered

**Pre-alpha 1 playtest, 2026-08-19.** Tester 2 believed L1 and R1 were mirrored roll buttons, one for
each direction, **and never found flips at all.** Tester 3 kept disengaging air authority part way
through a rotation. Neither found the mechanic on their own.

**The binding is a fair reason to read it that way.** L1 and R1 are bound **identically** to `Drift`,
so two mirrored shoulder buttons genuinely do the same thing. Held airborne, left stick Y is pitch and
X is roll (`HoverController_Propulsion.cs`, `ApplyAirControl`). Nothing on screen says so.

**Tester 3's dropped rotations have a mechanism worth recording.** Tricks arm on the rising edge of
air control, once per flight (`CLAUDE.md` > `HoverController_Tricks`). Releasing the shoulder mid-
rotation does not disarm the trick, it removes the authority needed to COMPLETE the revolution, and
revolutions bank on the crossing. **So the trick was still armed and simply never finished**, which
from the player's side looks like the game not counting it.

**This is an onboarding gap, not a defect, and the playtest proved that.** Owner, 2026-08-20: the
tricks were there for everyone once they were told about the mechanics and given a run to feel them
out. **All four testers were landing tricks in their second run** — timing jumps off ramps, taking
secondary air off a bounce, jumping out of a drift. Whatever the discovery cost is, thirty seconds of
explanation clears it and the skill sticks immediately.

**Gate: the core loop, via the FTUE that will teach it.** Owner decision 2026-08-20: this is what a
first-time user experience covers, and an FTUE needs a loop to be taught inside of. Filed with 2.18
because both are the control scheme becoming legible.

**Consequence for reading future playtests, and the only reason this needs writing down before the
FTUE exists:** testers reach the trick layer in run two, not run one. **First-run data says nothing
about tricks either way.** Do not read a quiet first run as a negative result on the trick economy.

---

## Tier 3 — Known traps and code quality

**These are not currently causing visible problems.** Each is recorded because it will be misdiagnosed as
something else when it does.

### 3.1 `RefillAmmo` secretly resets cooldown and wind-up

Implemented as `slot.Initialize()`, which also resets `cooldownRemaining` and `windUpProgress` and rebuilds
`baseEmitterRates`. So picking up ammo will **cancel your cooldown and dump your Chain Gun spin-up**: a
three-second wind-up erased by driving over a pickup. Nothing in the method name suggests it.

Harmless today because there are no pickups and no ammo; a confusing "minigun bug" the day both exist.
**Fix:** split ammo restoration out of `Initialize`.

### 3.2 `ParticleWeaponCollision` walks the hierarchy per collision event

Both `GetComponentInParent<IDamageable>()` and `GetComponentInParent<Rigidbody>()` are called **inside** the
per-event loop, although `other` is invariant across the whole loop. A 30-pellet shotgun burst landing on
one target does 60 hierarchy walks instead of 2.

The file's own note defers this to "pooling debt", but the hoist is independent of pooling and is a two-line
change. Do the hoist now; keep the pooling note for later.

### 3.3 The AI steers away from its own target

`AIHoverInput.obstacleMask` defaults to `~0`, so the 5-ray avoidance fan treats the player vehicle as a
wall. At close range avoidance urgency fully overrides waypoint steering, meaning **the AI actively backs
away from the thing it is trying to shoot.**

Also casts from `transform.position + Vector3.up * 0.5f`, which is inside its own hull.

**Fix:** restrict `obstacleMask` to terrain layers.

### 3.5 `RocketProjectile.Explode` allocates

Uses the allocating `Physics.OverlapSphere` while the rest of the project is scrupulously non-alloc
(`RaycastNonAlloc` in the hover springs, `OverlapSphereNonAlloc` in the scans, static reusable buffers in
`ProjectileSweep`). **Detonations are bursty, so this is garbage generated at exactly the worst moment.**

### 3.6 `WD_MachineGuns.combat.fireRate` is 0.01

That is the `[Min]` attribute floor, giving a **100-second cooldown**. Almost certainly a slider dragged to
the bottom rather than an intended value. Compare `WD_ChainGun` at 20.

Currently invisible: for a ParticleSystem Automatic weapon the emitters play regardless of `IsReady`, and
`maxAmmo` is 0 so no ammo decrements. The only observable effect is that `OnWeaponFired` fires once per 100
seconds. **It breaks the moment the Machine Gun gets ammo (1.4).** Move it with 2.7.

### 3.7 `SetActiveSlot` does not skip empty slots

Unlike `TickCycleWeapon`, which walks past slots with a null definition, `SetActiveSlot(int)` accepts any
in-range index. Pointing it at an empty slot **leaves the vehicle silently weaponless**, because `Update`
early-returns on `definition == null`.

Both paths now share `SwitchToSlot`, so the empty-slot skip only needs writing once and belongs there.

### 3.8 `SetRecoveryEnabled` has a sharp edge for its first caller

`HoverController_Foundation.SetRecoveryEnabled(bool)` has no callers today and `recoveryEnabled` is
permanently true. EMP does not use it, since Foundation's `FixedUpdate` early-returns wholesale on an EMP
freeze and skips recovery anyway.

Before anything wires it up: `IsDowned = recoveryEnabled && (...)`. So **`SetRecoveryEnabled(false)` on a
downed craft immediately clears `IsDowned` and hands full control back to a vehicle lying on its back**,
which is the exact bypass that was closed in Propulsion v1.3. That is arguably correct (disabling recovery
should disable the lockout recovery exists to enforce) but it is a trap worth knowing.

Minor: it also does not clear `unstickForceTimer`, so an in-flight unstick push finishes.

### 3.9 Scene wiring noise

Cosmetic, no runtime effect, but misleading when reading the inspector:

- Weapon slots 2/3/4 (the three missiles, all `Instantiated` mode) each have a particle emitter assigned,
  which that mode ignores. **No longer purely cosmetic:** switching away from a slot now stops its emitters
  and rewrites their emission rate, so slot 2 will reach into the **Shotgun's** emitter, which is what it
  actually points at, on every switch. Harmless today only because the Shotgun is burst-only, but it is now
  a live coupling rather than dead data. Slots 3 and 4 additionally hold a null element in the list.
- Slot 5 (Chain Gun, `ParticleSystem` mode) has a muzzle point assigned, which that mode ignores.
- `blast.damageLayers` is `Everything` on `WD_ChainGun`, `WD_MachineGuns` and `WD_Shotgun`, none of which
  read the blast section.

### 3.10 Smooth ground normals depend on Read/Write Enabled, and fail SILENTLY without it

`ResolveSurfaceNormal` interpolates a mesh's vertex normals so the hover system reads the smooth surface the
player sees rather than the flat triangle underneath. It needs `Mesh.isReadable`, which means **Read/Write
Enabled ticked in the model's import settings.** All 24 collision meshes in `Prototype_Scene` currently have
it. Nothing enforces that.

**The failure mode is the problem, not the dependency.** A terrain asset imported without it does not error,
warn, or behave oddly in any visible way. It silently falls back to flat triangle normals **for that object
only**, and the craft goes back to being nudged a few degrees several times a second while crossing it. It
will read as "this one part of the arena feels bumpy", which is not a description that points anywhere near
an import setting.

**When this will bite:** arena blockout (6.2), when new geometry arrives from outside the RVP package. It is
filed here rather than Tier 0 because nothing is wrong today and there is no work to do until then.

Options if it becomes a nuisance rather than a one-off: log a one-time warning naming the offending mesh the
first time a non-readable one is hit, or add an editor validation pass over the scene's `MeshCollider`s.
**Deliberately not built yet**, because a warning that fires legitimately on primitive colliders would be
the fifth instrument in this project to cry wolf.

**If anyone is tempted to untick it to save memory:** Read/Write Enabled keeps a CPU-side copy of the mesh,
so it doubles that mesh's memory. This feature is why it must stay on.

### 3.11 The smooth-normal mesh cache is never cleared

`_meshNormalCache` is a `static Dictionary<Mesh, MeshNormalData>` filled lazily per mesh driven over. **The
cache itself is required, not an optimisation:** `Mesh.triangles` and `Mesh.normals` allocate a fresh array
on every access, so reading them per hover ray would generate garbage four times per `FixedUpdate` forever.

What is missing is eviction. It is never emptied and it holds strong references to `Mesh` objects.

- **In a build this is harmless.** The cache lives as long as the level does, roughly 80KB per large terrain
  mesh, and only for meshes actually driven on.
- **In the EDITOR it accumulates**, because statics survive exiting play mode, and the held references also
  stop those meshes being unloaded. Negligible with one scene; it grows once there are several.

**Fix when convenient:** clear it on play-mode state change in the editor, and consider
`RuntimeInitializeOnLoadMethod` for domain-reload-disabled setups. Left out deliberately for now, since
adding lifecycle code that cannot be exercised in a single-scene project is its own risk.

### 3.15 An EMP freeze silently eats a landed trick

`Energy.Grant` refuses to pay while EMP-frozen, matching `TryConsume` so a freeze cannot be worked around by
banking a trick during it. That is defensible and probably right, since EMP denies tempo. **It was never
DECIDED, and that decision is what remains open.**

The readout half is fixed: both the HUD and the gizmo used to report any shortfall as "pool full", because a
saturated pool was the only cause either knew about; a frozen pool produced the identical shortfall and got
the identical, wrong label. The two causes are now named separately.

**What is left is the design question, not the display.** A landed trick during a freeze still pays nothing,
and whether that is right has never been argued: the player did the work and the landing was clean.

**Not reachable in play today**, because the AI cannot fire an EMP (1.6), which is why this is not in Tier 0
despite being a trick item. **Done looks like:** a decision on whether a freeze should forfeit the trick,
cancel it, or merely defer the payout until the freeze lifts.

### 3.16 The trick system lives in scene overrides, not on the prefabs

`HoverController_Tricks` on the player craft, and `/Canvas/TrickTracker_TMP` plus its `trickText` wiring, are
all **instance overrides in `Prototype_Scene`** rather than changes to `HoverCar_Prototype.prefab` and the
Canvas prefab. It works, and it works in a build, because the build ships the scene.

**Why it was left this way:** applying prefab overrides is all-or-nothing per instance, and neither instance
has been audited for what else is sitting unapplied. Pushing blind would silently commit unrelated pending
changes into the prefabs.

**When it bites:** the moment a second scene exists, or a second player craft is instanced, or the prefab is
used as the base for the five-vehicle roster (5.7). **Done looks like:** audit the override list on both
instances, then apply deliberately.

Note the AI craft has no `HoverController_Tricks` at all, which is correct rather than an oversight:
`AIHoverInput` hardcodes drift to false so it can never arm air control and would bank nothing.

### 3.17 Solved camera OUTPUTS are being saved into the scene as prefab overrides

`Prototype_Scene.unity` carries instance overrides for the vcams' `FollowOffset.y`, `FollowOffset.z`
and `Lens.FieldOfView`. **None of those is a setting.** They are the framing solver's per-frame
output: `CommitDrive` and `CommitStrafe` write them every `LateUpdate`, and in edit mode those writes
persist, so whatever pose the camera happened to be solving gets serialised into the scene.

Found 2026-08-17 during the 0.14 retune, and it had already caused real confusion in that session:
one of the saved values was `FollowOffset.z: -11` from a candidate framing that was tried and
abandoned, still sitting in the scene shadowing the prefab.

**Why it will be misdiagnosed:** the authored inputs live on `HoverCameraController`
(`framing.driveFollowOffset`, `strafe.followOffset`, `framing.baseFov`, `strafe.fovReduction`). The
vcam fields are downstream of those. So changing an authored value and seeing nothing happen looks
like the tuning field being dead, when in fact the scene is holding a stale copy of the output. It
will read as "the camera ignores its own settings".

**This is the known cost of an existing decision, not a new defect.** `CLAUDE.md` already records
that edit-mode writes are skipped when unchanged precisely because an unconditional per-tick write
leaves the scene permanently dirty. The skip reduces the churn; it does not stop a genuinely changed
value from being saved.

**Done looks like:** the overrides cleared, and a decision about whether to prevent them recurring.
Options, cheapest first: revert those specific overrides by hand whenever they appear; stop the
edit-mode commit writing `FollowOffset` and `Lens` at all and let the preview drive a runtime-only
copy; or mark the vcam fields so they are never serialised from a preview write. **Do not solve it by
disabling the preview**, which is the feature that makes the framing judgeable without driving.

Same family as 3.16 (scene overrides shadowing a prefab) but a different cause: 3.16 is authored work
that never got applied, this is machine-written output that should never have been saved.

The vcam's `m_LocalPosition` moves around in scene diffs for the same reason, that one being
Cinemachine writing the transform rather than the framing solver. **Owner 2026-08-18: not
noteworthy.** It is inert (overwritten on the first frame, and `FollowOffset` is the real authored
input) and it is covered by a **bake-down pass over prefab and scene state scheduled for the
pre-alpha 1 approval**, which is where this whole item gets settled.

---

## Tier 4 — Verification debt and instrumentation

**Nothing in this tier blocks play, the playtest, or continued development.** It is what is believed but
not proven, plus instruments that are missing or lying. Work it when a question actually needs answering.

### 4.1 There are no tests, at all

No asmdef, no test assembly, no EditMode or PlayMode tests anywhere in `Assets/` outside the RVP
third-party folder. Every regression so far has been caught by hand-driven measurement, which is why
several bugs survived multiple sessions.

Not urgent, but the projectile helpers (`ProjectileSweep`, `WeaponImpact`, `TargetingScan`, `HoverMath`)
are pure static functions and are the cheapest possible place to start.

### 4.2 No instrument records the live caps or which force is acting

`FrameSpikeWatch` and `MotionTrace` between them cover frame death, allocation and whether the drawn
frame matched the physics. **Neither records the live forward and lateral speed CAPS, or which force
branch acted**, which is exactly what would let the boost ramp and the drift bleed be graphed against
each other.

Propulsion already captures both for its gizmo (`_dbgFwdCap`, `_dbgLatCap`, `_dbgDrive`, `_dbgDrag`,
`_dbgBleed`), so **closing this is mostly a matter of exposing those rather than computing anything
new.**

The jump arc no longer needs this; it is measured well enough by the impulse table in `TuningLog.md`.

### 4.3 Particle weapon delivery is unverified end to end

Missile, EMP and HardLock detonation and knockback are verified in play mode. **The Shotgun, Machine Gun
and Chain Gun particle paths are not**, and 1.3 means damage is zero on most of them anyway, so only
knockback could be exercised today.

Related and still unmeasured: **particle collision at scale.** Every multi-vehicle performance figure
was taken with AI driving and no combat, because the AI cannot fire (1.6).

### 4.4 Allocation under sustained weapon fire has never been measured

**Narrowed 2026-08-20: the marker half closed, the allocation half did not.** Retitled rather than
renumbered, because what remains is the half the number was issued for. The GC question itself was
already closed: the periodic hitch was the editor's own allocation and does not
survive a build. What that three-minute run did not cover:

**Allocation under sustained weapon fire and heavy projectile traffic.** Never measured. `AllocationBisect`
requires a parked craft and every run since has been driving, so these paths have no allocation figure at
all. **The build makes this cheap:** idle allocation there is 0.01-0.08 MB/s, so anything the weapons
allocate will stand out against a floor of essentially zero, which was never true in the editor's
4.47 MB/s. Fire continuously through a long run and read `alloc_mb_s` in the spike CSV.

**Three findings from the 2026-08-20 performance sweep land here, and none of them is a new item.**
Full write-up in `TuningLog.md` > The movement and camera performance sweep.

- **`MotionTrace` preallocates about 139 MB** (870,000 samples x 168 bytes, from `captureSeconds`
  3000) and it is enabled in the scene and auto-captures after warmup, so it ships in builds. **This
  is an owner decision, not a code change**, because `captureSeconds` is an inspector value: dropping
  it to a realistic session length returns most of that memory. Note the interaction with the marker
  history above -- the buffer HAS filled in a real session, at t=666.9s, so the value cannot simply be
  cut to the shortest plausible run either.
- **`MotionTrace.FixedUpdate` was missing the `_full` guard its three sibling paths have**, so a
  filled buffer kept paying for a whole `FillShared` a hundred times a second while recording nothing.
  Fixed 2026-08-20. **Worth knowing that this was live during the 2026-08-18 session**, which filled
  at t=666.9s and then ran on with the leak for the remainder.
- **The chassis uses Continuous collision detection at 1000kg**, flagged and deliberately not changed.
  At 100Hz it moves about a metre per tick against a roughly 7m hull, so Discrete would probably be
  safe and cheaper -- but that is a physics-correctness call on a load-bearing pillar with no
  measurement behind it. **Only worth opening with a reason and a before/after.**

**The idle allocation figure this item wants cannot be gathered over MCP.** Readings during the sweep
ranged 0.48 to 2.58 MB/s and `eval` traffic pollutes all of them; see `Measuring.md` traps 48 and 49.
A clean run needs a focused editor or a build with nobody driving it remotely.

**The marker key is CONFIRMED WORKING in a build, 2026-08-20**, on the owner's report from the
four-tester session. It had never been proven before: the first build run recorded zero markers because
nobody pressed `M`, and the 2026-08-18 run reported `0 markers` despite the owner spamming `M` for
about a minute during a soft lock, **which was the buffer having filled at t=666.9s and not the key**.
That is now settled in the other direction and needs no further runs.

**What replaced it as the marker risk, and it is a process one rather than a code one.** The
2026-08-19 session dropped no markers at all, by choice: the owner watched every run and diagnosed the
soft locks live, which worked at four testers with one observer. **It does not scale, and the traces
from that session are correspondingly hard to navigate.** Anyone tested without the owner watching has
to be told to press `M`.

**The real finding from that run is the buffer, and it changed a shipped value.** `captureSeconds` was
600 against sessions running 20 minutes, so capture ended at 11.1 minutes and the entire soft lock,
which is the whole reason anyone wanted the trace, fell outside it. Worse, a chunk of the buffer was
spent recording a parked craft while the game idled on the `PlaytestSession` splash between testers:
the instrument does not know the session is not running and bills idle time at the full frame rate.
**Raised to 3000 on 2026-08-18** (870,000 rows, 144 bytes each, measured 119.5 MB preallocated, about
55 minutes at the observed 265 rows/sec), which covers a four-tester block in one launch. **The
general trap: a bounded capture sized against an intended session length silently truncates the
moment the session runs long, and it fails in the direction that loses the interesting end.**

### 4.5 `FrameSpikeWatch`'s throttling verdict compares two samples of a noisy signal

The verdict compares the **first** benchmark sample against the **last** one. That is only valid if a
single sample is a good estimate of the machine's speed, and it is not: across 91 heartbeats the
benchmark has **sd 0.093ms on a mean of 0.532ms, 17.4%**, ranging 0.366 to 0.655. Two samples drawn from
that spread can differ by 79% with nothing happening at all.

**It has fired twice and been wrong both times, and cost time both times.**

- First firing printed `THROTTLING LIKELY ... +37.1%` for a session that is flat when binned into
  30-second windows (first half against second half is +1.7%). It was believed and written into another
  item before anyone checked it.
- Second firing printed `-16.8%` for a session whose binned benchmark is flat to within 8%, during a
  genuine performance investigation. It pointed at the CPU, which was one of the few things measurably
  fine.

**This is trap 15 wearing different clothes, and worse:** `FrameSpikeWatch` exists BECAUSE a fixed
threshold against a noisy quantity cries wolf, and its baseline is a smoothed running average for exactly
that reason. The benchmark got a naive endpoint comparison bolted onto the same instrument built to avoid
one.

**Done looks like:** compare the mean of the first N beats against the mean of the last N (N around 10,
roughly 20 seconds), and stay silent unless the difference clears the measured noise, something like 2
standard deviations rather than an arbitrary percentage. Printing the binned series instead of a verdict
would also be honest, and is what actually answered the question both times.

**Until then, ignore the verdict line and bin `bench_ms` from the CSV by hand.** A wrong confident answer
is worse than no answer, which is the premise of this whole file.

---

## Tier 5 — Design decisions with no engineering blocker

**These need an owner call, not code.** Listed with what is already measured so the decision is cheap.

### 5.1 Hard Lock is not designed through

**The mechanism works and is verified.** What follows is about what the weapon SHOULD be, not whether it
functions.

Stated intent: a volley of **small, low-damage, low-impact missiles**, lockable onto a single target or
distributed across several. Currently implemented as the degenerate case, one lock and one missile, and
tuned as though it were a Rocket Launcher.

Current values are the wrong shape for that intent: `impactForce` and `splashImpactForce` are identical to
the Dumbfire, on a weapon whose identity is that it cannot miss. `turnRate` is 120, below the 160 measured
as the threshold for landing a clean hit. `lockConeAngle` is 45 degrees at `lockRange` 115: wide for a
single lock, but plausibly right for sweeping across several targets in a cascade.

The lock now commits to one target and holds it, which is the primitive a multi-target version needs.
Extending it means promoting `LockTarget` to a list and looping `Scanning` back on itself with
already-committed targets excluded from the scan; the validity rule and the release-to-fire trigger carry
over unchanged.

**Do not tune this weapon until the design is settled.** Any number picked now is for a different weapon.

### 5.2 The knock-around pass is only partly applied

The missile force cut sits on Soft Homing, not Dumbfire, by owner decision. `WD_Missile` (Dumbfire) and
`WD_HardLockMissile` both keep the original values.

That is GDD-aligned: soft homing is specced as low damage and force, the Rocket Launcher as the primary
momentum-disruption tool. But it does mean **the Dumbfire keeps the over-scaled value the audit flagged**,
which measured a delta-v larger than the craft's own top speed, and where a flank hit fully inverted a
stationary craft and threw it about 80m.

**The open question is only whether the Dumbfire keeps that value.** The split itself is verified and working
as designed.

### 5.3 `destabilizeFraction` has never been tested against a landing hit

Held at 0.15 across all three missiles. It was set **before detonation worked**, so its rationale was never
verifiable: every apparent hit at the time was really the lifetime fuse firing splash-only. Detonation now
works, so this is measurable for the first time.

**New coupling:** it also governs self-inflicted spin from a rocket jump, and **self-flips become reachable
past roughly 0.36.** Recheck that before raising this value.

### 5.4 `selfImpactScale` default is a guess

Set at 0.5 on all six weapons, sized against existing mobility so a rocket jump lands between a tap jump and
a charged one. **Unmeasured in play.**

Note the self-boost **scales with launch speed**, because the rocket does not inherit vehicle velocity, so at
speed you close on your own blast. That reads as skill (shooting a wall you are charging kicks harder) but it
means the mechanic is strongest exactly when you are already fastest. If you would rather it were flat,
measure falloff from the launch position instead of the detonation point.

### 5.5 Homing cannot catch a runner, and the speed pass turned that into a defect

**Read from the assets 2026-08-16: all three missiles are at `speed: 70`, against a `topSpeed` of 80 and a
boosted 100.**

So a craft running flat out in a straight line is **not being closed on at all; it is pulling away**, by 10
m/s unboosted and 30 boosted. Previously missiles at 70 against a top speed of 60 closed at 10 m/s, which was
slow but finite, and that is the situation the original framing was written for. **No `turnRate` can fix a
negative closing speed.**

The original conclusion still holds as intent (homing weapons punish players who are turning, fighting or
cornered rather than running) but it is no longer a choice, it is the only thing the numbers permit.

**Decide which:** raise missile `speed` above the new boosted ceiling so homing can threaten a runner, or
accept the escape and keep homing as a close-quarters harassment tool. Either way **the value to change is
`speed`, not `turnRate`.** Nothing flagged this when top speed moved, which is the same
neighbouring-system-invalidated-by-a-shared-quantity shape as the air jump.

### 5.7 Vehicle roster is one profile

`VTP_Default` is the only `VehicleTuningProfile` in the project. The GDD proposes a five-vehicle framework on
two independent axes (momentum character, stability vs agility): Bruiser, Interceptor, Chaos Vehicle,
Precision Drone, Transition Specialist. None of the other four exists.

The architecture supports it already, one asset per archetype with scene refs staying on the MonoBehaviours,
so **this is authoring work, not engineering.**

### 5.8 Vehicle scale: deferred, with two gates

The craft is a ~1.6x sedan and disproportionately wide, which is what makes it read stubby rather than simply
large. Measured dimensions and the full reasoning are in `CLAUDE.md` > Vehicle Scale.

**Deferred deliberately, not forgotten.** The handling is good and there is nothing built for the craft to
look wrong against. Two points where the decision should be re-opened, because both produce work that
INHERITS the scale and is expensive to redo:

1. **Before arena blockout.** Geometry authored at one scale and rebuilt at another is far more costly than
   rescaling a single vehicle. Note the alternative is equally valid and cheaper: **build the arena TO the
   craft** (corridors ~20m, entrances ~10m) and nothing needs retuning at all.
2. **Before the five-vehicle roster** (5.7). Authoring five handling profiles against an unsettled scale means
   tuning five vehicles twice.

If it is ever revisited, the cheapest single improvement is **narrowing the hull.** Width is the least coupled
dimension to the tuning, since the attitude torques ignore the inertia tensor.

### 5.11 Boosted strafe reaches a large fraction of drive top speed

**DEFERRED to PRE-ALPHA 2, and the deferral is principled rather than a punt:** the open question is whether
players CAMP in strafe under combat pressure, and there is no combat to apply pressure until weapons deal
damage (1.3). It cannot be answered before then, which is why it is not in Tier 0.

**Improved for free by the speed pass, from 100% to 83%.** Boosted omnidirectional strafe now sits at 66.3 m/s
against a drive top speed of 80, verified from the live caps. It is no longer true that strafe matches
drive-mode top speed, which was the sharpest form of the complaint.

**THE RATIO ITSELF IS NOW JUDGED AND CLOSED.** Owner, 2026-08-16, on the 83%: *"I think that's
relatively okay considering you only get 5 seconds boost and then have to wait or hit tricks to get
that back."* Confirmed from the assets: `maxEnergy` 100 / `boostEnergyPerSecond` 20 = **exactly 5.0s**
of continuous boost. Two facts that follow and should stop this being re-derived:

- **Strafe against drive is 66.25% both boosted and unboosted, and there is only ONE ratio.**
  `StrafeTopSpeedScaled` is `strafeTopSpeed * (effectiveTopSpeed / topSpeed)`, so boost scales both
  ceilings by the same factor. **The two cannot be tuned apart without changing that expression.**
- The 83% figure is boosted strafe against UNBOOSTED drive, which is the camping comparison rather
  than the mode-parity one. Both are correct; they answer different questions.

**What remains open is only the camping behaviour**, which still needs combat pressure to observe.

The owner's stated goal is that drive mode is for fast forward and backward travel and strafe is for aiming at
a reduced, consistent omnidirectional speed, and specifically that players should not "play the whole game in
strafe mode".

**One mechanic that materially limits the camping case and was not accounted for when this was written:
continuous boost requires forward-dominant throttle**, so pressing boost with pure lateral stick fires a DODGE
instead. **A player cannot boost straight sideways at all.**

**The lever if camping does happen:** stop boost scaling the strafe ceiling. Note that alone would recreate the
paying-for-nothing bug already fixed for reverse boost (energy drained for acceleration into an unchanged
cap), so it would need boost in strafe to become dodge-only, which is a larger behaviour change than it first
appears.

### 5.12 Revisit the aim framing once weapons can actually be judged

**DEFERRED to PRE-ALPHA 2 by the owner 2026-08-17**, on the same principled grounds as 5.11 rather
than as a punt. Succeeds 0.14, which shipped and closed; **the number is new because 0.14's job was
the pre-alpha 1 retune and that is done.** Shipped values and the full reasoning are in
`TuningLog.md` > Drive and strafe framing.

**Why it cannot be settled now.** The strafe camera exists so the player can aim, and the shipped
framing was judged against a reticle with almost nothing to shoot: four of six weapons deal zero
damage (1.3), and there is no combat pressure to frame against (1.6, 5.11). **A camera tuned for
aiming, judged without aiming, is tuned against the wrong thing.** The right moment is when weapons
and weapon physics are far enough along that engagement range, target size on screen and time to
react are real quantities.

**What to re-judge specifically, all measured and recorded so the comparison is cheap:**

- **The hull occupies 69.6% of screen height in strafe.** Improving that is the biggest available
  win and every candidate that achieved it cost something else.
- **Target size against hull size.** A craft at 60m subtends 9.6% of screen height. That number
  becomes judgeable only once you are shooting at one.
- **The crosshair-to-roof gap, 183px.** Enough that the reticle reads as out in the world rather
  than stuck to the cab, which was the failure of one candidate.
- **`reticleProjectionDistance` 200.** Nearly inert at the shipped framing, and it regains authority
  if the camera gets flatter.

**Read `TuningLog.md` before touching any of it.** Two candidate framings were built and rejected in
the 2026-08-17 session, one of which reproduced the Vertigo effect exactly, and the trade curve
between crosshair clearance, horizon height and hull size is recorded there. Re-deriving it is a
half-day.

---

### 5.13 Re-judge the aim feel now that nothing is propping it up

*Was `0.31`. **Retired from Tier 0 on 2026-08-20** and renumbered. Owner: the aim feel is good enough
for now, defer until weapons are further along. **Tier 5 rather than Tier 2** because this item was
always tuning that closes on a feel report rather than code, which is this tier's definition, and
because it now shares 5.12's gate: judging how the nose answers the stick is worth more when there is
something at the end of the barrel. **Read alongside 5.12** — that one is the aim FRAMING, this one is
the aim FEEL, and they will want the same session.*


**Successor to 0.29, which closed 2026-08-19.** New number rather than a reopen, because 0.29's job
was the defect and that is done and judged. **Nothing here is a known fault.** It is a short list of
values whose MEANING changed when the un-aim bug was fixed, and which have therefore never been
judged for what they now do.

**1. `aimPitchTrackingStrength`, live value 200.** Read `TuningLog.md` > Aiming on slopes before
touching it. **It was never a feel value:** it was the number holding the nose down against a
suspension that had stopped working, which is why aim tracking needed to be seventeen times leveling
strength. With the bug gone it barely affects attitude at all — 200, 100, 50, 25 and 12 all hold a 20
degree ramp within 8 degrees at support 1.000 — so it is now free to be tuned purely for how the nose
answers the stick, which is what its tooltip always claimed. **The owner tried 100 during the
2026-08-19 session and liked it, but the change did not persist to the asset**, so that reading is
unconfirmed and the shipped value is still 200.

**2. `strafePitchLimit`, live value 20, and its meaning changed.** With the hull levelling to the
horizon in aim mode, the clamp is now measured from the HORIZON instead of from the surface, with no
change to the clamp code. **A ramp no longer adds to your reach**, so on a slope steeper than 20
degrees you cannot point at something directly ahead of you on it, and the arena has the loop and the
screwtower. Raising it is cheap: 0.22 removed both things that made a large limit dangerous. **Not
reported as a problem yet** — the owner has driven several sessions since without raising it — so
this is a watch, not a fix.

**3. The strafe composer's dead zone and damping, `0.2` and `0.5` on the vcam.** Camera rotation lags
any hull pitch change, including commanded ones, at roughly 25px per degree at strafe FOV 57. **Do
not pre-empt this.** It predates everything in 0.29 and the owner has judged the current feel good
twice since; open it only if a play report points at it. Read the values off the vcam in the editor
rather than the prefab, per 3.16 and 3.17.

**Done looks like:** the owner has driven each of the three and either changed it deliberately or
recorded that it is right as it stands. **This item is tuning, not engineering**, and it closes on a
feel report rather than a measurement.

**Related but NOT in scope: the nose overshoots its commanded aim by about ten degrees across a
landing.** That is recorded as a consciously accepted limitation in `CLAUDE.md`, with the owner's
verdict that it reads as a natural consequence of a high aimed jump. **If it is ever reopened, it is
the pitch overshoot that gets fixed, not the camera**, which only moves because the strafe rig
inherits hull pitch by design.


### 5.14 The air jump does not justify itself

**Pre-alpha 1 playtest, 2026-08-19, all four testers.** The consensus: they could take or leave the
air jump. It did not hurt anything and it did not add enough to justify its presence.

**This is a VALUE report and NOT a feel report, and the distinction is the whole item.** Air jump feel
was judged good on 2026-08-14 (*"air jump feels good now"*) after `airJumpFallCancel` 0.6 fixed the
real defect, which was that the ability's strength depended on WHEN it was pressed. Nothing in this
report contradicts that. **Four players said the ability works and does not earn its place**, which is
a different question and does not reopen `TuningLog.md` > Air jump.

**Three things constrain any answer, and two of them are load-bearing elsewhere:**

1. **Wall jumping falls out of the air jump firing along local up** and is confirmed working in play
   (`CLAUDE.md` > Standing Decisions, closed as `5.10`). Removing or re-scoping the air jump removes
   wall jumping with it, and wall jumping was accepted as advanced movement.
2. **Cosine loss is intended.** Vertical gain scales with how upright the craft is, which is the direct
   consequence of the directional-juke decision and cannot be removed without giving up the wall-shove.
3. **It costs 15 energy** against a tap jump's 10 and a charge jump's 25, inside an economy that was
   rebalanced when `regenRate` was halved to 10 and tricks became an income.

**No action is being taken.** Owner, 2026-08-20: this sits as feedback. It is filed rather than
dropped because "works and is not worth it" is an open design question, and this tier is where those
live. **Do not act on it as a tuning problem** — making the air jump stronger is not what four people
asked for.

### 5.15 How much stacks on top of a jump, and whether it is too much at once

**Pre-alpha 1 playtest, 2026-08-19, one tester.** Tester 3 struggled to manage boost, jump, air
authority and tricks all building off each other, and was often boosting through tricks or dropping
air authority part way through a rotation.

**The discoverability half of this is 2.19 and is answered.** Once told how air control works, every
tester was landing tricks in their second run. **What is left here is the question 2.19 does not
cover: whether the abilities layering onto one jump is too much to hold at once even when understood.**

**One concrete ergonomic collision belongs to 2.18, not here:** `Boost` is L3, a click of the same left
stick whose deflection air control reads for pitch and roll. Some of "boosting through tricks" is that
one input doing two jobs, and it gets solved when the control scheme is finalised.

**Why this is a real question rather than a first-session complaint.** Everything stacked on a jump was
judged good individually — the trick economy (2026-08-15), boost in drive mode (2026-08-16), fall
gravity and the two-barrel-roll margin (2026-08-17), the charge squat (2026-08-17). **Each was judged
alone. Nobody has judged the combination**, and the project has already recorded the general lesson
that neighbouring systems need re-judging when a shared quantity moves (`TuningLog.md` > Air jump).

**No action is being taken.** Owner, 2026-08-20: this sits as feedback. **Do not simplify anything on
the strength of one first-run report** — the same session recorded an approachable learning curve and
real high-skill headroom, judged good in `CLAUDE.md`, and that is the thing this tier must not trade
away by accident.


---

## Tier 6 — Unimplemented features

### 6.1 Seven of thirteen weapons

Implemented: Machine Gun, Minigun (Chain Gun), Shotgun, Rocket Launcher, Soft Homing, Hard Lock.

Planned, no asset and no code: Sniper / Lightning Bolt, Laser Cannon, Gravity Well / Repulsor, Bouncing Disc
Blade, Floating Proximity Mine, Directional Remote Mine, and the per-vehicle Special.

Note `WeaponType.Mine` and `ProjectileMode` already exist and `TickMine` is implemented (fires at the muzzle
with no launch velocity), so **the two mine weapons are closer than the rest.**

### 6.2 Arena

The GDD has a full arena philosophy (track with rooms, three vertical layers, parallel routes at every major
node, corridors as momentum builders, falling as a tactic) and prototype scope calls for a small test arena
with ramps and obstacles. `Prototype_Scene.unity` is the only project scene.

**Measured constraint to design against: tunnel and doorway entrances want about 10m of clearance.** Interiors
are safe because the ceiling duck squats the craft; entrances are not.

**The arena is deliberately SPARSE.** Owner: "I hadn't really imagined an arena to be dense with props, maybe
just some environmental hazards and exploding barrels or something, but by no means full of objects on the
ground."

**So props are NOT coming to fix speed readability, and this item is not the blocker for it.** Ride height has
since been judged good independently, so only the speed half remains open, and what is left to read speed
against is ground texture density, road markings, the height and spacing of the large structures, and
screen-space effects such as 2.11's speed lines.

Two constraints any blockout must respect: the owner wants **platforms you can bump from underneath, not
spaces to duck under**, so the ceiling duck is not a design dependency; and the existing figure-eight loop is
the reference for drift curvature.

Note the environment was doubled to match the 2x vehicle mesh, which halved how fast the world goes past you
at a given speed.

### 6.3 No match flow

No menu, no match start or end, no scoring, no round structure. Combined with 1.1, there is currently no way
for a match to conclude.

### 6.4 Campaign layer

Per-vehicle pilot stories with intro and outro cinematics, Twisted Metal style. Explicitly a full product
layer rather than a test harness. Nothing exists.

### 6.5 Multiplayer

Explicitly a future phase. The solo game is specced to stand alone.

### 6.6 GDD diagrams

Section 15 lists two planned diagrams, Control Flow and Interaction Map, to be exported to `/Docs/`. Neither
exists, and neither does the folder.

---

## Renumbered 2026-08-16

**Tier M was dissolved** and the tiers re-sorted so that Tier 0 is movement and camera, which is what gets
worked next. **Old numbers are dead and are not reused.** `ROADMAP.md` was updated in the same pass.

| Was | Now | Why it moved |
|---|---|---|
| M.3 | **0.12** | Camera bug. Top tier |
| M.11 | **0.13** | Camera. **Narrowed**: the trick-camera half is closed, see the item |
| M.13 | **0.14** | Camera. Top tier |
| 3.13 | **0.15** | Camera, unblocked, and it gates two feel items |
| 2.1 | **0.16** | Jump is movement. **Since retired to 2.15**, effectively back where it started: the cue is movement, but what gates it is the ability scoping, and that is the tier that matters |
| 2.6 | **0.17** | Movement and camera feel (had already absorbed 0.9) |
| 5.6 | **0.18** | Movement. Already gated PRE-ALPHA 1 |
| 3.14 | **0.19** | Camera tooling |
| M.12 | **2.11** | Movement and camera, but **blocked on VFX**, so it defers |
| 0.3 | **4.1** | Verification debt. Blocks nothing |
| 0.5 | **4.2** | Instrumentation gap |
| 0.6 | **4.3** | Verification debt |
| 0.7 | **4.4** | Build checks |
| 0.11 | **4.5** | A lying instrument |

## Retired numbers

Closed and removed. Listed so an old note or commit message still leads somewhere, and so numbers are never
reused.

| Numbers | Where the outcome lives |
|---|---|
| M.1, M.2, M.4, M.14, 2.3, 2.8, 5.9, and all of the old Tier 4 | `CLAUDE.md` (standing decisions, module constraints) |
| M.5, M.6, M.7, M.8, M.9, M.10, 0.8, 0.10, 2.10 | `TuningLog.md` (measured tables and rejected attempts) |
| 0.1, 0.2, 3.4 | Done, nothing worth keeping |
| **3.18** | `GameDesignDocument.md` section 6. **Opened and closed the same day, 2026-08-18.** The GDD's control table and `HoverControls.inputactions` disagreed about steering, the table claiming yaw sat on the left stick. Raised as an owner question because the two readings pointed opposite ways: a rotted doc, or a real control-scheme regression that fifteen judged-good movement behaviours had been judged against. **Owner: the doc is wrong and the in-engine scheme is right, simple as that.** The table was corrected against the asset, which also fixed drift (L1 *or* R1), shield and EMP (both d-pad), and removed a keyboard column describing bindings that do not exist. The table now names the asset as its source of truth, the way tuning values already name `Assets/Data` |
| 0.9 | Merged into 0.17, which owned the whole boost judgement until it closed |
| **0.15** | `CLAUDE.md` > `HoverCameraImpulseRouter`. **Never a defect:** the three nulled channels are a deliberate playtest scoping decision, corrected by the owner 2026-08-16 |
| **0.17** | `CLAUDE.md` > Judged good in the 2026-08-16 playtest. Boost drive mode judged good; its one residual became **0.20** |
| **0.22** | `TuningLog.md` > Aiming and the hover sensors. Shipped 2026-08-17. Opened and closed inside one session, so it never appeared in this file as open work |
| **0.14** | `TuningLog.md` > Drive and strafe framing. Shipped 2026-08-17 for pre-alpha 1. **The pre-alpha 2 revisit is 5.12**, a new number rather than this one reopened |
| **0.23** | `TuningLog.md` > The drift hop was being suppressed. **Never a missing cue:** the hop was being silently skipped about one entry in three, and no cue was built. The VFX half is 2.12 |
| **0.20** | `TuningLog.md` > The boost jolt at a standstill. Shipped 2026-08-17. **The mechanism this file had recorded was wrong:** the speed gate was never the cause, and three runs showed the culprit was the look-ahead being dragged through its whole swing at boost acceleration. See `Measuring.md` trap 42 |
| **0.26** | `TuningLog.md` > The boost gate was a step against reversing. Shipped and **judged good 2026-08-17** by playing exactly the manoeuvre that produced it. Opened and closed inside one session. **Its engage cost was accepted rather than merely tolerated** — see `CLAUDE.md` > Judged good — so do not "restore" the sharper ramp |
| **0.16** | **2.15**, which carries the shipped mechanism and the tuning history forward. **Retired without ever being judged**, 2026-08-17. It had been held open by one question — whether the denied-jump channel was disabled along with EMP and weapon recoil by accident, being a movement cue rather than a weapon one. **The owner confirmed it is intentional** and that it will not be evaluated before the pre-alpha 1 playtest, which makes the gate energy-and-ability scoping rather than anything about movement, so the tier changed with it |
| **0.24** | **2.14**, which carries the measurements forward. **Retired without anything being built**, 2026-08-17: the thrusters it targets are placeholder and are being replaced, so both the tier and the timing changed. Retired rather than moved because numbers encode their tier and are never reused |
| **0.13** | `TuningLog.md` > Camera control while downed, and > The downed window for the measurement that sized it. Shipped and judged good 2026-08-17. **Two of this item's three shapes were never built and were not rejected** -- a downed framing and "something else" both remain available if a play report ever asks for them. **The handback was closed on the owner's play evidence over the assistant's measurements:** the 19 m/s "fling" was vertical, the craft returning to ride height, and being downed is rare on this chassis |
| **0.19** | `CLAUDE.md` > `HoverCameraController.cs`. Shipped 2026-08-17. Added `ReverseBoost`, `NoseAcrossTravel` and `Downed`, and replaced the hard-pinned `travelSpeed = forwardSpeed` with a null-defaulted override so disagreement must be stated rather than being impossible. **The gap was never that a state was missing, it was that the data model could not express one:** every field existed, but the preview pinned the two speeds equal, so `0.20` and `0.26` were both invisible in the inspector and each cost a driven playtest |
| **0.18** | `TuningLog.md` > The downed window > Closed as 0.18. Shipped 2026-08-17. **The title of this item was the wrong diagnosis:** `flipRecoverySpeedThreshold` was never the defect and stayed at 2. The arming clock was RESETTING to zero on any excursion past it, pricing a one-frame blip and a 229ms one identically, and the craft is pushed out of the gate by its own settling tumble -- so recovery interrupted its own clock. `flipRecoveryProgressDecay` 1 makes the cost symmetric. Worth 0.78s on a 25 m/s wipeout and 2ms at rest |
| **0.21** | `TuningLog.md` > Fall gravity and airtime, in the 2026-08-17 continuation rather than a new entry. Shipped `extraFallGravity` 30 -> 35. **Closed without a sweep**, because the closed-form model reproduced the 2026-08-14 table to within measurement slop on three independent quantities, so a sweep would have re-measured arithmetic. **The hard-landing framing this file carried was the wrong headline:** both of the owner's rules survive the whole usable range and the real cost is the barrel-roll landing margin, which has no ceiling to trip — it degrades continuously, ~25ms per 5 units. **Judged good in play the same day**, against a test the owner supplied: two barrel rolls still land |
| **0.25** | `TuningLog.md` > The charge squat. Shipped 2026-08-17. Ride height is now the charge meter, `chargeSquatDepth` 0.2 of hover height. **The owner answered this item's blocking design question: the squat EXPRESSES the charge and never stores it**, so nothing here stacks on `jumpImpulseMax` and the tuned arc keeps its meaning. Two of the three traps this file recorded were real and one was overstated. **Trap 1 was real and this file understated WHERE it bites:** support does not collapse on the slow charge ramp (worst 0.85, a steady 15% sag), it collapses to exactly 0 for 0.2s when the target STEPS -- landing with a charge already held, or ducking into a tunnel at speed. **The latent ceiling-duck bug this file predicted was therefore confirmed and is fixed as part of this**, by keying support to the authored ride height instead of the targeted one. Trap 2 handled by min-combining two ride-height writers; trap 3 is why depth is a fraction rather than a distance |
| **0.29** | `TuningLog.md` > Aiming on slopes: the un-aim reference, and the horizon. Shipped and **judged good in play 2026-08-19**: *"This genuinely feels much better."* Two things, and **the bug was the larger half**: the owner's design decision that aim mode stabilises the BASE rather than the gun (`CLAUDE.md` > Standing Decisions), and a real defect where `UnaimRotation` referenced a `AverageGroundNormal` that `ApplyHoverForces` had just reset to `Vector3.up`, so the un-aim was measured against the world vertical instead of the surface. **Invisible on flat ground, which is where 0.22 was measured**, and on a 20 degree ramp it unloaded the rear springs to exactly zero and left the craft hanging off its front pair. It also closed a hypothesis 0.22 had recorded and marked unmeasured. **`aimPitchTrackingStrength` 200 was never a feel value** and the successor for re-judging it is **5.13** (issued as 0.31, retired to Tier 5 on 2026-08-20) |
| **0.30** | `TuningLog.md` > The landing snap in strafe was the damping coefficient, not the damping. Shipped and **judged good 2026-08-19**: *"it feels good."* The stutter was neither the springs nor the craft, whose motion is continuous across touchdown; the strafe camera's vertical damping collapsed from 0.25 to 0 in the ~20ms hover support takes to arrive, snapping out two metres of accumulated lag in two frames. Fixed with a slew limit on the coefficient, `verticalDampingSlew` 0.6, the same instrument as the speed look-ahead's. **The residual dip on high aimed landings was ACCEPTED by the owner rather than fixed** and is recorded in `CLAUDE.md` > Consciously accepted limitations, along with the note that the thing to fix there would be the pitch overshoot, not the camera |
| **5.10** | `CLAUDE.md` > Standing Decisions. **Wall jumping already works** and was never built; it falls out of the air jump firing along local up. Confirmed in play by the owner 2026-08-17 and accepted as advanced movement |
| **0.28** | **2.16**, which carries the whole analysis forward unchanged. **Retired from Tier 0 on 2026-08-20 without anything being built**, because the owner gated it behind a representative arena and an arena is art, which is the sorting rule's own demotion case. **Two findings travelled with it and both argue for the gate:** no tester raised altitude legibility at all across four players and two runs each, against an item that was explicitly filed on the prediction that they would; and the owner's original complaint named the tiled ground and the propless test environment as the cause, so the cue would have been tuned against conditions that will not exist |
| **0.31** | **5.13**, which carries the three values and their measurements forward unchanged. **Retired from Tier 0 on 2026-08-20 without being judged.** Owner: the aim feel is good enough for now, defer until weapons are further along. Tier 5 rather than Tier 2 because it was always a feel report rather than code, and it now shares 5.12's gate |
| **0.27** | `TuningLog.md` > Closed as 0.27, for the ablation and the restore sweep; `CLAUDE.md` > Modules > Foundation and > Standing Decisions for the rules it leaves behind; `CLAUDE.md` > Judged good 2026-08-20 for the acceptance table. Shipped and **judged good in play the same day it was taken**, the owner driving both repro cases: unable to get stuck in either. **This item's own framing was right about the mechanisms and wrong about the fix.** It recommended a dead-man timer on the grounds that it "does not care which gate failed", which is true and is exactly why the obvious implementation fails: forcing authority alone does nothing against a wall, where authority was already held and losing, and suppressing the springs alone does nothing when carried, where righting never armed. **Both halves are required and that was only found by running the half-fix and watching it not work.** Two further corrections worth carrying: the leveling torque is not a contributor and was ablated out, and the craft has never been flung by a recovery, a claim that has now cost two sessions and is filed under `CLAUDE.md` > Disproved by measurement with `Measuring.md` trap 45 |
| **0.34, 0.35, 0.37** | **2.18, 2.17 and 2.19 respectively.** Issued during the pre-alpha 1 playtest intake on 2026-08-19, never written into this file at Tier 0, and **placed straight into Tier 2 on 2026-08-20** by the same sorting rule: 0.35 is gated on the arena, 0.34 and 0.37 on the control scheme being complete. **Listed here anyway so the three numbers are burned**, because they were discussed by number before they were filed and a number that has ever named a thing is never reused |
| **0.33**, **0.32**, **0.36** | `TuningLog.md` > The speed pass, for the derivation and the rejected alternative; `CLAUDE.md` > Judged good 2026-08-20 for the acceptance criteria, and > Propulsion for the ratio set the pass leaves behind. **All three shipped and judged good in ONE drive, 2026-08-20** — the build-three-then-judge-once sequencing the owner set on the same day, which worked and is worth reusing. Owner: *"genuinely better than the playtest build ... still really good control but less arcade-y."* **`0.33`'s own framing was the thing that had to be discarded to close it.** It ruled `topSpeed` out of scope because no tester had asked for more speed, and treated the ramp as the only free variable; the complaint was actually the ceiling, and raising `topSpeed` 80 to 105 with `maxForwardAccel` untouched delivered the longer ramp AND a quicker approach to every speed below it, because `accelCurve` is read as a fraction of the cap. **Two corrections travelled with it.** A flatter curve was measured as genuinely QUICKER to 80 m/s (1.92s against 2.30s) and is the more arcade-y one, so "too slow" must not be answered by flattening the ramp. And `0.36`'s one open question — whether the lean arrives quickly enough — was answered by the owner raising `driveLateralPush` to 10 and judging it good, so **no dedicated response term was needed and `lateralDamp` was never touched** |

**The old Tier 4 was a deletion list and is fully executed**, which is why the number was free to reuse for
verification debt. Three dead APIs were removed. If a lock tone or a HUD pitch indicator is ever wanted, add
it deliberately rather than restoring them: each was dead because the consumer that justified it never
existed.
