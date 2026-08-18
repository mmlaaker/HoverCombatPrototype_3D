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
| **0** | 0.12 · 0.13 · 0.19 · 0.25 |
| **1** | 1.1 · 1.2 · 1.3 · 1.4 · 1.5 · 1.6 |
| **2** | 2.2 · 2.4 · 2.5 · 2.7 · 2.9 · 2.11 · 2.12 · 2.13 · 2.14 · 2.15 |
| **3** | 3.1 · 3.2 · 3.3 · 3.5 · 3.6 · 3.7 · 3.8 · 3.9 · 3.10 · 3.11 · 3.15 · 3.16 · 3.17 |
| **4** | 4.1 · 4.2 · 4.3 · 4.4 · 4.5 |
| **5** | 5.1 · 5.2 · 5.3 · 5.4 · 5.5 · 5.7 · 5.8 · 5.11 · 5.12 |
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
8. **0.13** shape chosen and half-built 2026-08-17; the orbit is **judged good**. What remains is
   the handback. **Next**
9. ~~**0.18**~~ shipped 2026-08-17. **The threshold was never the defect** -- the arming clock now
   decays instead of resetting, and the threshold stayed at 2. Outcome in `TuningLog.md` > The
   downed window > Closed as 0.18
10. ~~**0.16**~~ **retired 2026-08-17 without being judged.** The scoping that hides it is intentional
    and confirmed by the owner; the successor is **2.15**
11. **0.19** tooling that prevents a future bug rather than work you can feel
12. ~~**0.26**~~ fixed and **judged good 2026-08-17**, outcome in `TuningLog.md` > The boost gate was
    a step against reversing. The engage cost it carried was played and accepted
13. **0.25** charge jump squat. **Placed last by the owner**: it touches only the charge jump and the
    hooks already exist, so it is the least urgent thing here rather than the least valuable

### Before the next playtest build

Cheap things that make a long session worth more, or that stop it producing an unreadable result:

- **4.4** press `M` once in the first thirty seconds and confirm the marker landed in `Player.log`
  before continuing. Eight sessions have already been lost to a marker key that silently never arrived.
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

### 0.12 A sliver of bumper still clips on the pitch-up

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

### 0.13 The camera has no answer while the craft is flipped and helpless

**NARROWED 2026-08-16 after the owner tested the original scope.** The craft flips, you have no agency
until it rights itself, and the camera simply settles to the side. Nothing useful happens in a window
where the player is doing nothing and wants to be looking at where they will end up.

**The trick half is CLOSED and must not be re-proposed.** The original item paired this with "right
stick moves the camera during heavy air, so a long trick sequence can set up the landing". The owner
built a session around it and rejected it: **with trick control already on one stick, camera control on
the other is too much to do at once.** That is a decision from play, not a guess. Air control keeps the
camera behaviour it has.

**What remains open is only the flipped case. THE SHAPE IS NOW DECIDED AND HALF-BUILT.** The owner
chose "hand the player the stick" on 2026-08-17 and it shipped the same day: Right Stick X swings the
camera around the craft while downed, Right Stick Y keeps doing camera pitch. `downedYawSensitivity`
1.5, `downedYawRange` 180, `downedYawRecenterSpeed` 1.5, `downedCameraHold` 0.5. See `TuningLog.md` >
Camera control while downed for the verification, and > The downed window for the measurement that
justified it.

**The window is 2 to 5 seconds, measured, which is far longer than this item had assumed.** 1.96s for
a craft that comes to rest inverted, 4.84s for a wipeout carrying 25 m/s.

**JUDGED GOOD 2026-08-17** after the owner tuned it: *"I tuned the values. it feels good."* The two
that moved are scene overrides on `Prototype_Scene`, not prefab values: `downedYawRange` 180 -> **90**
and `downedCameraHold` 0.5 -> **0.2**. Sensitivity and recentre stayed at 1.5, so the axis still
matches camera pitch exactly.

**ONE CONCERN ON `downedCameraHold` 0.2, raised rather than overruled because it was never put to the
owner.** The hold exists to bridge the gaps where `IsDowned` flickers off during a wipeout, and **the
longest gap measured was 0.25s** — longer than the 0.2 now shipped. A wipeout that chatters like the
measured one will drop camera control for a few frames mid-crash. **The at-rest flip cannot show
this**, because it never chatters at all, so a clean test of the feature does not exercise it. If
camera control ever stutters at the start of a crash, this is the number, and 0.3 clears the measured
worst case. Left as tuned: it may simply not be noticeable, which is a play question rather than a
measurement one.

**What is left:**

- **The handback, which is the real remaining design problem.** Control returns at 33 degrees of tilt
  with the craft already moving, and in a sliding wipeout being flung at 19 m/s. The camera can be a
  long way off-axis at exactly the moment the player needs to drive. It currently springs back at
  `downedYawRecenterSpeed`, which is the pitch axis's answer borrowed wholesale rather than a
  considered one. **Suspect this first if recovery feels disorienting rather than the orbit itself.**
- **Whether a downed FRAMING is still wanted on top.** The second shape below was not rejected, just
  not chosen first; pulling back or reframing toward where the craft needs to get to would compose
  with the stick rather than compete with it.

**The two shapes not taken**, kept because neither was rejected on merit:

- **A specific framing for the downed state.** Pull back, or swing to a readable angle, or frame the
  craft against where it needs to get back to. Costs no input and cannot be fumbled.
- **Something else entirely.** The requirement is that the window stops feeling dead, not that a
  particular mechanism is used.

**Do not conclude that camera panning during recovery is a bug.** A separate report of the camera
swinging out on a standing charge jump was root-caused and fixed (it was the travel-heading bound, see
`CLAUDE.md`). What is left from that report is a wart worth knowing: **air control makes the left stick
PITCH the moment the craft clears the clearance gate, and a charged jump always clears it**, so holding
any throttle through a charge jump reliably starts a tumble from standing. Propulsion documents that
hazard for hops and solves it with the height floor, which by design does not protect the charged case.
**That is currently the most reliable way to reach the state this item is about.**

**One latent issue, separate and not the cause of anything observed.** `UpdateHeadingProxy` scales by
`Time.deltaTime` with no clamp, so a 194ms GC frame permits ~35 degrees of heading movement in a single
frame and the converge term reaches 90% of chassis yaw at once. Both terms are designed to prevent
snaps at 3.7ms and both stop protecting at 194ms. A `Mathf.Min(Time.deltaTime, ~0.05f)` would make the
camera hitch-proof. **Not applied:** it was never demonstrated to cause a felt artifact, and the
editor's idle allocation means the hitch may not exist in a build at all.

### 0.19 No camera preview state can reproduce a mid-flip pose

`CameraPreviewState.Inverted` sets the craft upside down but still travelling along its own nose, so it
cannot express the moment that mattered in the boost-gate bug: **nose swept past perpendicular to travel
while moving at speed.** Every one of the twelve states has velocity aligned with the chassis, which is
precisely the assumption that bug lived under.

That is part of why the defect survived so long: it was invisible in the inspector and only appeared
under a controlled playtest. **A state carrying a nose/travel disagreement would make this class of bug
judgeable without driving**, which is the argument for building it. Needs `FramingInputs.travelSpeed` to
differ from `forwardSpeed`, which `BuildPreviewInputs` currently pins equal on purpose.

Lowest priority in this tier: it is tooling that prevents a future bug rather than work you can feel.

### 0.25 The charge jump has no wind-up, and the charge itself is invisible

**LAST IN THIS TIER by owner decision 2026-08-17**, on the grounds that it touches only the charge
jump and the hooks are already in place. Not speculative and not blocked; simply the least urgent
thing here.

Owner idea, with the reference: **Jak 3's hoverboard high jump**, where holding the button crouches
the board and plays VFX gathering energy underneath it, released on the jump. The craft would do the
same: settle lower while charging, gather, and launch.

**The strongest argument for it is not feel, it is readability.** `jumpMaxChargeTime` is 2 and the
impulse runs `jumpImpulseMin` 20 to `jumpImpulseMax` 40 across it. That is a two-second window with a
doubling payoff, and **nothing anywhere tells the player where they are in it** -- no HUD element, no
vehicle state. A squat makes the ride height itself the charge meter, read off the craft rather than
off a corner of the screen, which is better placed than a bar would be because the player is already
looking there.

**Scope, decided with the owner:** charge jumps only.

- **No squat on a tap jump.** There is no hold, so any wind-up is pure added latency decorating a
  charge that does not exist. A charge jump is already a hold, so **the charge time IS the squat time
  and it costs no responsiveness at all.**
- **No squat on the air jump.** Nothing to compress against.

**What already exists, and this is why it is cheap.**

- **`ComputeEffectiveHoverHeight()` is the mechanism, already shipped and proven.** The ceiling duck
  uses it to lower the craft smoothly on its springs from `hoverHeight` 7 down to
  `minDuckHoverHeight` 1.5. The squat is a second consumer of it, not new physics.
- **The charge fraction already exists.** `jumpChargeTimer` against `jumpMaxChargeTime`, and the jump
  code already computes the normalised value to lerp the impulse with.
- **`SetAimPitch(degrees, weight)` is the pattern to copy** for handing intent from Propulsion to
  Foundation without Propulsion touching the Rigidbody itself.

**Three traps, and the first one will bite immediately.**

1. **A SQUAT DRIVES `HoverSupport` TO ZERO WHILE IT DESCENDS.** `heightFactor` is keyed to
   `_effectiveHoverHeight`, so dropping the target to 5 while the craft is still at 7 puts
   `avgDistance` outside the band and support reads **0**. Support gates energy regen, leveling
   strength, drag, air control, fall gravity and the drift hop impulse, so mid-squat the craft would
   look exactly like it was airborne. **The ceiling duck has this same latent problem today and
   nobody has hit it**, which is evidence the trap is real rather than theoretical. Settle this
   before anything else.
2. **Effective hover height gains a second writer.** Both the duck and the squat want to lower it, so
   use the seed / contribute / commit pattern the camera framing uses rather than letting whoever
   ran last win. Both lower, so the safe combination is the lowest of the two.
3. **A squat spends belly clearance**, which rests at about 4.7m. On uneven ground a deep squat
   scrapes while charging. Keep it shallow or clamp its depth against measured clearance.

**The design question that blocks building it: does the squat STORE energy or only express it?**
Lowering the springs and releasing them genuinely pushes, which would be honest and satisfying, but
it stacks on `jumpImpulseMax` and would overshoot the arc already tuned in `TuningLog.md`. Decide
before writing code, because it changes what gets written.

**The VFX half is Tier 2** with the rest of the blocked FX work, same split used for the drift cue.
The squat is Tier 0 because the movement half is not blocked on anything.

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
restores pose and velocity and nothing else, on purpose, so runs after one stay readable. It does settle
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

**The movement half is NOT blocked and lives in 0.25**, which is where the mechanism, the traps and
the open design question are recorded. Same split as 0.23 and 2.12.

**Sequencing note that matters here:** the squat makes the charge readable through ride height
alone, so 0.25 is worth judging on its own before this lands. If the squat already communicates the
charge, this becomes polish. If it does not, this is the thing that was actually missing.

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

### 4.4 Two things the first build did not test

The GC question itself is closed: the periodic hitch was the editor's own allocation and does not
survive a build. What that three-minute run did not cover:

**Allocation under sustained weapon fire and heavy projectile traffic.** Never measured. `AllocationBisect`
requires a parked craft and every run since has been driving, so these paths have no allocation figure at
all. **The build makes this cheap:** idle allocation there is 0.01-0.08 MB/s, so anything the weapons
allocate will stand out against a floor of essentially zero, which was never true in the editor's
4.47 MB/s. Fire continuously through a long run and read `alloc_mb_s` in the spike CSV.

**The marker key has never been confirmed in a build.** The first build run recorded zero markers because
nobody pressed `M`: not evidence of failure, but not evidence of success either, and this project has
already lost eight sessions to a marker key that silently never arrived. **Press `M` once in the first
thirty seconds of the next long run and confirm `[MotionTrace] MARKER 1` in `Player.log` before
continuing.** If it is missing, stop and fix it rather than spending the rest of the session producing an
unfalsifiable trace. Gamepad input is already proven in a build.

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
| **0.18** | `TuningLog.md` > The downed window > Closed as 0.18. Shipped 2026-08-17. **The title of this item was the wrong diagnosis:** `flipRecoverySpeedThreshold` was never the defect and stayed at 2. The arming clock was RESETTING to zero on any excursion past it, pricing a one-frame blip and a 229ms one identically, and the craft is pushed out of the gate by its own settling tumble -- so recovery interrupted its own clock. `flipRecoveryProgressDecay` 1 makes the cost symmetric. Worth 0.78s on a 25 m/s wipeout and 2ms at rest |
| **0.21** | `TuningLog.md` > Fall gravity and airtime, in the 2026-08-17 continuation rather than a new entry. Shipped `extraFallGravity` 30 -> 35. **Closed without a sweep**, because the closed-form model reproduced the 2026-08-14 table to within measurement slop on three independent quantities, so a sweep would have re-measured arithmetic. **The hard-landing framing this file carried was the wrong headline:** both of the owner's rules survive the whole usable range and the real cost is the barrel-roll landing margin, which has no ceiling to trip — it degrades continuously, ~25ms per 5 units. **Judged good in play the same day**, against a test the owner supplied: two barrel rolls still land |
| **5.10** | `CLAUDE.md` > Standing Decisions. **Wall jumping already works** and was never built; it falls out of the air jump firing along local up. Confirmed in play by the owner 2026-08-17 and accepted as advanced movement |

**The old Tier 4 was a deletion list and is fully executed**, which is why the number was free to reuse for
verification debt. Three dead APIs were removed. If a lock tone or a HUD pitch indicator is ever wanted, add
it deliberately rather than restoring them: each was dead because the consumer that justified it never
existed.
