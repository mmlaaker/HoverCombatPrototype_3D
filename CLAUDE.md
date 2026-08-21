# Hover Combat Prototype: Claude Context

**What exists, how it works, and what must not be broken.** This file is loaded every session, so it
carries current state and binding constraints only. How something came to be is git's job.

## Documentation Map

Seven documents, no overlap. **A fact lives in exactly one of them**; if you find the same thing in
two, that is a defect worth fixing rather than a redundancy worth keeping.

| Question | Document |
|---|---|
| **What exists, how does it work, what must not be broken?** | **`CLAUDE.md`** (this file) |
| What is not done? | `TODO.md` |
| Which unfinished things gate the next milestone, and in what order? | `ROADMAP.md` |
| How do I measure this without fooling myself? | `Measuring.md` |
| How was a shipped number arrived at, and what was rejected on the way? | `TuningLog.md` |
| What should this be? Why? | `GameDesignDocument.md` |
| Physics derivations and method. **Frozen** at `4a34f21`, values superseded | `PhysicsAudit.md` |

**Live tuning values come from `Assets/Data/VTP_Default.asset` and `Assets/Data/WD_*.asset`, never
from any document.** Numbers in this file are structural (ratios, thresholds that encode a rule) or
are labelled with the date they were measured.

## Behavioral Rules

- Read existing files before writing. Do not re-read unless the file has changed. Confirm intent with
  the user before writing.
- Thorough in reasoning, concise in output.
- Skip files over 100KB unless the task explicitly requires them.
- No sycophantic openers or closing fluff.
- No em-dashes.
- Do not guess APIs, versions, flags, commit SHAs, or package names. Verify by reading code or docs
  before asserting.
- **Weight owner feel-reports above both the docs and the derived readouts, and check the instrument
  before checking the game.** The owner's instinct has beaten the recorded data four times.

---

## Project Summary

Ground-level aerial dogfighting game. Hover vehicles behave like low-altitude jets dogfighting above
the ground. Combat and traversal are the same skill expression: momentum management determines fight
outcomes, not aim or loadout. Inspired by Twisted Metal, Rocket League, Unreal Tournament, Wipeout,
Jak 2, Jak X and Tony Hawk Pro Skater. Solo campaign against AI opponents; multiplayer is a future
phase.

**Design motto: "The fun comes from control."**

## Engine & Stack

Unity 6.3 URP (no HDRP). Rigidbody hover physics. Unity Input System (Vector2 action, axes split in
code). Cinemachine 3.1.6 third-person camera. GitHub + Fork + VSCode.

**Fixed Timestep 0.01 (100Hz) with Rigidbody interpolation on both vehicle prefabs.** Default 50Hz
against a high-refresh display produces visible jitter.

**This machine's display is 2560x1440 at 165Hz, which makes 60fps the worst available cap.** 165/60 is
2.75, so frames would alternate between spanning 2 and 3 refreshes, a 50% swing in on-screen duration
every frame. Evenly paced options are `vSyncCount` 1 (165), 2 (82.5) or 3 (55). Settled on VSync every
V blank. **For judging FEEL, even pacing matters more than the number.**

SMAA High lives on the Main Camera in the vehicle prefab and is not optional: see Standing Decisions.

---

## Design Pillars (Guardrails)

Reject or redesign any feature that violates these:

- **Hover is persistent.** Always airborne. Not a modifier or special state.
- **Momentum is the primary skill expression.** Positioning and momentum management determine
  outcomes, not loadouts, stats, or aim in isolation.
- **Combat and traversal are the same skill.** There is no mode switch. Map reading, momentum
  management and weapon use happen simultaneously.
- **Hit disruption over flat damage.** Being hit should disrupt momentum first, health second.
  Weapons that manipulate momentum vectors are prioritized over weapons that simply deal damage.
- **No asymmetric loadouts.** All vehicles share the same weapons and abilities. Identity comes from
  handling profile and one unique special weapon only.
- **Vehicles change how, not whether.** Every vehicle can execute every action. The moment a player
  feels they need a specific vehicle for a specific situation, the design has crossed into hero
  shooter territory.
- **Special weapons are exclamation points, not answers.** A special must not be the solution to a
  specific tactical problem.
- **Pickup placement creates vulnerability.** Powerful pickups live in exposed positions. Map control
  is the mechanism for forcing engagement.
- **Clarity at high speed.** Visual readability is a gameplay requirement. Spectacle that reduces
  readability is a design failure.
- **Feel first.** Mechanics must be fun before they are beautiful.

### Arena Design Guardrails

- **Track with rooms.** Corridors build momentum and create pursuit pressure. Rooms are where fights
  happen. Both must be present.
- **Three vertical layers.** Ground, mid and high must each serve a distinct tactical purpose.
- **Parallel routes at every major node.** At least two routes of roughly equal travel time.
- **Corridors are not transitions.** They are where momentum builds. Width must accommodate vehicle
  turning radius at combat speed.
- **Falling from height is a tactic.** Descent must be dramatic enough that following is genuinely
  risky.

---

## Architecture Principles

- **Opposing forces cause jitter.** Prefer mutually exclusive force application over tuning competing
  forces against each other.
- **Timestep mismatch is a jitter source.**
- **Update/FixedUpdate ordering is a real bug surface.** Use timestamps (`Time.unscaledTime`) over
  frame-scoped booleans for cross-boundary state.
- **ScriptableObjects for shared data.** Vehicle-specific scene references belong in slots, not
  definitions.
- **`DefaultExecutionOrder` is a last resort.** Only with a clear architectural reason.
- **`ForceMode.Force` and `ForceMode.Acceleration` are timestep-independent.** Confirmed.
- **Single write authority.** Where several features want to write one value, seed from the authored
  base, contribute through pure methods that touch no live state, and commit once. Used by Foundation
  for attitude and by the camera for framing. Both were adopted after "whoever ran last won" destroyed
  authored values.

---

## Vehicle Scale (measured 2026-08-07)

| | Measured | Real sedan | Ratio |
|---|---|---|---|
| Body collider | 6.88 L x 3.19 W x 2.35 H m | 4.7 x 1.82 x 1.45 | **1.46 / 1.75 / 1.62x** |
| Visual mesh | 7.74 x 3.81 x 2.64 m | | |
| Collider union (incl. rims) | 6.88 x 3.81 x 2.35 m | | |
| Mass | 1000 kg | ~1400 kg | 0.71x |
| Inertia tensor | (3293.6, 3756.1, 1091.0) | | roll ~3x easier than pitch |

**The collider union is NOT centred on the vehicle origin.** In vehicle-local space: centre
`(0.000, 0.877, 0.732)`, half-extents `(1.907, 1.174, 3.439)`. The hull sits 0.88m above and 0.73m
forward of the transform. **Anything that builds a box around the craft must apply that offset**;
assuming the origin is the centre hangs the box almost a metre below the craft, and the corner it
misplaces is the rear-bottom one, **which is the first corner to leave frame**. This cost a full
degree of false clipping in the camera framing check before it was caught.

**Do not read these back from `Collider.bounds` at runtime**: that is a world-space AABB and inflates
the moment the craft banks. Renderer bounds are useless too, since they include world-space particle
systems and measure 13.5m tall.

Consequences worth knowing before tuning:

- **Density is ~16 kg/m^3 of bounding volume, about 7x lighter than a real car.** Nothing is broken by
  this and the impulse tuning is calibrated against it, but every `impactForce`, `splashImpactForce`,
  jump impulse and dodge value would rescale if the mass ever moved.
- **Perceived speed is speed divided by vehicle length, and this craft is long.** It reads about half
  as fast as its km/h suggests. If a tuning pass leaves you wanting more sensation of speed, shrinking
  the craft achieves it as effectively as raising `topSpeed` and disturbs far less.
- **The belly rides about 4.7m off the ground**, higher than a double-decker bus. On-theme for
  "low-altitude jets", but it will look strange beside any human-scale prop.

**Decision: not rescaling** (see `TODO.md` 5.8 for the two gates that would reopen it). **A rescale
would not disturb any attitude tuning**, because the hover and attitude system is inertia-independent
by construction: every Foundation torque uses `ForceMode.Acceleration`, which ignores the inertia
tensor, and `ApplyTurning` multiplies by `inertiaTensor.y` to cancel it explicitly. What it WOULD
disturb: impact spin (`AddForceAtPosition` uses `ForceMode.Impulse`, which does respect inertia) and
every length value (`hoverHeight`, `sensorRange`, `ceilingClearance`, `splashRadius`, weapon ranges,
muzzle offsets).

**Real-world scale would NOT make the tuning values easier to reason about**, which is the reason it
was considered. The values are already in real units. What actually breaks intuition is
`extraGravityMultiplier` 3, which puts every derived figure ~4x off intuition regardless of vehicle
size. The fix for that was the derived-values inspector, not a rescale, and it exists.

---

## Modules

Format is **Does** (current responsibility), **Public** (the surface other modules use), and
**Constraints** (rules that a future change must not break, and why). Version history is not kept;
`git log` has it.

### `HoverController_Foundation.cs`

**Does.** Spring-damper hover lift with gravity feedforward, so the spring corrects error only instead
of also holding the craft up. Leveling torque as the single attitude authority. Three-path recovery
(upright-stuck unstick, flip righting, and the dead-man righting that backstops it). Air control torque on behalf of Propulsion. Asymmetric
gravity (generous on the rise, decisive on the fall). Ceiling duck and charge squat, min-combined into
one ride-height target. Hard-landing spring give. Hover
rays read the interpolated smooth surface normal, not the flat triangle normal, and are cast from
where the sensors would be if the craft were not aiming. In aim mode the craft levels to the HORIZON
rather than to the ground, so terrain does not steer the gun.

**Public.** `IsHoverGrounded`, `HoverSupport`, `HasAirControlClearance`, `AverageGroundNormal`,
`IsDowned`, `SetRecoveryEnabled(bool)`, `SetAimPitch(degrees, weight)`,
`SetAirControl(pitch, roll, weight)`, `SetJumpCharge(fraction 0..1)`, `OnHardLanding(severity 0..1)`.

**Constraints.**

- **Do not modify without explicit justification.** Last modified 2026-08-20 to close `0.27`, the
  soft lock, which was the milestone blocker and had five sightings. Before that, 2026-08-17 on the
  owner's report that aiming sank the craft into the terrain. There is no queued justification now;
  the next change needs a new one.
- **Leveling torque is the SINGLE attitude authority.** Nothing else may apply pitch or roll torque.
  Propulsion hands over intent; it never applies the torque itself.
- **`_hoverCentroid` is solved once at the top of `ApplyHoverForces` and read by both probes that
  need it** (the ceiling duck and the air-control clearance), each of which used to walk the four
  hover points itself. Safe to share only because NOTHING MOVES between them: `AddForce` does not
  touch the Transform, and the solver integrates after `FixedUpdate` returns, so every Transform holds
  the pose the tick started with. **That is also the constraint** -- if anything is ever added that
  moves the chassis mid-tick (a direct `rb.position` write, a teleport, a warp), this cache is the
  thing that silently goes stale, and the same reasoning covers the hoisted `chassisOrigin` inside the
  sensor loop.
- **The hover sensors are placed, pointed and loaded as if the craft were NOT aiming**, and every
  term in the spring loop shares that frame: ray origin, ray direction, the gap, the closing rate
  and the position the force is applied at. Splitting them puts the spring in one frame and its
  damping in another. What this fixes is in `TuningLog.md` > Aiming and the hover sensors; the short
  version is that a sensor bolted to a tilting hull measures a slant instead of a height, and that
  one error sank the craft, drove `HoverSupport` to zero while parked, pointed the front sensors
  past the nose, and unloaded the rear pair entirely past about 20 degrees.
- **The un-aim angle is MEASURED FROM THE HULL, never from the commanded aim.** The chassis is a
  torque servo and lags the stick, so anything sized from the command corrects a tilt the craft has
  not reached yet. That shipped once and put ~19 m/s^2 of phantom lift under a fast stick sweep.
  See `Measuring.md` trap 41.
- **The un-aim is referenced to `_lastGroundNormal`, and the ORDER that makes that possible is
  load-bearing.** `ApplyHoverForces` resets `AverageGroundNormal` to `Vector3.up` at the top of the
  method, before the sensor loop refills it, so the previous tick's value has to be captured before
  that line or the un-aim silently references the world vertical instead of the surface. **That was a
  real shipped bug and it is invisible on flat ground**, where the two vectors are the same: on a 20
  degree ramp it tilted the sensor frame 21 degrees, unloaded the rear pair to exactly zero force,
  and left the craft hanging off the front pair with a +71 nose-up torque the leveling servo spent
  its whole authority cancelling. Measurements in `TuningLog.md` > Aiming on slopes. The sensors have
  to be aimed before they have measured anything, so one tick of lag on a normal is the correct price.
- **In aim mode the attitude reference is the HORIZON, not the ground**, via
  `foundation.aimLevelsToHorizon` blended by strafe weight. Drive mode is untouched at any value of
  it, because that weight is zero there. Airborne the two references are the same vector, so this is
  grounded-only by construction and the inverted and mid-trick cases need no rule of their own.
- **`aimPitchTrackingStrength` is no longer propping anything up.** Read the current value as a feel
  choice about how the nose answers the stick, and nothing else. Its old 200 was holding the nose
  down against the un-aim bug above, which is why it had to be seventeen times leveling strength; with
  that gone, everything from 12 to 200 holds a 20 degree ramp within 8 degrees at full support.
- **Un-aiming is PITCH ONLY.** Roll must keep reaching the sensors: bank is real, the drift flip
  lived in the roll axis, and **wall jumping depends on the sensors rotating with a rolled craft**
  (see the wall jump entry below; it is confirmed working, not speculative).
- **Consequence, and it is a real one:** while aiming, the springs generate no pitch-restoring
  torque, because all four now push on equal arms. Attitude on that axis is held entirely by the
  leveling torque. That moves toward the single-attitude-authority rule rather than away from it,
  but it is a behaviour change and not a refactor.
- **Use `HoverSupport`, not `IsHoverGrounded`, for anything that should behave differently in the
  air.** `IsHoverGrounded` answers "can the rays see ground", which stays true for **2.5m of free
  fall** because above `hoverHeight` spring compression goes negative and clamps to zero. Four systems
  once read that band as grounded. `HoverSupport` is continuous on purpose: a second boolean just
  moves the cliff, and handing air control authority while leveling is at full strength puts two
  attitude authorities on one axis. Fall gravity and air control scale by `(1 - support)` while
  leveling and drag scale by support, so the handover is a crossfade with no overlap.
- **Drive, jump charge and drift entry deliberately keep the generous `IsHoverGrounded` signal.**
  Losing throttle on a bump crest would be worse than the bug.
- **TWO writers lower the ride height and neither wins by running last.** The ceiling duck is
  involuntary, the charge squat is the player holding jump; the target is seeded at `hoverHeight`,
  each contributes, and the LOWEST commits. Both only ever lower, so the minimum is the one
  combination that cannot violate either.
- **The duck's result is kept separately, because the two are NOT interchangeable downstream.** The
  air-control clearance probe reads the duck alone: it measures room ABOVE the height the craft is
  entitled to, and **a voluntary squat lowers the craft without creating any room.** Feeding it the
  combined value reports squat depth as free space and grants attitude authority the craft has not
  earned.
- **Air control needs its OWN downward probe.** The hover sensors top out at `sensorRange - hoverHeight`
  of measurable clearance while a tap jump apexes well above that, so the threshold is simply outside
  what they can see. It measures clearance BELOW rather than height gained, so a hop off a ledge arms
  and the same hop on flat ground does not, with no special case for either.
- **That gate compares the BALLISTIC PEAK from current rise velocity, not present clearance.**
  Present-tense cost 0.225s of dead air at the start of every charged jump, which is exactly where a
  flip needs to read as already having momentum. Peak height is conserved under ballistic motion so
  the prediction cannot flicker during the climb, and it decays to plain measured clearance on descent.
- **Hover rays must skip self-hits.** `groundLayers` is Everything and the chassis colliders sit on
  the vehicle layer, so past ~15.5 degrees of bank the craft shot its own rims and one flank's spring
  jumped an order of magnitude. This was the drift flip.
- **`sensorRange` defines GROUNDED, not just sensing.** Between `hoverHeight` and `sensorRange` the
  springs produce zero lift but the craft still counts as grounded. Do not widen it casually. Hard
  landing speed is sampled on that edge, so raising `sensorRange` makes hard landings *less* likely.
- **Flip recovery arms on its own angle** (`flipRecoveryArmAngle`), separate from the downed lockout
  (`flipRecoveryAngleThreshold`). One value doing both jobs left a band too tilted to drive and not
  tilted enough to authorize righting.
- **Righting releases at `flipRecoveryReleaseAngle`, not the arm angle.** Releasing at the arm angle
  parks the craft in a hover-supported equilibrium at ~78 degrees.
- **Recovery disarm is on attitude only.** Adding an `|| IsHoverGrounded` term revokes authority
  mid-rotation, because a craft on its flank still lands two rays on the floor.
- **THE DEAD-MAN RIGHTING NEEDS BOTH OF ITS HALVES AND NEITHER IS SUFFICIENT.** It forces
  `rightingAuthorized` AND suppresses the hover push, and each half closes a different soft lock.
  Against a wall righting was already authorized and losing, so forcing authority changes nothing;
  carried by something moving the righting never armed at all, so suppressing the springs unblocks
  nothing (measured: the timer fired and the craft sat at 177 degrees for another twelve seconds).
  **Do not simplify it to one of them.**
- **Its detector asks whether the craft is MOVING, not whether it is improving**, and that is what
  keeps ordinary recoveries out of it. A fast wipeout spends about two seconds rolling FURTHER over
  before it settles, so a progress-based test reads an ordinary crash as stuck and fires ~2s early.
  The margin is wide rather than tuned: a stuck craft moves 0.00 degrees per window against a
  1 degree band, while the quietest window of a 25 m/s wipeout still swings 5.76.
- **The hover push FADES back in over `forcedRecoveryLiftRestore`; it must never be restored in one
  step.** The step version is the only thing in this project that has ever genuinely launched the
  craft: 13m sideways and 4.63m above ride height, entirely manufactured by switching the springs off
  and back on. **The limit on the value is not the craft dropping** (it rests on the ground
  throughout), it is that control returns before the fade finishes, so a long one hands the player
  soft hover.
- **Its window and band are `const`, deliberately, and should not become tuning fields.** The owner's
  rule is that `flipRecoveryDelay` alone prices a flip; a second duration on the same subject would
  be a competing knob for one feeling. They are a detector, not a feel choice.
- **Unstick arms only while vertically settled, and only against surfaces within
  `unstickMaxSurfaceAngle` of horizontal**, so belly scrapes get no pulse trains and walls and other
  vehicles are never pushed off.
- **Smooth normals need `Mesh.isReadable` and fail SILENTLY without it.** See `TODO.md` 3.10.
- Ground contact is tracked by timestamp, not by an exit callback.

### `HoverController_Propulsion.cs`

**Does.** Drive, reverse, strafe, drag and over-speed bleed. **A shaped acceleration ramp.** Boost
blend. Drift as a held slide. Dodge burst. Grounded and air jump. Chassis bank. Forwards air-control
intent to Foundation.

**Public.** `DriftLerp`, `StrafeModeBlend`, `AirControlWeight`, `BoostLerp`, `OnJumpDenied(bool grounded)`,
`OnDriftHop`, `OnDodge`, `OnDriftSpent`.

**Constraints.**

- **While drifting, "am I at top speed?" is answered by TOTAL horizontal speed, in every place that
  asks.** Both `ApplyDrive` and `ApplyOverSpeedBleed`, or neither. Enforcing against the forward axis
  is the same thing only while heading equals velocity, and a drift separates them by design, which
  turned the cap into `total = cap / cos(angle)`.
- **Drive and drag are mutually exclusive.** `DRIVE+DRAG` in the gizmo is legitimate in two documented
  cases (throttle inside the drag fade band, and drift forcing full drag weight); only `UNEXPLAINED`
  is a fault.
- **`maxReverseAccel` doubles as the brake** and is deliberately NOT strafe-blended, so braking is
  identical in both modes. It is set by stopping distance, not by reverse speed. **It is exempt from
  `accelCurve` for the same reason**, and the exemption is structural: the reverse branch never
  touches `blendedFwdAccel`, so it cannot be shaped by accident.
- **`accelCurve` is read against a FRACTION of the cap in force, never against m/s.** That is the
  only reason one curve covers drive, boost and strafe. It also fixes an asymmetry that predates it:
  boost scales accel by 1.5 and the ceiling by only 1.25, so boost used to reach its HIGHER top speed
  in LESS time than the craft reached its ordinary one. A curve read off absolute speed would have
  preserved that. `AccelCurveMultiplier` is the single site.
- **Because of that fraction, `topSpeed` is the HORIZONTAL SCALE of the acceleration ramp and not
  just its ceiling.** Raising it stretches the same curve over a wider range, which puts the flat
  part of the curve over more of the speeds actually driven — so **raising the cap makes the craft
  reach every absolute speed SOONER while taking longer to top out.** Measured 2026-08-20: cap 80 to
  105 with `maxForwardAccel` untouched moved time-to-60 from 1.14s to 0.89s and time-to-top from
  2.30s to 3.02s. **"Too slow" is therefore at least as likely to be the ceiling as the ramp**, and
  the two levers pull opposite ways on time-to-top. Check which complaint is being made before
  touching either.
- **Six values are RATIOS against `topSpeed` and must move with it.** `maxReverseAccel` and
  `reverseTopSpeed` at 0.838, `strafeTopSpeed` and `minDriftSpeed` at 0.667, `driftSustainedTopSpeed`
  at 0.562, and `strafeAccel` scaled by the CAP ratio rather than against `maxForwardAccel` (what
  sets strafe's feel is time-to-its-own-cap). **`minDriftSpeed` must EQUAL `strafeTopSpeed`; the
  profile validator enforces it.** The one that gets missed is `maxReverseAccel`, because it reads as
  a reverse value and is actually the brake — left behind, stopping distance from top speed went 48m
  to 82m.
- **`hardLandingMinSpeed` and `hardLandingMaxSpeed` are NOT in that set and must not be scaled with
  `topSpeed`.** `impactSpeed` is closing speed along the ground normal, set by the jump impulses and
  `extraFallGravity`, not by how fast the craft drives. Scaling them switches hard landings off for
  ordinary jumps. They look like speed ratios; they are landing values.
- **A FLATTER `accelCurve` produces a QUICKER craft that feels MORE arcade-y**, and this was measured
  rather than reasoned. A three-key curve reached 79 m/s in 1.92s against the shipped curve's 2.30s
  and was rejected in play. **Thrust that stays high and then stops at a wall reads as a governor;
  thrust that tapers toward a limit reads as physical.** Do not answer "feels too slow" by flattening
  the curve (`TuningLog.md` > The speed pass).
- **DRIFT IS EXEMPT FROM `accelCurve` and must stay exempt.** A drift's cap falls by design, so the
  fraction pins near 1 and the whole slide runs at the curve's floor — measured as a held slide
  turned into a brake, 53.5 m/s against 77.3 at 1.5s in. Faded by `driftLerp`, not switched. **A
  drift already costs acceleration; that is what it is FOR**, so shaping it charges twice.
- **The curve's right-hand end must stay above zero.** A curve that reaches zero makes top speed
  asymptotic: the craft approaches the number in the inspector forever and never arrives, which
  presents as a mysterious cap below the tuning value. There is a 0.01 safety floor in code, but the
  floor that is meant to be tuned is the curve's own.
- **The lane change is NOT a steering mechanism and must not become one.** Owner's definition,
  2026-08-20: *"a lane change, or a way to make a small adjustment to your line or line up a shot a
  little better, without being twitchy."* **`driveLateralPush` is scaled by forward speed so it is a
  constant ANGLE off the line** (5.7° at the shipped 8) rather than a constant sideways speed —
  which makes it exactly zero at a standstill, because a stationary craft sliding sideways is
  strafing without the trigger. **That scaling is the owner's hard constraint, not a refinement.**
- **The lane change is applied AGAINST the lateral drag, in the same block, and the two are one
  mechanism.** The knob is the sideways speed the craft settles at, and multiplying by the same
  damping coefficient is what makes that true regardless of `lateralDamp`. **The cost is that the
  knob sets distance and not timing:** measured at 4/8/16, the craft is 64% of the way to its own
  final speed at one second in all three cases, so the shape is invariant and only the scale moves.
  **A small lean that arrives quickly would need a dedicated response term. `lateralDamp` is NOT
  that term — the owner has ruled it out, as too much feel is built on it.**
- **`yawSpeedFade` reads the value straight out, unlike `accelCurve`.** Settled yaw rate is
  `yawAccel / yawDamping`, which is linear, so 0.5 on the curve IS half the turn rate — no integral
  between the number and the feel. **It scales `yawAccel` and must not scale `yawDamping`:** the
  first lowers the rate the nose tops out at, the second would make the turn mushy to initiate,
  which is a different complaint from the one that was reported.
- **The yaw fade reads ACTUAL horizontal speed over the UNBOOSTED `topSpeed`.** Both halves are
  deliberate and both differ from `accelCurve`. Not the forward axis, because hard cornering opens a
  slip angle that collapses it while the craft is still travelling just as fast, which would relax
  the fade inside the manoeuvre it governs. Not the boosted cap, because a corner at 100 m/s is
  harder than one at 80, so dividing by it hands authority back exactly when the craft is fastest.
  **`boostYawMultiplier` extends the fade across the band above `topSpeed`**, which is otherwise
  flat, and is driven by speed rather than by `boostLerp` so boosting out of a slow corner is free.
- **The yaw fade is DRIVE MODE, GROUNDED, NON-DRIFT only, and each exemption is load-bearing.**
  Drift, because it only exists above `minDriftSpeed` so an unexempted fade bites hardest inside it.
  Strafe, because right stick X is *aim* yaw there and fading it degrades aiming. Airborne, already
  scaled by `airTurnMultiplier` and belonging to tricks. Blended by `Max` of the two blends, not by
  multiplying them, so a strafing drift does not land in a half-faded band.
- **Do not sweep `accelCurve`. Solve it.** Time to top speed is
  `(topSpeed / maxForwardAccel) x integral of 1/curve(f)`, accurate to about 1%, so a target time
  maps to keys arithmetically in one `eval` call with no play mode. **The three complaints map to
  three different parts of the curve and are not interchangeable:** punch is the leading flat run,
  sluggishness is the middle keys, and slow to finally top out is the right-hand end — which moves
  time-to-60 by 0.017s across a five-fold change and therefore cannot fix the other two. Recipe and
  worked example in `TuningLog.md` > Tuning the ramp to a target time.
- **The `HoverSupport < 1f` term in the air-control gate looks redundant and is load-bearing.** It is
  what guarantees leveling and air control never both act on one axis under any tuning.
- **The downed lockout must include THRUST, not just torque.** Drive is a force, so it escapes a
  torque-only lockout and keeps a downed chassis moving above `flipRecoverySpeedThreshold`, resetting
  the arming clock. Yaw *damping* is deliberately still applied while downed: it is a stabilizer, not
  agency.
- **Drift's purpose is aiming.** Weapons fire along chassis forward, so a held slide is the only way to
  aim off the line of travel at full speed. The angle is the reward. **No exit payout and no spinout,
  both explicitly rejected by the owner.**
- **`driftLateralDamp` must stay below `lateralDamp`.** Above it, drift becomes a grip-assist button
  that slides *less* than ordinary cornering.
- **Cap expressions are shared helpers** (`BlendedTopSpeed`, `StrafeTopSpeedScaled`,
  `BlendedReverseTopSpeed`), because two methods rebuilding the same cap separately has produced a
  dead band twice.
- **Continuous boost requires forward-dominant throttle.** Pure lateral stick plus boost fires a dodge
  instead, so a player cannot boost straight sideways at all.
- **Drive thrust and forward drag act along `_driveAxis`, never along `transform.forward`.** In
  strafe mode the nose is an AIM direction, not a travel direction, so raw chassis forward fired the
  engine at the sky or the floor in proportion to the aim angle. Solved ONCE per FixedUpdate and
  read by both, because drag is the force that opposes drive and a damping force on a different
  axis than the thrust it opposes is not damping. Fixing only the thrust moves the artifact from the
  press to the release, since full drag arrives at throttle 0. Drive mode is identity.
- **Reverse is the binding case for anything aim-pitch related**, because `maxReverseAccel` is
  deliberately not strafe-blended (it doubles as the brake), so it is the larger number.
- **`ApplyChassisBank` is not visual-only:** `meshRoot` owns all five mesh colliders.
- Lateral damp excludes intended strafe velocity, so dodge bursts at the cap are additive (surge, then
  glide back down).
- **Known and deliberately not fixed:** `ApplyBoostBlend` reads `_strafeModeBlend`, which updates later
  in the same tick, so the strafe-mode boost gate runs one tick behind. Hoisting it would also change
  what the dodge trigger sees, which is a behaviour change rather than a consistency fix.

### `HoverController_Energy.cs`

**Does.** Shared non-damaging ability pool. Regen with a spend lockout. EMP freeze. Pays out to tricks.

**Public.** `Energy`, `EnergyNormalized`, `IsEmpFrozen`, `IsRegenerating`, `TryConsume`, `Grant(float)`.

**Constraints.**

- **Regen is grounded-only**, gated on `HoverSupport` (not `IsHoverGrounded`, which stays true for
  2.5m of free fall, precisely the window this exists to stop paying for). Airtime used to refill the
  meter for free, putting passive regen in direct competition with the trick economy it is the
  fallback for.
- **The lockout TIMER still runs while airborne, deliberately.** It measures time since you last
  demanded from the pool, which is a fact about spending and not about where the craft is.
- **`Grant` does NOT touch the regen lockout.** That lockout punishes SPENDING, and being paid is the
  opposite transaction.
- **`Grant` refuses while EMP-frozen**, matching `TryConsume`, so a freeze cannot be worked around by
  banking a trick during it. See `TODO.md` 3.15, which is the open design question.
- **A missing `foundation` reference fails OPEN** (regen everywhere) rather than closed, so it cannot
  silently starve the pool and look like a tuning problem.
- Two consequences fall out of the mechanism rather than being chosen: **flip recovery costs energy
  time on top of the control lockout** (a craft on its flank has no support), and **boosting airborne
  is strictly one way** with no recovery until you touch down.

### `HoverController_Tricks.cs`

**Does.** Pays energy for completed revolutions landed in the air. **Owns no physics and writes
nothing to the Rigidbody**, which is why Foundation needed no modification.

**Public.** `BarrelRollCount`, `FlipCount`, `IsTracking`, `EscrowEnergy`,
`OnTrickResolved(bool banked, float payout, float granted)`, `PayoutFor(...)`.

**Constraints.**

- **Revolutions are COMPLETED and bank on the CROSSING.** Never accumulated travel (wobbling the stick
  became an income), and never net displacement (over-rotating a flip and straightening for the
  landing would eat the flip just earned). Progress is SIGNED.
- **It banks the THRESHOLD per revolution, not a whole turn.** A whole turn makes the credit a
  one-time head start and lets the instrument's ~3% shortfall accumulate until a long spin silently
  fails to clock.
- **The corkscrew multiplier scales the PAYOUT, not the accrual.** Scaling the accrual meant a diagonal
  turn fed each axis at half rate and completed neither.
- **Discounted payouts round to the NEAREST increment, never upward.** Upward erases any discount
  smaller than one increment, which at these prices is most of them. Half-up, not `Mathf.Round`, which
  is banker's rounding and would send neighbouring values in opposite directions.
- **Pitch and roll only; yaw excluded.** A flat spin is neither trick, and paying for it would make
  sitting on the stick an income.
- **Body frame is correct here** (against trap 19): a flip and a roll are defined by the craft's own
  axes, so asking which WORLD axis it turned about gives a different answer for the same trick
  depending on which way the player was facing.
- **Arrival needs BOTH hover support and physical contact.** A clean landing produces no collision at
  all (the springs catch the chassis first) while a craft on its flank produces no support (the rays
  point sideways), so **each signal is blind in exactly the case the other catches.**
- **Landing attitude is decomposed against the surface normal, not world up**, so a clean landing on a
  hillside is judged on whether it was clean rather than on how steep the hill was.
- **SETTLE is a second gate** because a tumble latches `IsDowned` a fraction of a second AFTER contact,
  and paying on the contact frame would pay for crashes.
- **Report the payout AND what reached the pool.** One number cannot carry both meanings, and a readout
  honest about the wrong quantity reads as a bug in the system it reports on. See trap 39.
- **It must sit on the vehicle root**, because arrival detection needs `OnCollisionEnter`.
- Armed on the rising edge of air control, once per flight. A tap jump never clears the clearance gate
  so it can never pay, and an EMP tumble or a rocket hit pays nothing because neither arms air control.

### `HoverController_Weapons.cs`

**Does.** Weapon slots, four `WeaponType`s (SingleShot, Automatic, Missile, Mine), two
`ProjectileMode`s (Instantiated, ParticleSystem). Missile lock.

**Public.** `ActiveSlot`, `ActiveSlotIndex`, `CurrentLockState`, `LockProgress`, `LockTarget`,
`RefillAmmo()`, `SetActiveSlot()`.

**Constraints.**

- **Both switch paths share `SwitchToSlot`.** Switching must stop the outgoing emitters, restore their
  emission rate (the wind-down path leaves it scaled), clear wind-up charge, and drop any missile lock.
  `Update` only ticks the ACTIVE slot, which makes the idle-branch cleanup unreachable the instant a
  slot stops being active, so an orphaned emitter fires permanently.
- **The same-index early-out is a correctness guard, not an optimisation.** Without it,
  `SetActiveSlot(currentIndex)` while firing stops your own burst.
- **Weapons never spend energy.** EMP only gates fire via `IsEmpFrozen`.
- `SetActiveSlot` does not yet skip empty slots (`TODO.md` 3.7).

### `HoverCameraController.cs`

**Does.** Solves camera framing every `LateUpdate`. Drive cam follows a runtime `CameraHeadingProxy`
carrying position and a stabilized yaw; strafe cam uses `LockToTargetNoRoll` with vertical-only
position damping, slew-limited so it cannot collapse at a landing. Contributors cover pitch orbit,
shoulder shift, look deadzone, turn-bleed suppression, speed look-ahead, boost lens and pull-back,
and a framing guard.

**Public.** `NotifyVehicleWarped(positionDelta)`, `EvaluateState`, edit-mode preview by
`CameraPreviewState`.

**Constraints.**

- **`[DefaultExecutionOrder(-100)]` is load-bearing and must stay.** This writes the Follow Offset and
  the composer's Target Offset in `LateUpdate`; `CinemachineBrain.LateUpdate` then reads them to place
  the camera. Both sat at order 0 on DIFFERENT GameObjects (this on `Camera`, the brain on
  `MainCamera`), so Unity's arbitrary tie-break decided the order and nothing pinned it. **Measured
  2026-08-20 before pinning: 832 comparable frames, 0 stale -- it was already correct, which is exactly
  why the attribute matters.** If the tie-break ever flipped, the brain would place the camera from the
  PREVIOUS frame's offsets: roughly a metre of error at 90 m/s, arriving only on the boost and
  look-ahead transients the offsets exist to drive, with nothing logged. It would present as "the
  camera feels slightly behind" and there would be no code change to blame.
- **`Measure()` finds the worst corner in TANGENT space and converts to an angle once.** The normalize
  is unnecessary (every use is a ratio of two dots against the same vector) and `max|atan2(y,cf)|`
  equals `atan(max|y|/cf)` while `cf > 0`, so eight square roots and fourteen `atan2` calls are
  removable EXACTLY, not approximately. Verified 2026-08-20 across every preview state at three aspect
  ratios: worst difference **0.0000052 degrees**. **Do not "restore clarity" by putting the per-corner
  `atan2` back** -- the guard bisects, so this runs up to 14 times a frame, and it does so precisely
  during hard cornering, boost and drift, which is when the frame can least afford it.

- **SINGLE WRITE AUTHORITY.** Seed a `FramingSolution` from the authored bases, CONTRIBUTE via pure
  methods that touch no Cinemachine state, COMMIT once at the end of `LateUpdate`. Adding an effect
  cannot clobber another because it does not write anything. Writing straight to the transposer,
  composer or lens had already destroyed two authored values.
- **Camera SHAKE is deliberately excluded from this solver.** Impulse listeners apply post-body,
  downstream of everything here, so they already cannot fight framing, and folding them in would
  destroy the property that makes them safe.
- **`ForwardGate` reads TRAVEL, never the nose.** The gate answers "am I backing up", which is a
  question about travel. The chassis only answers it while the nose points along the travel line, and
  a flip breaks that for about 7ms per crossing, slamming every boost cue shut at full speed.
- **The gate is SLEW-LIMITED, and the limit is on the result rather than the threshold.**
  `forwardGateSpeed` stays low so the gate remains a direction test; `forwardGateSlew` bounds how fast
  the gate value may move. Without it the gate stepped 1 to 0 in 23ms whenever travel speed crossed
  zero — ordinary play, since boost in reverse is supported — carrying 4 degrees of lens and half a
  metre of rig at 21 m/s, and snapping back on when the stick came forward again. **Do not "fix" a
  future recurrence by widening `forwardGateSpeed`**: that converts a direction test into a speed
  ramp, which `TuningLog.md` rejected twice, for 0.20 and again for 0.26.
- **The travel-heading bound limits drift SINCE air control engaged, never absolute offset from
  travel.** Do not "simplify" this back. The absolute form assumes a craft takes off pointing roughly
  where it is going; take off sliding backwards and the clamp is unsatisfiable at entry, so it hauls
  the camera through the player's roll to satisfy itself. **Do not conclude the bound is unnecessary
  either:** it still prevents the divergence and landing stutter it was built for.
- **The travel heading is LATCHED, not recomputed.** `travelHeadingMinSpeed` means "stop refreshing",
  not "stop applying". The old reading disabled the bound during exactly the steep post-flip descents
  it was built for, because a flip bleeds off horizontal speed. A falling craft keeps the heading it
  had, so the latched value stays correct.
- **The rate ceiling is NOT dead code, despite not being what fixed the stutter.** It is what
  guarantees no single frame can snap, and it is applied ONCE to the total move rather than to a branch
  step, so neither branch nor the bound can produce a snap whatever the others do.
- **Speed look-ahead is RATE-LIMITED, and that limit is memory, so it lives in the integrate stage.**
  `IntegrateLookAheadDistance` slews `_lookAheadDistance` toward `SpeedLookAheadTarget` at
  `speedLookAheadSlew`; `ContributeLookAhead` just adds the arriving distance and stays pure.
  **`FramingInputs.lookAheadDistance` is metres, already limited** — do not "simplify" it back into a
  function of `forwardSpeed` inside the contributor, which is what it was before 2026-08-17 and is
  what let boost drag the look point through its whole six-metre swing 50% faster than throttle can.
  The integrator runs OUTSIDE the strafe branch for the same reason the boost envelope does: freezing
  it while the crosshair is up would let it arrive as a step when drive resumed. Reasoning and the
  three-run measurement are in `TuningLog.md` > The boost jolt at a standstill.
- **The strafe vertical damping COEFFICIENT is rate-limited too, for the same class of reason, and it
  also lives in the integrate stage.** `IntegrateStrafeDamping` slews toward
  `verticalDamping * (1 - hoverSupport)` at `verticalDampingSlew`; `ContributeStrafeDamping` just
  reads it and `inp.hoverSupport` is deliberately unused there. **The support gate was right about
  WHAT the damping should be and wrong only about how fast it may travel:** support changes at a mean
  of 0.09 per second over undulating ground, which is what it was validated against, and at roughly
  50 per second on a landing, which snapped two metres of accumulated lag out in two frames. Holding
  the damping on THROUGH the touchdown is the point, so the lag closes out under the damping instead
  of being teleported away. `TuningLog.md` > The landing snap in strafe.
- **Preview and live feed the SAME solver**, through `GatherLiveInputs` and `BuildPreviewInputs`. A
  preview that recomputes the framing starts lying the first time a contributor changes.
- **Three preview states deliberately break the nose-follows-travel assumption**, and they exist
  because two shipped defects hid in exactly that gap and neither was reproducible in the dropdown.
  `ReverseBoost` (gate must read 0 and kill every boost term), `NoseAcrossTravel` (forward speed 0
  while travelling at full speed, so the look-ahead collapses while the boost terms stay live), and
  `Downed` (camera at the limit of its downed orbit). **A gate that fails OPEN is invisible in every
  other state**, because everywhere else the gate is supposed to be open anyway -- which is precisely
  why `0.26` survived in the inspector.
- **`travelOverride` is null by default and that default is load-bearing.** A preview state that says
  nothing about travel gets travel aligned to its nose, which is both the honest pose and the safe
  one. Disagreement now has to be stated explicitly. Do not invert this by making states declare
  agreement -- the whole point is that forgetting produces a correct row rather than a contradictory
  one.
- **The downed camera orbit rotates about `Vector3.up`, NEVER the craft up, and this is load-bearing.**
  The drive rig is bound `LockToTargetWithWorldUp`, so the follow offset lives in a frame that yaws
  with the chassis but never pitches or rolls with it. Rotating about world up therefore keeps the
  horizon level all the way around the orbit -- verified at 0.000 degrees of camera roll at 122
  degrees off-axis with the craft at 54 degrees of tilt. Using the craft up would roll the horizon
  upside down at exactly the moment the player is trying to read it.
- **`IntegrateDownedYaw` latches `IsDowned`; do not "simplify" it to read the flag directly.**
  `IsDowned` chatters -- measured dropping and re-engaging three times inside 0.9s of a 25 m/s
  wipeout, because bounces break the ground contact the lockout reads. Without `downedCameraHold` the
  camera is handed to the player and snatched back twice before the craft settles. `TuningLog.md` >
  The downed window.
- **`ContributeDownedYawOrbit` runs AFTER the boost terms and the shoulder shift, and BEFORE the
  guard.** It rotates the finished offset rather than adding to it, so everything else that displaced
  the camera orbits with it; and the guard still has to measure the pose that ships. It leaves
  `targetOffset` alone on purpose, which is what makes it a look-around rather than a pan off the craft.
- **`BuildPreviewInputs` assigns `travelSpeed`, `lookAheadDistance`, `forwardGate` and `downedYaw`
  once after its switch, not per case**, so a state added later cannot forget any of them. A per-case
  assignment is how one preview state sat silently wrong for five versions. Note a preview state that
  omits an input a GATE reads will silently show the ungated case, and it looks like a correct row
  rather than a missing one.
- **Every seed is live-editable and everything derived from it is recomputed per frame, not cached**,
  so dragging one during play takes effect immediately.
- **The framing budget cannot be tuned away.** The craft sits at the CENTRE of the drive camera's
  orbit, so the angle from camera down to craft is always exactly the elevation angle and no amount of
  distance changes it. **Elevation, minus how far the look axis tips, must stay under half the vertical
  FOV.** Two contributors spend that one budget independently and neither knows the other exists, which
  is why no pair of defaults can bound it and why the framing guard exists. Booming the camera back is
  a weak lever by comparison.
- Edit-mode writes are skipped when unchanged, which is a rounding error at runtime and the whole point
  in edit mode, where an unconditional per-tick write leaves the scene permanently dirty.

### `HoverCameraImpulseRouter.cs`

**Does.** Owns every camera punch. Subscribes to `Foundation.OnHardLanding`, `Propulsion.OnJumpDenied`,
`EMP.OnEmpFired` and `Weapons.OnWeaponFired`. Test hotkeys F9 landing, F10 crash, F11 denied jump
(+Shift for air), F12 EMP (+Shift for recoil).

**Constraints.**

- **Deliberately separate from `HoverCameraController`.** Impulse listeners run post-body on the vcams,
  downstream of the framing stage, so an impulse can never fight the per-frame writes. Framing needed
  the single-authority refactor because its contributors DID fight; shake never had that problem and
  folding the two together would import it.
- **Five sources, not one, because a source IS a channel.** Shape and duration are authored on the
  `CinemachineImpulseSource` and only the velocity vector is a per-call argument, so different
  characters need different sources. See trap 14 for why reconfiguring one source per shot is unsafe.
- **Every channel except the landing converts its world direction through `ToScreenSpace` first**,
  because the listeners read the vector as a screen direction (trap 11). The landing is deliberately
  left raw, because its raw behaviour is the punch the owner signed off on.
- **Recoil strength is authored per weapon** on `WeaponDefinition.combat.recoilVelocity`, defaulting to
  0 so it is opt-in. Authoring it on the router would mean a slot-index lookup that silently means a
  different weapon the moment the loadout is reordered.
- **Only Impulse Shape, Impulse Duration and the per-call velocity do anything.** The rest of the
  source inspector is dead (trap 12).
- Lives on the Camera object with all five sources as children, so `vehicleRoot` is a serialized field
  and crash detection needs `VehicleCollisionRelay`.
- **Three of five channels are unassigned ON PURPOSE, and this is a scoping decision rather than a
  defect.** EMP, weapon recoil and denied jump are nulled for the current playtest, in which the
  owner has disabled the shield, the EMP and every weapon except the machine gun to keep the session
  about movement. Formerly `TODO.md` 0.15. **Consequence: 2.2 and 2.15 cannot be judged in this
  build.**
- **The denied-jump null is intentional too, and that is settled.** It was held open for a while on
  the reasoning that a denied jump is a MOVEMENT cue rather than a weapon or energy one, so it might
  have travelled with the other two by accident. Owner, 2026-08-17: it did not, and it will not be
  evaluated before the pre-alpha 1 playtest. **Do not restore `jumpDeniedSource` to make 2.15
  judgeable early** — all three channels come back together when the abilities do.
- **The nulls are SCENE INSTANCE overrides; the prefab is fully wired.** `Prototype_Scene.unity`
  overrides all three to `{fileID: 0}` on the vehicle instance. **Reading the prefab to answer "is
  this hooked up" gives the wrong answer**, which cost a wrong conclusion on 2026-08-16 before the
  owner corrected it. The runtime `Awake` warnings are the reliable check. Same hazard family as
  3.16.

### `VehicleHUD.cs`

**Does.** Health, energy, ammo, missile lock fill, reticle, and the trick tracker at
`/Canvas/TrickTracker_TMP`. Event-driven except the lock fill and the trick counter, which are polled
because they change continuously and have no transition to hang an event on.

**Constraints.**

- **The trick tracker prints what the trick was WORTH, not what fitted in the pool**, with `(Full)` and
  `(EMP)` appended as marks when the grant fell short. The two causes are named separately because the
  player's response differs: a full pool means spend some, a freeze means wait.
- **The reticle is projected at a FIXED distance and must not be made to follow what the aim ray hits.**
  The chase camera sits behind and above the craft, so sliding the world point along the aim line sweeps
  it across the screen by parallax: measured, depth moved the crosshair 459px on a 1153px screen while
  the full range of aim travel moved it ~10px, a 45:1 ratio. Discarding depth was cheap because the rig
  binds `LockToTargetNoRoll`, so the camera pitches with the chassis and aim is already cancelled on
  screen by design. **Do not re-derive this by reasoning about the reticle alone; the binding mode is
  the whole explanation.**
- The aim DIRECTION still comes from the vehicle and is the same ray the guns get, so the crosshair and
  the shot cannot disagree. **What keeps terrain out of that ray is upstream, in Foundation's attitude
  reference**, not anything this file does.
- **`ReticleVisible` / `ReticleCanvasY` are the single source of the crosshair's screen position.**
  `MotionTrace` reads them rather than reprojecting the aim ray, because a second derivation could
  disagree with the game while looking authoritative. Check `ReticleVisible` first: it is false in
  drive mode and on every one of `SyncReticle`'s bail-outs, and a consumer that ignores it records a
  stale position as a live one.
- A missing `HoverController_Tricks` is deliberately not warned about, unlike the other modules, since
  an AI craft has none.

### Weapons support

| File | Does | Constraints |
|---|---|---|
| `WeaponDefinition.cs` | **The single source of all weapon tuning.** Nested sections mirroring `VehicleTuningProfile`: `combat`, `impact`, `flight`, `homing`, `blast`, `emitter`, `windUp`, `weaponLock` | Carries the project's only `OnValidate`, pushing the emitter section into every `ParticleWeaponCollision` referencing it. **Projectiles read the asset LIVE; particle emitters must be WRITTEN to**, because Unity reads particle settings off the component |
| `ProjectileSweep.cs` | Static. "Did this projectile pass through anything since the last physics step" | **Two details are load-bearing.** The sweep is *predictive* (FixedUpdate runs before the solver, so a retrospective sweep is permanently one step behind). The radius is inflated by 2x `Physics.defaultContactOffset`, because the solver parks a stopped body exactly that far clear. **Self-hits are filtered by IDENTITY, not by the layer matrix** |
| `WeaponImpact.cs` | Static. The single implementation of "split a knockback impulse between contact point and centre of mass" | Both delivery paths call it, so a particle hit and a rocket hit destabilize identically. Guards null rigidbody and a blast centred exactly on the COM (which would normalize to NaN) |
| `ParticleWeaponCollision.cs` | Particle hits, and configures the emitter from the definition | **Enforces the collision requirements** (`enabled`, World, `sendCollisionMessages`, `lifetimeLoss` 1) rather than documenting them, since those were silent failure modes. Derives `collidesWith` by writing the definition mask then stripping the firer's own layer. **Keep `destabilizeFraction` at 0 for sustained fire**; per-bullet torque accumulates every frame |
| `RocketProjectile.cs` | Dumbfire/homing rocket, splash with falloff | **Nothing is serialized except debug flags**; everything arrives from the `WeaponDefinition`, so editing during play retunes rockets in flight. Null-definition **destroys itself** rather than disabling, since a disabled projectile hangs in mid-air. **The flare is not a separate flight phase**: it always homes, at a point offset from the target that decays to zero, so convergence is guaranteed by construction. Steering lives in `FixedUpdate`, and the order (age, steering, hit detection, fuse) matters because the sweep's lookahead must be computed after steering sets the velocity |
| `EmpProjectile.cs` | Soft-homing single shot. No splash, no damage. Applies a freeze, or is absorbed by an active shield | Same `FixedUpdate` ordering constraint as the rocket. Freeze duration is pushed by `HoverController_EMP` so per-vehicle tuning owns it rather than the prefab |
| `TargetingScan.cs` | Static. "Find the best enemy in front of me right now" | Range is checked against real distance, not the sphere radius. Caller owns the buffer so it allocates nothing |
| `IProjectileOwner` / `IProjectileDefinitionCarrier` / `IProjectileDamageCarrier` / `IProjectileImpactCarrier` | Spawn-time push from `FireAllMuzzles`, all null-conditional so a prefab implements only what it needs | `IProjectileDefinitionCarrier` supersedes the two narrower pairs: handing over the whole asset means nothing is copied and nothing can drift. The older ones are still honoured so untouched prefabs keep working |
| `MissileFlareMode.cs` | Enum: `Alternate`, `Random`, `Left`/`Right`/`Up`/`Down` | Alternate and Random exist because a fixed direction reads as a scripted animation the second time you see it |
| `WeaponDebugDraw.cs` | Scene draws for the impact half: impulse split, splash attribution, particle contacts | Uses `Debug.DrawLine` with a duration, not `OnDrawGizmos`, because impacts are instantaneous and the rocket destroys itself on detonation. Every method is `[Conditional("UNITY_EDITOR")]` |

### Other vehicle modules

| File | Does | Constraints |
|---|---|---|
| `HoverController_Aim.cs` | LateUpdate only. Rotates the active slot's `vfxMount` | Reads actual vehicle pitch from the Rigidbody; no independent accumulation. Instantiated-mode weapons unaffected |
| `HoverController_Shield.cs` | Fixed-duration invulnerability burst | Absorbs one incoming EMP (shield deactivates, no freeze). Cannot be cancelled by the player. New activation blocked while already frozen |
| `HoverController_EMP.cs` | Fires an `EmpProjectile` after a cone acquisition | Energy paid once on activation, before the spawn. No target still fires, flying straight |
| `VehicleHealth.cs` | HP pool, `OnDamaged`/`OnDeath`, invulnerability, `Respawn()` | `Respawn()` has no callers and is incomplete (`TODO.md` 1.1); timed invulnerability can never be switched on (1.2) |
| `VehicleLayerAssigner.cs` | `[DefaultExecutionOrder(-20)]`. Assigns the hierarchy to `PlayerVehicle` / `AIVehicle` by input component | **This is what lets one shared prefab split into two collision identities at runtime.** Everything that strips "my own layer" from a mask depends on running after it |
| `VehicleCollisionRelay.cs` | Forwards `OnCollisionEnter` from the vehicle root to the impulse router | **Exists because Unity delivers collision callbacks only to the GameObject owning the Rigidbody**, never to children, and there is no subscribe API. **Deliberately holds no tuning and makes no decisions**: a relay that started filtering would be a second place to look |
| `HoverVehicleVFX.cs` | Three-tier cosmetic particle driver, side bursts on dodge, ground dust on hard landing | The dust prefab slots are empty, so the code path runs and produces nothing (`TODO.md` 2.4). **Everything the tiers vary is an EMISSION-TIME parameter**, so the rendered plume can never track its driving signal promptly and always trails by up to one particle lifetime. `emissionRate` is the only knob that reaches particles already in the air, and it is 40 at all three tiers, so it is doing nothing. Measured 2026-08-17; it is a requirement on the replacement effects, see `TODO.md` 2.14 |
| `EnemyHealthBar.cs` | World-space health bar, billboards in LateUpdate | Re-syncs in `OnEnable`, so it is respawn-safe. Currently the only thing that is |
| `PlayerHoverInput.cs` | Unity Input System reader | **Left stick is read as a Vector2 from ONE action**, deliberately: per-axis actions let Unity's normalization crush one component when both are deflected. Disables itself and logs if any action is missing. Rising edges are derived here, not in consumers |
| `AIHoverInput.cs` | Full `IHoverInputProvider` with a Roam / Flee / Dead FSM | Swapping this for `PlayerHoverInput` is the only change needed to make a craft AI-driven. **Two real gaps and several deferrals: `TODO.md` 1.6 and 3.3.** Doubles as the scripted-input rig for measurement |
| `HoverMath.cs` | `NormalizeAngle`, 0..360 to -180..180 | Unity reports -10 degrees as 350 and every attitude read needs the signed form |

### Instrumentation

All four live on the `/PerfDebug` scene object, so one object disables every instrument.
**Deliberately NOT wired to `HoverDebugSettings`:** that switch exists to turn off drawing that costs
frames, and these draw nothing, cost almost nothing while idle, and are worthless if they happen to be
off during the session you needed them for.

**`PlaytestSession` is the exception and sits on its own `/PlaytestSession` object**, so that killing
the instruments does not also kill the splash screen and the controls gate. It is session scaffolding
rather than an instrument: it measures nothing and exists to run a person through a session.

| File | Answers | Constraints |
|---|---|---|
| `FrameSpikeWatch.cs` | "Did a frame die, and what was happening?" | **Detection is nearly free, capture is expensive, because capture only runs on frames that were already ruined.** Detection is RELATIVE (a multiple of a running baseline AND over an absolute floor) because a fixed threshold cries wolf in one environment and stays silent in the other. **The baseline learns only from non-spike frames**, or a bad patch raises the bar until the tool goes quiet exactly when things are worst. Console logging defaults OFF as a measurement decision: logging once became a plausible source of the allocation it was reporting. **Its throttling verdict is broken, see `TODO.md` 4.5** |
| `AllocationBisect.cs` | "What allocates?" | Ablation, not theory: disables one MonoBehaviour at a time and ranks by what changed. **Read the FLOOR row first** (every candidate disabled at once); if allocation survives that, no per-component blame means anything. Craft must be PARKED. Reads the profiler's per-frame counter, not total-memory deltas |
| `MotionTrace.cs` | "Did what I SAW match what the physics did, and was it the craft or the camera?" | **Core metric is the residual**: drawn frame delta projected onto direction of travel, minus real speed. Sampled at `beginContextRendering`, not `LateUpdate`, so the camera is read after the brain has committed it. **Zero allocation in the hot path is load-bearing, not tidiness.** Buffer is bounded and stops when full rather than ringing, because the analysis wants one contiguous timeline. **All four sample paths must guard on `_full`, not just three** — `FixedUpdate` was missing it until 2026-08-20 and went on paying for a whole `FillShared` at 100Hz after recording had stopped. **It preallocates about 139 MB at `captureSeconds` 3000**, resident in builds too; see `TODO.md` > 4.4. **`M` drops a marker**, and it must stay a plain key off the F row (trap 30). **v1.4 adds the three aim angles** — `nose_elev`, `nose_vs_ground`, `ground_elev` — each measured directly rather than derived from the other two, plus `aim_cmd` and `reticle_y`. They are nose ELEVATIONS, not Euler pitch, because Euler pitch inverts the reader's intuition and is contaminated once the craft is banked. **`reticle_y` is READ from `VehicleHUD`, never recomputed**, and `reticle_valid` is false in drive mode so an empty column is obviously empty |
| `PlaytestReset.cs` | Escape hatch for a soft-lock or a fall outside the environment | Hold Select (Share) or `R` for 0.6s, read off the device directly so a debug utility never requires editing the shipped input asset. **Restores pose, velocity and ENERGY. Health, shield, ammo and weapon slot are still deliberately untouched**, since topping those up would make it a half-built respawn. **`restoreEnergy` defaults ON, and that is a playtest default, not a measurement one:** a refill is a step change in the pool that no spend explains, so UNTICK IT for any run whose energy trace matters. Owner's call 2026-08-18; the original restore-nothing behaviour is one tickbox away. **The camera must be told** via `NotifyVehicleWarped`. Interpolation is toggled off across the move or the Rigidbody draws the teleport as one streaked frame. Logs `Time.unscaledTime` so the fake speed spike can be discounted in the trace. **`ResetNow()` is the public entry point**, so the warp has exactly one implementation |
| `PlaytestSession.cs` | Runs one tester through one session: splash, session clock, and a controls overlay gated on elapsed play. **Not an instrument**, and lives on `/PlaytestSession` | **The gate is the point:** the blind period is 90s of PLAY (the clock starts when the splash is dismissed, not on load), so it is identical across testers by construction and the facilitator is out of the loop. **Builds its whole canvas in code**, so nothing can come half-wired; the accepted cost is that layout cannot be authored in the scene and only the two `[TextArea]` copy blocks are editable. **Every alpha is solved from state each frame and nothing else writes one** — an earlier draft set them at the phase transitions, and any path reaching a phase without its transition left an opaque splash over a live session. **Does not pause:** `timeScale` 0 risks a discontinuity in the `Time.unscaledTime` state Foundation and Propulsion carry. **Options both dismisses the splash and toggles the overlay**, because dismissing on any button hands that press to the game and starts the session with a jump. Hold `P` restarts for the next tester: craft home via `ResetNow()`, energy full, slot 0, gate re-locked. **Every phase change is stamped with `Time.unscaledTime`**, so one continuous `Player.log` cuts back into one session per tester, and a pre-unlock Options press is logged as a discoverability reading |

**`FrameSpikeWatch` and `MotionTrace` do not overlap and were designed not to.** The first fires only
past `minSpikeMs` against a ~3ms baseline and otherwise samples every 2 seconds, so the whole band
where ordinary jitter lives is invisible to it by construction. Run both together.

### Editor tooling

| File | Does | Constraints |
|---|---|---|
| `Editor/VehicleTuningProfileEditor.cs` | Custom inspector for `VehicleTuningProfile`. Twelve derived readouts anchored under the fields that feed them, nine invariant warnings | **Sections are drawn by ITERATING each nested container's children**, so a field added to a `*Tuning` class cannot be silently dropped. **Say what to CHANGE, not what the number is:** a readout is a decision, not a unit dump, and each block ends with "to get X, change Y and Z". **Distances print in craft lengths alongside metres**, because the arena is not at real-world scale while the craft is an on-screen ruler. **Do not put live figures in tooltips**: a readout recomputes and cannot go stale, a written number silently rots. `DerivedDrift` SOLVES rather than evaluates, and reports which of the two terms is binding |
| `Editor/TuningBaseline.cs` | Prefab-style override marking for ScriptableObject tuning assets: fields moved since a session baseline draw bold with their old value | **The baseline is a session snapshot, not the class defaults**, measured rather than assumed: `VTP_Default` differs from its own field initialisers in 40 of 93 fields, so defaults would bold half the inspector permanently. Snapshot lives in `SessionState`, so it dies when Unity closes, which is exactly a tuning session's lifetime |
| `Editor/WeaponDefinitionEditor.cs` | Custom inspector for `WeaponDefinition`, hiding sections the selected type does not read | **Every field is drawn by NAME, so a new field is invisible until added here too (trap 13).** That is the price of conditional display and cannot be traded away. **The free-heading trick survives nesting**: `[Header]` on the first field of a group, drawn by `PropertyField`, so a heading appears and disappears with its group. **Do not add explicit headings** |
| `Editor/HoverCameraControllerEditor.cs` | State selector plus the framing verdict readout | **Grades three ways, not two** (whole hull clear, centred but clipped, leaving frame), because clipping the rear underside at close range is ordinary framing and a check that fires during normal driving gets ignored. Fields drawn by plain top-level iteration |
| `Editor/HoverVehicleVFXEditor.cs` | Preview buttons pushing a single VFX tier onto continuous emitters in edit mode | |

---

## Inter-Module Wiring

- **Input.** All HoverController scripts acquire input via `GetComponent<IHoverInputProvider>()` in
  Awake. No inspector wiring. Swap the component to change who drives.
- **Foundation to Propulsion.** Propulsion reads Foundation's ground state each FixedUpdate to gate
  throttle, drag and drift.
- **Energy to Propulsion / Weapons / Shield.** Propulsion calls `TryConsume` per frame for boost and
  once for jump and dodge. Shield calls it once on activation. **Weapons spend no energy.**
- **Shield to Health.** `TakeDamage` short-circuits when the shield is active. The separate
  `_isInvulnerable` flag remains for post-respawn grace.
- **Energy to Shield.** `EmpProjectile.Consume` calls `Shield.TryAbsorbEmp`; if true the freeze is
  skipped. Shield also subscribes to the freeze event as a defensive fallback.
- **Weapons to projectiles (spawn-time push).** `FireAllMuzzles` hands the freshly instantiated prefab
  its optional interfaces in order, all null-conditional. **This is why weapon tuning lives on the
  definition and not on the prefab: the definition is the single author, the prefab is the delivery
  mechanism.**
- **Propulsion to Foundation (aim pitch).** Propulsion calls `SetAimPitch` every FixedUpdate, `(0, 0)`
  on strafe exit and EMP freeze. Foundation's leveling torque drives toward the target on the pitch
  axis only. **Propulsion never applies pitch torque itself. One attitude authority.**
- **Propulsion to Foundation (air control).** `SetAirControl(pitch, roll, weight)` every FixedUpdate;
  weight is air-control blend times `(1 - strafe blend)` so strafe aim wins airborne.
- **Propulsion to Foundation (jump charge).** `SetJumpCharge(fraction)` every FixedUpdate, and `0` on
  EMP freeze — load-bearing there, because the freeze returns from `FixedUpdate` above `HandleJump`.
  **One number, no verbs: Propulsion cannot lower the craft, only report a fraction**, so there is no
  path by which the jump could reach into ride height and change the launch it was tuned for.
- **Foundation to VFX / Camera (hard landing).** `OnHardLanding(severity)` fires on the
  airborne-to-grounded edge above `hardLandingMinSpeed`.
- **Propulsion / EMP / Weapons to the impulse router.** The bool on the jump denial separates two
  different failures: grounded means the charge was spent for nothing, air means the token survives and
  you may retry, so the air punch is the smaller of the two.
- **Foundation / Propulsion to Tricks to Energy to HUD.** Tricks is a pure reader that pays the pool
  through `Energy.Grant`.
- **Anything that teleports the craft to the camera.** `NotifyVehicleWarped(positionDelta)` must be
  called immediately after, or the camera damps toward the new position from its remembered old one and
  flies the length of the level. It takes a delta rather than a destination because **what needs
  clearing is Cinemachine's damping state**, not the proxy's position. Yaw is snapped there rather than
  slewed, because the rate limit that stops a flip whipping the orbit is exactly wrong for a teleport.
- **Foundation to Propulsion (downed lockout).** `IsDowned` is true while the craft is *touching* ground
  and tilted past the threshold, latched until upright. **`IsHoverGrounded` is not a substitute and gets
  it exactly backwards:** a craft on its flank finds no ground with its downward rays, while a craft
  hovering a wall never touches the surface. **Tilt alone is not a substitute either**, since a barrel
  roll passes the threshold every time and gating on tilt would cut authority mid-trick.

### Layers and the collision matrix

Named layers: Default (0), TransparentFX (1), Ignore Raycast (2), Water (4), UI (5), **PlayerVehicle
(6)**, **AIVehicle (7)**, **Projectile (8)**. Vehicles go on 6/7 at runtime; the four projectile prefabs
sit on 8 in the asset, root and every child.

**The matrix has exactly one row of exclusions, all on Projectile:** it ignores Projectile (a volley
must not detonate itself), Ignore Raycast (the sweep is a raycast, so honour the convention),
TransparentFX, Water and UI. Everything else collides.

**Projectile deliberately still collides with both vehicle layers.** Making the matrix do the
self-filtering would make the sweep ignore vehicles too, and projectiles would pass through their
targets. The matrix keeps direct hits working and identity does the filtering.

**Two per-vehicle masks are authored in the scene rather than derived and must stay mirrored:**
`HoverController_Weapons.lockTargetLayers` is 128 on the player and 64 on the AI. The AI's was 0 until
corrected, which would have made every missile lock fail silently.

### Debug settings

**`HoverDebugSettings` at `Assets/Data/HoverDebugSettings.asset` is the debug master switch, wired
everywhere:** 22 scene components plus 28 across the vehicle, missile and emitter prefabs. Every draw
site calls `IsEnabled(HoverDebugCategory.X)`; a component with no asset assigned falls back to its own
local `drawDebug` bool. **If a draw ever ignores the toggle, the field is unassigned on that
component.** Category map: Foundation to Recovery, Propulsion to Movement, Weapons/Aim/EMP to Weapons,
Energy/Health/Shield to Resources, AIHoverInput to AI, projectiles and emitters to Impacts.

**Gizmos cost real frames, which matters because a tuning pass judges feel.** Measured in one session:
all categories 5.22ms mean / 6.27ms p95; Movement only 3.70 / 4.69; master off 3.36 / 3.76. **The
"Movement only" preset recovers roughly 80% of the gap while keeping the readout you actually want.**
Physics is unaffected in all three cases, but frame *delivery* is not, and inconsistent delivery
corrupts exactly the judgement a tuning pass is making. **Turn the master off while assessing feel;
turn categories on while reading numbers.** `Measuring.md` has the full overlay reference.

---

## Systems Reference

### Energy

Shared non-damaging ability resource. Governs mobility and utility only, not weapons. Regenerates when
idle **and grounded**. Depleted by Boost, Jump, Shield and EMP, and paid back by landed tricks.

| Ability | Input | Cost | Contract (when it succeeds) |
|---|---|---|---|
| Boost | Hold | Continuous drain | Repositioning, not permanent speed |
| Jump | Tap / hold to charge | Scales with charge | Verticality meaningfully disrupts targeting |
| Shield | Tap | Flat | Timed, not spammed. Absorbs one incoming EMP |
| EMP | Tap | Most of the meter | Soft-homing, direct hit only. Denies tempo, not control |

### Weapons

All vehicles share the full roster. Vehicle identity is handling profile plus one unique special. All
weapons use limited pickup ammo except the Machine Gun, though **nothing enforces that yet**
(`TODO.md` 1.4).

| # | Weapon | Input | Status | Notes |
|---|---|---|---|---|
| 1 | Machine Gun | Hold | Implemented | Infinite ammo. Low DPS. Force, **not concussive**. `damage` 0.25 |
| 2 | Chain Gun | Hold | Implemented | Wind-up and wind-down. Force, not concussive. **No VFX prefab exists** (`TODO.md` 2.20), and `fireRate` sits at the `[Min]` floor of 0.01 (3.6) |
| 3 | Shotgun | Tap | Implemented | 30-pellet burst, `destabilizeFraction` 1, where spread geometry supplies the rotation. **Deliberate, not a violation of the flip rule** — see 5.16 |
| 4 | Rocket Launcher | Tap | Implemented | Straight line, no homing. Primary momentum disruption tool. **Force needs re-baselining against `topSpeed` 105** (5.16) |
| 5 | Lob Bomb | Tap | **Not built** | Rocket delivered over an arc. Identical to 4 except in trajectory for now. Needs 6.7 |
| 6 | Soft Homing | Tap | Implemented | Harassment tool. Low damage and force, small concussive blast |
| 7 | Hard Lock Missile | Hold to scan and lock, **release to fire** | Implemented, placeholder shape | Lock COMMITS to one target. Intended as a volley across one or several targets. Needs 6.8 |
| 8 | Hard Lock Chain Lightning | Hold to scan and lock, release | **Not built** | Near-instant damage instead of missiles. No blast, not concussive. Needs 6.8. **Answers the Sniper question** |
| 9 | Laser Cannon | Hold to charge, release | **Not built** | Sustained beam, pierces targets, force along the axis, no blast. Needs 6.11, **which is gated on 6.12** |
| 10 | Gravity Well / Repulsor Bomb | Tap to deploy | **Not built** | Lobbed deployable. One active; does not affect deployer. Needs 6.7 |
| 11 | Bouncing Disc Blade | Tap | **Not built** | Ricochets in 3D. **Impact rather than blast**, may destabilise. Needs 6.10 |
| 12 | Mine | Tap to deploy | **Not built** | Force and concussive blast. **Consolidated from two mines to one, 2026-08-21.** Needs 6.9, the cheapest of the jobs |
| — | Sniper | — | **Held, undecided** | Its one strong claim was reaching a runner, and the 5.5 speed rule gave that to the whole roster |
| — | Flamethrower | Hold | **Held, undecided** | Continuous shove through a wide particle cone. **Cheapest weapon in the roster, and 6.12 makes it the gate on the most expensive one** |

**The Special is a concept, not a slot.** Every vehicle gets a unique one; what they are is TBD, and
`5.7` (one vehicle profile) is why "per vehicle" cannot mean anything yet. Expected route is promotion:
build a weapon for general use, designate it a Special if it earns that.

**Force and concussive are the authored vocabulary.** Force is `impact.impactForce`; concussive is
`impact.destabilizeFraction`. **Explosive weapons may flip a vehicle; non-explosive weapons should not**
(owner rule, 2026-08-21). **Projectiles are faster than vehicles** and should catch a straight runner —
the boosted ceiling is 131 m/s and all missiles currently sit at 70 (5.5).

`WeaponType.Mine` and `TickMine` already exist, so 12 is much closer than the rest.

**Definition assets are `Assets/Data/WD_*.asset`.** All weapon tuning lives there; the prefabs and
emitters carry no tunable values. **Layer masks are authored once**, with the firer's own layer stripped
at runtime, which is what removed the per-vehicle mask duplication.

### Pickups

Ammo, Energy Cell, Health Repair. **`PickupManager` is unwritten** (`TODO.md` 1.5).

---

## Measurement Traps: index

**Full entries are in `Measuring.md`, along with the recipes, the build procedure and the debug overlay
reference.** Read this index before any measurement; open the file when one looks relevant. **Numbering
is stable and never reused.**

| # | |
|---|---|
| 1 | Settle before measuring, and confirm the state flag |
| 2 | Measure the state flag, not "level and quiet" |
| 3 | Never trust a zero you have not proven can be non-zero |
| 4 | Derivation cannot tell you whether code runs |
| 5 | Check the spawn point is not inside geometry |
| 6 | `eval` cannot see the first ~0.3s |
| 7 | A guard on a deferred caller guards the wrong moment |
| 8 | The editor does not tick while unfocused, so edit-mode writes land but nothing redraws |
| 9 | A capture can render before the brain has consumed the write |
| 10 | An A/B whose two runs traverse different ground measures the ground |
| 11 | A vector that "works" may be being reinterpreted, and an axis-aligned test case hides it |
| 12 | Most of the Cinemachine impulse source inspector is dead, and it silently accepts tuning |
| 13 | A new field on a `Weapon*Tuning` class is INVISIBLE until `WeaponDefinitionEditor` is told about it |
| 14 | A Cinemachine impulse reads shape and duration off the LIVE source for its whole life |
| 15 | `Quaternion.Angle` returns exactly 0 for the rotations a high-refresh frame contains |
| 16 | A residual comparing render against physics is invalid on the tick travel direction changes |
| 17 | A clean draw error does NOT mean a smooth image, and both must be reported |
| 18 | DISPROVED: the periodic frame hitch is not Adobe |
| 19 | A body-frame component is not its world-frame namesake |
| 20 | A threshold added for numerical safety can become a gate that disables the feature in its own headline case |
| 21 | A changed C# default does nothing to an object that already serialised the old one |
| 22 | A handling A/B that free-drives the terrain measures the terrain, convincingly |
| 23 | `ForceMode.VelocityChange` is invisible until the next physics step |
| 24 | An inspector edit to a ScriptableObject lives in memory, dirty, until something saves it |
| 25 | A formula that reproduces a known figure can still be wrong, and reproducing it is what makes it dangerous |
| 26 | A camera pose reconstructed from rig settings is a guess; read the live `Camera.main` |
| 27 | A quantity equivalent to another in the common case gets treated as interchangeable, and the special case is where the bug lives |
| 28 | When a defect inflates a headline number, every knob tuned against it reads as broken |
| 29 | A mode must be measured against the baseline it is supposed to differ from. Put the OFF row in the table |
| 30 | An instrument that fails silently costs the whole playtest, and healthy-looking output is not proof it worked |
| 31 | A safety clamp becomes a driver of the artifact it prevents, once its unstated precondition is violated |
| 32 | An A/B that fails to distinguish is not evidence of no effect; check the "on" arm turns the mechanism on |
| 33 | A 194ms frame is a camera bug that lives nowhere in the camera |
| 34 | `fixed_steps` at 33-34 is an arithmetic ceiling, not a fingerprint |
| 35 | A periodic background event will land inside your investigation and look exactly like a hit |
| 36 | When you recalibrate a threshold, grep for every other place that reads the same quantity |
| 37 | A premise recorded in the tracker is not evidence, even written confidently and citing code |
| 38 | A quantity that discards sign measures effort, not achievement, and will pay out for going nowhere |
| 39 | An instrument that collapses two causes into one label will confidently blame the wrong one |
| 40 | The memory floor is not a proxy for editor accumulation, and a restart can fix what no counter shows |
| 41 | A servo-driven quantity read from its COMMAND rather than its ACHIEVED value is wrong exactly during fast input |
| 42 | A recorded-but-never-run mechanism needs the third arm: the case that is fine AND shares an ingredient with the bad one |
| 43 | Reconstructing a loop from outside it is not measuring it; when the reconstruction disagrees with observed motion, the reconstruction is wrong |
| 44 | `ForceMode.Acceleration` on `AddTorque` is a direct angular acceleration and does NOT go through the inertia tensor. Calibrate units before theorising |
| 45 | A magnitude that discards direction describes the wrong event. Speed called a flip recovery a "fling" twice; displacement is the honest instrument |
| 46 | A test whose two outcomes predict the same number proves nothing and still returns one. State what the FAILING case would print before believing a verification |
| 47 | A velocity component read in the craft's own frame drifts when nothing acts on the craft, because the axis rotates. World frame for forces, local frame for behaviour |
| 48 | Frame timing read through `eval` measures the measurement. Remote-driving the editor cost a 3077ms frame; relative micro-benchmarks inside one eval survive, frame-level numbers do not |
| 49 | Only a constant workload separates "the game got heavier" from "the machine got weaker". `FrameSpikeWatch`'s benchmark hit 3.57ms against 0.30ms and invalidated the window |
| 50 | An instrument that does not report its sample count lets "no data" read as "no problem". Print the count that could have shown the failure next to the verdict |
| 51 | The asset file and the editor's copy drift apart BOTH ways. Read and write tuning through the editor while it is running, and do not assume a diff line is yours |

**Four checks in this project have fired during correct play** (the Movement gizmo's drive/drag warning,
the camera framing verdict, the dodge teleport warning, and `FrameSpikeWatch`'s throttling verdict).
**A check that cries wolf during good tuning gets ignored, which costs more than having no check at
all.** Weigh that before adding a fifth.

---

## Standing Decisions

**Do not relitigate these.** Each was investigated and closed; reopening one needs a new reason, not a
rediscovery of the thing that prompted it.

### Physics and handling

- **THE VEHICLE IS ALWAYS THE TURRET, AND AIM MODE STABILISES THE BASE RATHER THAN THE GUN.** Owner's
  decision 2026-08-19, and it settles a question the code had never answered. The muzzles are on the
  nose, the nose is where the shot comes from, in drive and in aim alike, and there is no roll. What
  aim mode is meant to feel like is a turret on a stable hovering base, the Warthog gunner. **So the
  hull levels to the horizon while aiming and nothing is decoupled from it.** Three things follow and
  each is a reason not to "improve" this later: there is no gimbal and no separate aim frame to keep
  in sync, the vfxMount versus muzzle-transform split stops mattering so all six weapons behave
  identically, and the strafe camera can keep inheriting hull pitch, which is what holds the reticle
  still on screen. **In drive mode the ground steers the gun and that is intended**, so a craft on a
  ramp shooting at a craft on the same ramp hits it. Implemented as
  `foundation.aimLevelsToHorizon`, weighted by strafe blend; `TuningLog.md` > Aiming on slopes.
- **THE CHARGE SQUAT EXPRESSES THE CHARGE. IT NEVER STORES IT.** Owner's decision 2026-08-17, closing
  the question `TODO.md` 0.25 was blocked on: the squat exists so ride height can be the charge meter,
  and it must add nothing to the tuned jump. **The springs will violate this by themselves unless
  stopped** — snapping the ride target back at release leaves the craft compressed under a target
  above it, and measured, that added 1.00m to a 20.59m jump (4.9%) at the instant the impulse fires.
  Hence the asymmetric blend: instant DOWN, because the height has to agree with the charge it
  reports, and rate-limited UP over `chargeSquatRelease` so the craft is clear of its own springs
  before the target catches up. **Nothing downstream of this — including the 2.13 discharge FX — may
  add impulse on release.** The honest cost is 1.30m off the charged apex, because the craft launches
  from the bottom of its own crouch; the inspector readouts subtract it so it cannot go stale.
- **SUPPORT IS KEYED TO THE AUTHORED RIDE HEIGHT, NOT THE TARGETED ONE**, and this looks backwards
  until you see the failure. `HoverSupport`'s height band used to move down with the target, so a
  craft whose target had just dropped read as being ABOVE its band while sitting closer to the ground
  than ever. Measured on a one-tick target step: support **exactly 0 for 0.2s**, degraded for 0.38s,
  which makes a grounded craft behave as airborne across regen, leveling, drag, the drift hop, fall
  gravity and air control. **The band asks whether there is ground close enough to be holding the
  craft up, and that answer cannot get worse by moving closer to the ground.** This was a latent
  ceiling-duck bug before the squat existed; driving into a tunnel at speed is the same step case.
- **AIMING DOES NOT MOVE THE CRAFT.** Decided 2026-08-17 across two fixes that turned out to be the
  same principle: in strafe mode the chassis pitch exists to aim, and nothing physical may read it
  as an intent to travel. Thrust and drag remove it (Propulsion), and the hover sensors are placed
  and pointed as though it were not there (Foundation). **The craft is a turret in pitch: it pivots
  about a mount held at hover height, and the mount does not care where the barrel points.** Roll is
  explicitly excluded and still reaches everything. Drive mode is untouched in both cases, measured
  to identity rather than assumed.
- **ONE EXCEPTION, and it is deliberate: the AIR JUMP.** It fires along `transform.up`, so aim pitch
  tilts it. Spotted from the scene view 2026-08-17 and left alone. Measured at `airJumpImpulse` 20:

  | aim | vertical | horizontal | direction |
  |---|---|---|---|
  | 20 deg nose UP | 18.79 | 6.84 | up and **BACKWARD** |
  | 0 | 20.00 | 0.00 | straight up |
  | 20 deg nose DOWN | 18.79 | 6.84 | up and **FORWARD** |
  | 36 deg nose DOWN | 16.18 | 11.76 | up and **FORWARD** |

  **The direction is the opposite of the intuition**, because pitching the nose down tilts the roof
  forward. **The grounded jump and the drift hop are NOT affected**; both fire along `Vector3.up`.
  Kept because the local-up axis is the same property that makes a wall jump most of the way built
  (and it is confirmed working, see the wall jump entry). **Do not file a
  sideways air jump in strafe as a regression against the rule above; it is outside it on purpose.**
- **The costs of that were accepted with it, not overlooked.** Aiming no longer produces any sense of
  the craft settling or leaning. The hull still tilts, so belly clearance still falls from 6.70m
  level to about 4.31m at 36 degrees, and **the nose can approach terrain the sensors no longer look
  at**, since they sit at un-aimed positions. If an unexplained scrape appears while aiming down over
  a rise, that is the cause and `TuningLog.md` records the rejected middle option.
- **`flipRecoveryDelay` is the flip punishment knob.** Raise it to make flipping hurt more; it is dead
  time, which reads as punishment. **Do not lower `flipRecoveryTorque`** for the same purpose, which
  just makes righting look sluggish, and **do not raise `flipRecoveryReleaseAngle` toward the arm
  angle**, which reintroduces the ~78 degree equilibrium stall.
- **`flipRecoveryDelay` IS THE ONLY KNOB THAT PRICES A FLIP, and that is now a design rule rather
  than a description.** Owner, 2026-08-20, setting the dead-man's timing: every other stage of getting
  the player back on their wheels happens as fast as it can be told apart from a recovery that is
  working, so lengthening or shortening the helpless period is one number. **Anything added to this
  area later must not introduce a second duration on the same subject.**
- **A HOVER SENSOR CANNOT TELL A WALL FROM A FLOOR, and that is the root of the wall soft lock rather
  than anything in the recovery code.** With the craft on its side the rays point horizontally, the
  springs hold it at hover height off the wall belly-first, and that pose IS the pose it is stuck in,
  so righting is not fighting an obstacle but a servo that re-corrects every tick. Measured frozen at
  111.37 degrees for thirteen seconds with authority held throughout. **Ablation settled it and the
  intuitive answer was wrong:** with leveling torque off it stalls at the identical angle, with the
  springs off the same righting torque finishes in 0.68s. **The leveling torque is not a contributor,
  so do not re-open it.** A contact-relative velocity for `isSlow` remains a real improvement on its
  own merits and is not needed for this.
- **The `strafeAccel / lateralDamp` relationship is NOT an invariant. Do not add a check for it.** Drag
  now damps lateral velocity minus intended strafe velocity, so it contributes exactly zero at the cap
  and the cap is reachable regardless of that ratio. This was flagged as a live invariant once and
  rejected on reading the code.
- **Dodge and unstick delta-v are `force x (duration + fixedDeltaTime) / 2`, not `force x duration / 2`.**
  Both compute progress and only THEN decrement the timer, so the first tick runs at full magnitude and
  the last runs at one timestep's worth rather than zero. At 100Hz that is ~6.7%. **Do not "simplify"
  either formula back.**
- **Air control rotation is predictable in closed form.** Steady rate is torque over damping; spin-up
  time constant is one over damping. **The non-obvious lever: scaling torque and damping TOGETHER by the
  same factor leaves the rotation rate identical and halves the spin-up time.** That is how "one flip and
  two barrel rolls, but the entry feels slow" was solved without changing the totals. `airRollTorque =
  2 x airPitchTorque` locks a clean 1 flip : 2 rolls relationship, and the shipped ratio sits under that
  deliberately so the flip keeps margin.
- **Small jumps are structurally floaty and gravity cannot fix it.** The springs catch you at
  `(liftDamping x v + gravityShare) / liftStrength` metres above ride height, so a slow fall is caught
  ~2.2m up and a fast one over 6.5m up. A tap jump spends roughly three quarters of its descent already
  cushioned. **That is the hovercraft's character rather than a defect.**
- **The flip arming clock DECAYS, it does not reset. Do not "simplify" it back to `flipTimer = 0f`.**
  The reset priced every interruption identically -- a one-frame blip and a 229ms excursion both threw
  away everything banked, and what pushes the craft out of the gate is its own settling tumble, so
  recovery was interrupting its own clock. `flipRecoveryProgressDecay` 1 is symmetric, so an
  interruption costs exactly its own duration and this can arm nothing the reset would not have armed
  eventually. Worth 0.78s on a 25 m/s wipeout and 2ms at rest.
- **The downed lockout and the righting torque run on DIFFERENT gates, and conflating them is the
  standard mistake.** Control is lost on contact plus `flipRecoveryAngleThreshold` (80) with no delay
  and no speed gate; it is returned at `flipRecoveryReleaseAngle` (35), mid-swing, with the craft
  still rotating. `flipRecoveryDelay` gates NEITHER -- it gates only the righting torque, and needs
  `flipRecoveryArmAngle` (70) and speed under `flipRecoverySpeedThreshold` (2) held continuously.
  Measured window from wipeout to control: **1.96s at rest, 4.84s carrying 25 m/s** (`TuningLog.md` >
  The downed window).
- **A flat-ground charge jump must never trigger a hard landing. A charge jump plus an air jump is
  allowed to.** Owner's rule, 2026-08-16, and it is a constraint on tuning rather than a description:
  it caps `extraFallGravity` at **43** against `hardLandingMinSpeed` 58, and the slider stops at 40 to
  keep a margin. At the shipped 35 the charge jump lands at 55.0 m/s, 3 clear. Numbers in
  `TuningLog.md` > Fall gravity and airtime.
- **`extraFallGravity` is a weak lever on airtime and a strong one on descent weight.** 30 to 40 is
  14% heavier on the way down and only ~3% less time in the air, because the rise runs on separate
  gravity and the fall shortens by a square root. **Do not reach for it to fix "too long in the
  air"**; that complaint wants presentation (`TODO.md` 2.11), and this has been the wrong lever twice.
- **What actually bounds fall gravity is the barrel-roll landing margin, not the hard landing.** Both
  hard-landing rules survive the entire usable range; the margin just degrades continuously at about
  **25ms per 5 units**, with no ceiling to trip. It is 0.09s at the shipped 35. **Anything that moves
  fall gravity, roll rate or jump impulse re-prices it**, so re-measure rather than assuming the last
  figure holds.
- **The banking contract: decided not to decouple.** At full bank the inertia magnitudes are unchanged
  but the basis rotates, the COM shifts 2.5cm, and a unit yaw command leaks -3.91% into pitch. Not
  decoupled, because the hull's own principal axes are already 0.81 degrees off its local axes and leak
  -3.46% of yaw into roll **at zero bank**, which decoupling would not fix.
- **The four rim mesh colliders stay. Do not propose removing them again.** They never touch anything at
  the clearance the craft runs at, and they did cause the drift flip, but the owner is keeping them
  deliberately: vehicle silhouettes are not decided and this is a useful awkward shape to test against.
- **Landing on top of another vehicle counts as floor**, so unstick pushes you off it. Explicitly
  accepted.
- **Ceiling duck cannot anticipate an overhang ahead.** The probe looks straight up, so entering a low
  tunnel at speed still hits the leading edge. Interiors are safe; **entrances want ~10m of clearance**,
  which is a cheap blockout rule. A real fix needs a velocity-based lookahead probe. Judged not worth
  worrying about yet.
- **Slope parking is declined.** Sliding is the honest hovercraft behaviour and the owner has felt no
  want for a hold.
- **Unifying strafe acceleration across all four directions is withdrawn.** It would trade away
  `maxReverseAccel` staying unblended so braking is identical in both modes.
- **The air jump is KEPT**, as a mid-air juke and trick-height extender.
- **WALL JUMPING WORKS, and it was never built.** Confirmed by the owner in play 2026-08-17: orient
  toward a wall, air jump, and you push off it. It falls out of the air jump firing along the craft's
  LOCAL up, so tilting toward a surface aims the shove away from it, and the hover rays cast along
  `-point.up` so a rolled craft genuinely puts rays on the wall and the springs push. **It is
  difficult to do, and that is accepted rather than a defect:** the owner classes it as advanced
  movement. Closed as `TODO.md` 5.10 without a line of code.
  **Two things this now constrains.** The air jump must keep firing along local up, and **un-aiming
  the hover sensors must stay PITCH ONLY**, because roll is the axis this depends on.
- **Airborne yaw is working, and the confusion about it is a coordinate frame.** `ApplyTurning` uses the
  craft's LOCAL up and is never scaled by air-control weight, so **air control does not suppress yaw, it
  rotates the axis yaw acts about.** Rolled 90 degrees, yaw input pitches you in world terms. That is why
  steering feels absent during a trick while measuring as fully live, and it is why the right stick is a
  good candidate for camera control in that state (`TODO.md` 0.13).

### Consciously accepted limitations

**These are known, measured, and left alone on purpose.** Each was reported, chased to a mechanism and
then accepted by the owner rather than fixed. **Reopening one needs a new play report, not a
rediscovery of the mechanism**, which is written down here precisely so it does not get rediscovered.

- **The strafe camera dips on a landing from a high, aimed jump, and that is accepted.** After the
  rate limit closed the two-frame snap (`TuningLog.md` > The landing snap in strafe), the camera still
  travels to roughly 2.1m below the craft before recovering. **The remaining motion is not the
  damping.** The strafe rig binds `LockToTargetNoRoll` and therefore inherits hull pitch by design,
  which is exactly what keeps the reticle still on screen, so a nose-up hull swings the authored
  `(0, 3.5, -6)` offset downward and backward. Measured at rest, aiming up 23.4 degrees, **the camera
  sits 0.03m above the craft rather than 3.5m.** Any pitch overshoot therefore moves the camera
  bodily, and the nose runs about ten degrees past its commanded aim across a landing. Owner,
  2026-08-19: *"it feels like a natural consequence to doing a higher altitude jump while aiming, like
  I'm coming down harder so it makes sense I can't hold my aim."*
  **If this is ever reopened, the thing to fix is the pitch overshoot, not the camera.** It is
  upstream of the camera motion and it is unexplained; tuning `verticalDampingSlew` against it would
  be tuning against a symptom.

### Judged good in the 2026-08-16 playtest

**These are acceptance criteria, not observations.** A change that degrades one of them is a
regression even if it improves something else.

- **Boost, drive mode.** Owner: "boost generally feels pretty good". The multiplier and all four
  shipped camera terms stand. **This withdraws the queued `boostBlendSeconds` 0.35 to 0.15 change**,
  which was contingent on boost still reading flat, and it did not. What remains is presentation
  only (`TODO.md` 2.11), which the owner independently reached the same conclusion about.
- **Speed and turning. PARTLY REOPENED 2026-08-20 by the pre-alpha 1 playtest — read the reopen before
  using this as a bound.** `topSpeed` 80, `strafeTopSpeed` 53, the yaw pair and the drift yaw
  multiplier were bounded by an owner feel report on 2026-08-16.
  **What the four-tester playtest changed, and it is a deliberate reopen rather than a contradiction:**
  two testers independently asked for steering authority to fade with speed, and a third said top speed
  arrives far too quickly. The entry was judged by one driver at one skill level, which is the new
  reason this file requires for reopening rather than a rediscovery of what prompted it.
  **CLOSED AND SUPERSEDED 2026-08-20 by the speed pass** (see > Judged good 2026-08-20, below). The
  reopen this entry describes ran its course: all three items were built, judged together in one
  drive, and closed. **The values below are historical from here on** — read the newer entry for what
  is live. What the sub-bullets are still good for is the reasoning about which things were
  deliberately NOT reopened.
  **Precisely what was reopened and what was not:**
  - **`topSpeed` 80 was treated as fixed for the duration of the reopen, and that turned out to be
    the wrong bound.** Nobody had said the craft was too fast, so 80 was held and the approach to it
    made free. **The feel pass established the opposite: the ceiling WAS the complaint** (`topSpeed`
    is now 105). The entry was not wrong about what the testers said — it was wrong to convert "no
    one asked for more speed" into "the ceiling is not the lever". **Absence of a request is not a
    bound.**
  - **The yaw pair was reopened** for a speed-dependent term, **and that term shipped and was judged
    good 2026-08-20** (`yawSpeedFade` plus `boostYawMultiplier`, re-eased by the owner in the closing
    drive). `yawAccel` 15 and `yawDamping` 8 themselves did NOT move — the fade multiplies the first
    and leaves both in the asset untouched, so this entry's original values still stand and only
    their effect at speed has changed.
  - **`driftYawMultiplier` and the drift behaviour are NOT reopened** and must not be reshaped as a
    side effect. Drift is separately judged good below and has its own equilibrium via `maxDriftAngle`.
  - **The strafe/drive RATIO stands; the numbers that express it have moved.** 53 against 80 became
    **70 against 105** in the speed pass, holding the ratio at 66.7%. The ratio is the judged-good
    thing and it survived; the absolute is derived from it and `topSpeed`.
- **Drift, as a held slide.** `driftLateralDamp`, `driftForwardDamp`, `driftYawMultiplier`,
  `driftSustainedTopSpeed` and the sustain/bleed pair are bounded. **The drift HOP is excluded and is
  was a separate complaint, closed 2026-08-17 as a suppression bug rather than a missing cue
  (`TuningLog.md` > The drift hop was being suppressed). Its VFX half is `TODO.md` 2.12.
- **The strafe-to-drive top speed ratio, ~66% in both boosted and unboosted form** (66.25% as judged,
  66.7% since the 2026-08-20 speed pass rescaled both numbers). Owner accepted
  it against the 5.0s continuous boost budget (`maxEnergy` 100 / `boostEnergyPerSecond` 20). Boost
  scales both ceilings by the same factor, so **there is only one ratio and it cannot be tuned per
  mode.** This bounds `TODO.md` 5.11 to the camping question alone.

Added 2026-08-17, judged the same day they were built:

- **The boost camera across a reversal**, `forwardGateSlew` 3.5. The owner drove the manoeuvre that
  produced the defect — boosting backwards and forwards — and accepted the result. **The engage cost
  was accepted, not merely tolerated:** the gate multiplies the overshoot as well as the sustained
  terms, so the lens sits up to 1.8 degrees narrower through the first quarter second from a
  standstill. **That softer first third is now the intended ramp shape.** Restoring the sharper one
  is a regression, not a fix, and re-introduces the 23ms step (`TuningLog.md` > The boost gate was a
  step against reversing).
- **The speed look-ahead rate limit**, `speedLookAheadSlew` 8, which closed 0.20. Chosen to be a
  provable no-op on flooring it from rest, so that case remains the reference the number is set
  against: if the accepted case ever changes, this bound has to be re-derived rather than kept.
- **Camera control while downed**, `downedYawSensitivity` 1.5, `downedYawRange` 90,
  `downedYawRecenterSpeed` 1.5, `downedCameraHold` 0.2 (the last two pairs are scene overrides).
  Owner: *"I tuned the values. it feels good."* **The sensitivity and recentre deliberately equal
  their camera-pitch twins and run the same arithmetic** -- that is what makes the two axes feel like
  one control, so moving one without the other is a regression in feel even if each reads fine alone.
- **`extraFallGravity` 35**, which closed 0.21. Owner: *"I can still hit two barrel rolls so as long
  as that and the ability to do one flip persists and air time went down by a little bit, that's
  good enough for me."* **This is the only entry in this section that ships with its own test.**
  Two rolls is the binding case — a flip is faster, so it fails second — which means **any future
  change that spends trick margin can be judged against the two-roll landing alone.** The margin is
  0.09s here, and the 50-degree roll tolerance no longer covers it (see Trick economy).

### Judged good in the pre-alpha 1 playtest, 2026-08-19

**FOUR TESTERS, NOT THE OWNER.** Every other acceptance criterion in this file was set by the owner
driving. These were set by four people who had never touched the game, two runs each, with the owner
watching. **That makes them the only entries here that are evidence about players rather than about
the owner's hands**, and it is why they are recorded as criteria rather than as anecdotes.

- **HOVER READS AS HOVER, and this is the strongest confirmation the project has.** All four reported
  the craft felt clearly hovering, and specifically that it was **manoeuvring with respect to the
  ground rather than flying or pushing off nothing**. That is the "Hover is persistent" pillar
  confirmed by strangers who were told nothing about it. It bounds ride height, the spring-damper
  tuning and the gravity feedforward as a system: **a change that makes the craft read as flying is a
  regression even if every number that produced it looks better in isolation.**
- **Drive mode.** Tester 4: basically perfect, and the controls very standard for this type of game.
  Judged by someone with genre expectations and nothing invested in this build.
- **THE TWIN-STICK SCHEME SURVIVES CONTACT, and the way it survived is the finding.** Players **tried
  to invent better control schemes and failed to produce one.** They reported the twin sticks as
  contextualized once they understood the drive/aim mode switch. **Individual binding confusion does
  not contradict this** (`TODO.md` 2.18 and 2.19 record all of it): the mapping of specific actions to
  specific buttons is open, the two-stick model with a held mode switch is not.
- **THE LEARNING CURVE, and this is what the session was really testing.** All four improved sharply
  between run one and run two. All four asked for a second run unprompted. By the second run they were
  landing tricks, timing jumps off ramps for more air, taking secondary air off a bounce and jumping
  out of a drift — **none of which anyone had explained beyond the basic mechanics.** All four
  independently described it as easy to pick up within fifteen minutes and hard to master, and could
  see real high-skill expression in it. They also watched the owner drive at a much higher level than
  their own first runs and read that gap as something to climb rather than as a wall.
  **This is the acceptance criterion that most easily gets traded away by accident**, because
  simplifying any single mechanic in response to a first-run complaint spends it. See `TODO.md` 5.15.
- **A FIRST IMPRESSION THAT REVERSED ITSELF IN THIRTY SECONDS, recorded as a standing caution rather
  than as a result.** Tester 4's first reaction was that steering felt too floaty. Within about thirty
  seconds of play they reversed it unprompted: because it is a hovercraft, the floatiness was
  contextualized and made sense. **They then called drive mode basically perfect.** The same session
  produced two separate high-speed complaints that did NOT resolve themselves (closed as 0.32; see
  > Judged good 2026-08-20).
  **The rule this supports: a complaint from the first thirty seconds of a first run is not yet data.**
  Give it a second run before it is written down as a defect, and weight the complaints that survive
  contact over the ones that arrive with it.

### Judged good 2026-08-20

- **THE SOFT LOCK IS CLOSED, judged by the owner driving both repro cases.** Owner: unable to get
  stuck in either. That closes `TODO.md` 0.27, the only item in the project that could end a session.
  **What this bounds is the whole dead-man design** (module constraints above): both halves of the
  fix, the movement-based detector, the faded restore, and the choice to say nothing to the player.
  Measured alongside the verdict, and these are the numbers a regression would move:

  | case | rescue fires | helpless for | rise above ride height |
  |---|---|---|---|
  | stuck against a wall | 2.18s | 2.80s | 2.16m |
  | carried at 4 and 8 m/s | 1.34s | 2.09s | 2.46 / 2.50m |
  | ordinary recovery, two entries | **never** | 1.80 / 1.95s | 1.54 / 1.76m |
  | five wipeouts, 20 to 35 m/s | **never on any** | 0.01s to 3.63s | 1.09 to 1.79m |

  **The acceptance criterion is the "never" column, not the fires column.** Both ordinary recovery
  paths are untouched and two of those wipeouts kept the craft down for over 3.4 seconds without the
  rescue looking at them twice. **A change that makes the dead-man fire during an ordinary recovery
  is a regression even if every soft lock stays closed.**
- **The rescue is SILENT and that is a decision, not an omission.** Owner's call, 2026-08-20, taken
  with the options on the table (HUD line, camera kick, rumble, sound). It reuses the ordinary
  righting motion, so from the seat it is the recovery taking a beat longer; announcing it would only
  tell the player a rescue system exists. **Do not add a cue for it without a play report asking.**
- **SPEED, ACCELERATION, STEERING AND THE LANE CHANGE, all judged in one drive.** Owner: *"genuinely
  better than the playtest build ... feels a lot more realistic now I guess? Still really good
  control but less arcade-y."* **This closes `0.33`, `0.32` and `0.36` together** and it is the pass
  the whole PRE-ALPHA 1 sequencing was built around. Live values: `topSpeed` **105**,
  `maxForwardAccel` 87 unchanged, `maxReverseAccel` and `reverseTopSpeed` **88**, `strafeTopSpeed`
  and `minDriftSpeed` **70**, `strafeAccel` **62**, `driftSustainedTopSpeed` **59**, `accelCurve`
  five keys, `yawSpeedFade` re-eased by the owner, `driveLateralPush` **10**. Derivation in
  `TuningLog.md` > The speed pass.
  **What this bounds, and it is a shape rather than a number: "less arcade-y" is the acceptance
  criterion.** It came from a ramp that tapers toward its ceiling, and the measured alternative — a
  flatter curve that is genuinely QUICKER to 80 m/s — was driven and rejected. **A change that makes
  the craft accelerate harder through the middle of its range is a regression against this even
  though it is faster.**
  **The camera followed the vehicle in the same pass, by the owner's own edit:**
  `speedLookAheadReference` 80 → **105** and `Lens.FarClipPlane` → **5000**, both as overrides in
  `Prototype_Scene.unity` and the `HoverCar_Prototype` variant (the base player-controller prefab
  still reads 80, so **read the live value through the editor, not the base prefab**). The reference
  is a DIVISOR, so tracking `topSpeed` costs look-ahead at every speed below 105 and buys camera
  response above 80; that trade was made in play and is part of what was judged.
  **Two values were deliberately left alone and each is the obvious next knob for a specific
  complaint**, so reach for the matching one rather than re-opening the set: `boostSpeedMultiplier`
  1.25 if boost reads flat, and a dedicated response term if the lane change reads mushy — **never
  `lateralDamp`**.

### Trick economy

- **Revolutions are COMPLETED, never accumulated travel**, and bank on the crossing, so a counter-rotation
  to straighten up for the landing can never take back a revolution already earned.
- **The corkscrew discount scales the payout, not the accrual**, and only outside a tolerance band.
- **Discounted payouts round to the nearest increment, never upward.**
- **Landing attitude is measured against the surface, not world up.**
- **Regen is grounded-only.** Airtime pays what you earn in it and nothing else.
- **Partial revolutions pay nothing, and this is self-consistent rather than harsh:** three quarters of a
  roll is 270 degrees of bank, which the arrival gate rejects, so a partial trick cannot produce a clean
  landing anyway. Either you roll out and get paid, or you unwind and land clean for nothing.
- **The landing roll tolerance and the airtime margin are two different gates on the same trick, and
  which one binds depends on fall gravity.** `trickMaxLandingRollAngle` 50 is worth about 0.116s of
  grace at the measured roll rate. While the two-roll airtime margin was 0.12s the tolerance covered
  it completely, so you could roll straight into the ground and pass. At `extraFallGravity` 35 the
  margin is 0.09s and **airtime binds alone.** The two numbers were never designed to match; they
  matched by accident and stopped. Expect the failure to read as "I finished the roll and it still
  didn't count".
- **The landing limits are measured, not guessed, and the asymmetry is real.** Roll goes downed at a
  much lower angle than pitch, because rolling puts the craft on its 3.8m width about its lowest-inertia
  axis so it keeps tipping, while pitching puts it on its 6.9m length so it settles. **A single limit
  would have been wrong.**

### Weapons

- **`turnRate` 160 is the floor for any weapon expected to connect.** Measured against a target 15m
  off-axis at 30m: 90 missed by 8.9m, 120 only clipped the blast edge, 160 was the first clean direct hit.
- **Missiles do not inherit vehicle velocity**, so the self-boost from a rocket jump scales with launch
  speed. That reads as skill but means the mechanic is strongest exactly when you are already fastest.
  If you would rather it were flat, measure falloff from the launch position instead of the detonation
  point.
- **Rocket-jump spin is safe at the current `destabilizeFraction`, but the margin is not large.**
  Axis-aligned self-blasts produce exactly zero rotation, because the contact point lands on the push
  axis through the COM. Only diagonal blasts tumble you. **Self-flips become reachable past roughly
  0.36.**
- **`selfImpactScale` 0 restores full exclusion** on any weapon that should not rocket-jump, since the
  impulse helper early-outs on zero magnitude. The firer never takes splash *damage* on any path.
- **The rocket jump is kept deliberately** rather than deleted, as GDD-aligned momentum expression.
- **`destabilizeFraction` saturation begins just above 0.3.** Inertia is roughly 3x easier to roll than
  to pitch.
- **Past roughly 10-14 rad/s the craft commits and one-sided hover lift drives it the rest of the way
  over**, which is the point of no return the tooltip describes. The relationship is not linear in tilt.
- **A hard landing does not produce an `OnCollisionEnter` at all.** The hover rays hold the chassis clear
  even with the springs given away. **Consequence for anything that wants ground contact: ask Foundation,
  not the physics callbacks.** Walls are different and do generate contacts.
- **Collision impact speed is about 0.6 of approach speed**, which is why the crash thresholds are
  authored in m/s and why `collisionMaxSpeed` is well below top speed. **`Collision.impulse`'s DIRECTION
  is deliberately unused**, since it runs from the first body to the second and depends on collider
  ordering; the averaged contact normal says the same thing without the ambiguity.
- **The arena is many separate mesh colliders, and the crash path depends on it.** `OnCollisionEnter`
  fires per collider PAIR, so hitting a wall while driving on the road is its own callback carrying only
  wall contacts. **Had the arena been one collider, no Enter would fire for the wall at all.** Recheck
  if the arena is ever re-authored as a single mesh.
- **`WD_Shotgun`'s stale `missileFireMode: 2` is left as-is.** Meaningless on a SingleShot weapon, never
  read, hidden by the inspector. It is the C# default landing in YAML after a rename.

### Presentation

- **SMAA is not optional.** A reported "camera jitter" was no antialiasing: measured, the camera holds
  the subject to within half a pixel, so there was nothing in the motion to fix. With no AA a
  high-contrast silhouette that is nearly STATIC in screen space crawls between pixel boundaries, and the
  eye locks onto the car precisely because it is the stationary thing. **This is a "clarity at high
  speed" failure where the aliasing was worst on the most important object on screen.**
- **DO NOT let that entry talk you out of suspecting the camera.** A second "camera jitter" report was
  genuinely the camera. **The separator is cheap:** the antialiasing one appears in a straight line at
  steady speed with the craft nearly motionless in frame and measures clean in every transform; the
  camera one appears after a stunt, on release or landing, and shows up immediately as heading-proxy
  divergence in a `MotionTrace` capture. Same three words from the owner, opposite conclusion.
- **The crosshair is projected at a FIXED distance.** See the `VehicleHUD` row.
- **Boost pull-back is drive-only and is applied along Z rather than by extending the orbit radius.**
  Straight back is both further out and slightly flatter, which shows more horizon and more ground
  streaming past, where booming along the radius would hold the angle and only add distance. **Strafe
  gets the lens and nothing else, because the strafe crosshair is yaw and pitch and moving the rig moves
  the player's aim.**
- **POSES BLEND, THEY DO NOT SNAP.** Owner rule, 2026-08-17, given when a snapped chassis bank was
  proposed as a drift-entry cue: everything on this craft eases into a new pose, and a snap on one
  axis reads as a different system rather than as emphasis. Applies to bank, attitude, framing and
  ride height. **An event that needs punctuation has to find it somewhere other than a discontinuity.**
- **Right-stick camera pitch is attenuated by steering effort.** Right stick X is yaw, so holding a drift
  line means holding X hard over, and no thumb pushes a stick perfectly horizontally. **Scaled on turn
  magnitude rather than on drift**, because the same bleed happens in any hard corner.

### Performance

- **Editor play mode is not a performance measurement.** Idle allocation is ~5 MB/s with zero game
  scripts running, and `eval` polling alone pushes it past 18. **Judge performance from a build**, where
  the same project measures 0.01-0.08 MB/s.
- **Game scripts allocate nothing measurable.** A full ablation with all 49 game components disabled
  measured allocation slightly HIGHER than with everything on. Three plausible theories about game code
  were each disproved before anyone thought to measure the floor, and **the floor was the answer.**
- **The periodic frame hitch was the editor's garbage collection and does not survive a build.**
  Roughly a hundredfold drop in allocation; 99.98% of frames under 8ms.
- **The profiler's memory counters DO return real data in a non-development player**, so there is no
  reason to reach for a development build and contaminate an allocation measurement with the profiler's
  own allocation.
- **Background applications caused the "choppy" sessions.** Closing NitroSense and Snipping Tool cut
  spikes and big stalls by 78-79%. NitroSense polls hardware sensors continuously and those reads block.
- **Six vehicles is not a throughput problem, but the TAIL is the story.** Mean moves modestly while p90
  and p99 roughly double and frames over 8ms go from 0.1% to 7.5%. **That matters here more than
  elsewhere because this project's felt problem has consistently been consistency, not throughput.**
  Physics is not the constraint. The cost is main-thread per-frame work, and particles scale with it.
  **Measured with AI driving but no combat**, so particle collision at scale remains unmeasured.
- **When the editor degrades and the counters disagree, restart before investigating.** A session
  degraded badly while every counter said the editor was healthy, including an identical memory floor
  across two days. Restarting fixed it completely. **Thirty seconds against an afternoon, and the
  counters have now been shown not to cover the failure.**

---

## Disproved by measurement. Do not re-investigate

**Read this before investigating any symptom that sounds familiar.** It exists specifically to stop a
session being spent re-deriving a dead end.

- **Environment SCALE as the cause of micro-bumpiness.** Uniform scaling leaves every angle identical, so
  doubling the world changed how OFTEN facets arrive and not how large each one is. The owner still felt
  it after reverting to 1x, which is the cleanest possible confirmation.
- **Camera damping as the cause of the same.** The report covers DRIVE mode, which no camera change that
  session touched.
- **The Recovery debug category** as a frame-rate cost or an allocation source.
- **Stuck weapon emitters** as a frame-rate cost. Four of them measured 0.14ms, because shipped emission
  rates are only 8-20/sec.
- **Flip recovery** as the trigger for choppy sessions. Across 15 recoveries, spikes land at every
  distance from one, from 0.0s to 27.6s.
- **Game scripts** as the source of idle allocation.
- **Belly contact** as a drift-flip cause. Clearance is 6.30-6.70m at *every* bank angle.
- **Bank shifting the centre of mass** (2.5cm, negligible), **bank changing inertia magnitudes**
  (unchanged), and **gyroscopic roll coupling from faster yaw**.
- **`flipRecoverySpeedThreshold`** as the flip-recovery cause. Recovery was already armed the whole time;
  `authorized True` in the gizmo was the tell.
- **Airborne air control** as the drift-flip cause. The flip reproduces on a flat plane with **zero
  airborne ticks**.
- **Self-collision** as the "bullets eaten" cause. The emitter's own layer bit is never set in any mask.
  Collision quality is not a factor either; all six emitters are verified correct.
- **`CollisionDetectionMode`** as the projectile-detonation cause. All four modes measured silent at
  70 m/s, so CCD was not doing its job.
- **Camera MOTION** as the cause of perceived subject jitter. 0.43px std on a straight run, 0.42px
  peak-to-peak parked.
- **The speed look-ahead injecting velocity noise into the aim.** At speed it sits pinned and saturated,
  contributing nothing.
- **Chassis bank oscillation** as a jitter source. 0.0000 degrees peak-to-peak parked.
- **Script execution order between the camera and the brain.** The proxy is written before it is read.
- **The framing guard** as the cause of the boosted-flip lurch, killed by a clean A/B with the guard
  provably disabled. **See trap 32 for why "provably" matters.**
- **Frame time, the heading proxy, and gimbal / euler decomposition jumps** as causes of that same lurch.
  The proxy moves exactly 0.000 degrees while air control is held; zero single-frame yaw jumps over 15
  degrees exist in the run.
- **The convex hull on `car.fbx`** as the cause of unattributed floor contacts. They were hard landings.
- **THE CRAFT BEING FLUNG OR LAUNCHED BY A FLIP RECOVERY. It is not, it never has been, and this has
  now cost two sessions.** Corrected by the owner on play evidence on 2026-08-17 and again on
  2026-08-20, both times against an assistant's confident number. A recovery moves the craft **0.15m
  horizontally** while it rises about 5m from lying on its hull back to ride height, so a speed
  readout peaks near 20 m/s describing a craft that goes straight up and stays where it fell. Owner,
  both times: it flips over basically right where it is. **The error is the instrument, not the
  tuning** — speed magnitude cannot separate standing back up from being thrown, so measure
  displacement (`Measuring.md` trap 45). The wording that kept regenerating this lived in a code
  comment and has been corrected at source. **A genuine launch on recovery would be new behaviour and
  a real defect**, so do not dismiss one; just prove it with displacement before naming it.

**One camera theory is UNPROVEN rather than disproved, and the distinction matters:** `SmartUpdate` on
the brain against a rig whose Follow is a plain script-driven Transform and whose LookAt is an
interpolated Rigidbody. The argument is plausible, those two want different update slots, but the A/B run
was invalidated (trap 10) and the antialiasing fix removed the symptom before it was retested. **If a
timing-shaped camera artefact ever appears, this is the first thing to test properly**, on a controlled
flat straight-line run.

---

## Workspace Conventions

- Core logic: `Assets/Scripts/*.cs`
- Data-driven values: `Assets/Data/` (ScriptableObjects)
- Editor-only code: `Assets/Scripts/Editor/`

**`Assets/RVP/Scripts/Editors/` is a trap: Unity's magic folder name is `Editor`, singular, so `Editors`
is an ORDINARY folder and everything in it compiles into the runtime assembly.** All twelve RVP files
there survive only because each is wrapped in `#if UNITY_EDITOR` from line 1 to the last line, which is
why the project builds at all. **A new file dropped in that folder without the guard breaks the player
build and the editor will not say a word**, because the editor compiles `UnityEditor` fine. There is no
asmdef anywhere in the project to catch it either. Put new editor code in `Assets/Scripts/Editor/` and
this never comes up.

---

## Prototype Checklist

- [x] Project, repo, IDE integration
- [x] Hovercraft movement & physics, judged good 2026-08-14
- [x] Camera and input control
- [x] Energy + ability system
- [ ] Basic combat loop (primary fire + pickups)
- [~] AI opponent: drives, roams, flees and shoots; cannot yet threaten the player

**This list is a flat inventory. `ROADMAP.md` is what orders it.** What remains on the last two items,
and why, is in `TODO.md` Tier 1.
