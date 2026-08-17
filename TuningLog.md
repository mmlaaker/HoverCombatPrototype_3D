# Tuning Log

**How a number was arrived at.** Sweeps, measured tables, rejected attempts, and the reasoning behind
values that are now shipped and judged good.

**This file is not loaded by default and is not read to find work.** It is opened by name when a
specific value is being re-tuned, or when someone is about to re-derive something that was already
measured. If you are looking for what to do next, that is `TODO.md`.

The split across the documentation:

| Question | Document |
|---|---|
| What is not done? | `TODO.md` |
| What exists, how does it work, why is it built that way? | `CLAUDE.md` |
| **How was this number arrived at, and what was rejected on the way?** | **`TuningLog.md`** (this file) |
| How do I measure this without fooling myself? | `Measuring.md` |
| Which unfinished things gate the next milestone? | `ROADMAP.md` |
| What should this be? Why? | `GameDesignDocument.md` |
| Physics derivations, frozen at `4a34f21` | `PhysicsAudit.md` |

**Two rules for reading anything below.**

1. **Every number here is dated and may already be stale.** Live values come from
   `Assets/Data/VTP_Default.asset` and `Assets/Data/WD_*.asset`, never from this file. What is
   durable here is the SHAPE of each result: which knob moved what, what was rejected, and which
   comparison was the one that mattered.
2. **Item numbers such as M.5 or 2.10 are historical.** They refer to `TODO.md` entries that were
   closed and removed. They are kept so the trail from a commit message or an old note still leads
   somewhere.

---

## Movement

### Speed and cornering
*Closed as M.5, shipped 2026-08-12, judged good 2026-08-13: "this feels much better."*

Direction was set by the owner: raise top speed, lower the boost multiplier. 1.5x is high as boost
multipliers go (most racing games sit nearer 1.15x to 1.3x) and 50% already read as merely okay,
which is close to proof that more multiplier was not the lever.

| Value | Was | Shipped |
|---|---|---|
| `topSpeed` | 60 | **80** |
| `maxForwardAccel` | 65 | **87** |
| `reverseTopSpeed` | 50 | **67** |
| `maxReverseAccel` | 50 | **67** |
| `strafeTopSpeed` | 40 | **53** |
| `minDriftSpeed` | 40 | **53** |
| `strafeAccel` | 35 | **47** |
| `dodgeForce` | 650 | **900** |
| `boostSpeedMultiplier` | 1.5 | **1.25** |
| `boostAccelMultiplier` | 1.5 | unchanged |
| `yawAccel` | 13 | **15** (measured at 16, trimmed by the owner after driving) |
| `speedLookAheadReference` (scene) | 60 | **80** |

**Damping was deliberately NOT scaled.** `forwardDamp`, `lateralDamp` and `yawDamping` are inverse
time constants, so the coast-down and forward-to-reverse ratios the owner had already confirmed
survive the change untouched. `hardLandingMinSpeed` / `hardLandingMaxSpeed` were left alone too,
since landing impact comes from gravity and drop height rather than from top speed.

**Live caps, verified with the craft pinned so terrain could not contaminate the read:** drive 80,
drive+boost 100, reverse 67, strafe 53/53, strafe+boost 66.3/66.3. **None of these are visible in
the inspector**, which is why they are worth recording.

**`speedLookAheadReference` is a camera value pinned to a vehicle value with nothing enforcing the
link.** It was 60, which WAS `topSpeed`. Raising top speed without raising it saturates the camera's
speed look-ahead before you reach top speed, so the camera stops responding to your fastest driving.
Highest-risk coupling in this cluster; check it first if `topSpeed` ever moves again.

#### The turn-radius model was wrong, and the corner had to be measured

**Do not size `yawAccel` from `radius = topSpeed / (yawAccel / yawDamping)`.** The general form of
this mistake is `CLAUDE.md` trap 25. Measured sweep at `topSpeed` 80:

| `yawAccel` | Radius | Corner speed | Slip angle |
|---|---|---|---|
| 13 | 36.7m | 59.1 m/s | 57.4 deg |
| **16** | **25.4m** | 50.3 m/s | 61.9 deg |
| 19.5 | 17.1m | 41.5 m/s | 68.0 deg |
| 24 | 10.3m | 30.7 m/s | 74.7 deg |

The formula prescribed 19.5, which would have produced a corner a third tighter than the one the
owner had confirmed as good, because it assumes cornering speed holds constant while it actually
falls from 59 to 41 m/s across that range.

Apples-to-apples against the committed baseline, same pinned start and route:

| | Radius | Corner speed | Yaw rate | Slip |
|---|---|---|---|---|
| Original, 60 / 65 / 13 | 26.6m | 42.5 m/s | 92 deg/s | 57.7 deg |
| Shipped, 80 / 87 / 16 | **25.6m** | **50.6 m/s** | 113 deg/s | **60.5 deg** |

Radius within 4%, slip within 3 degrees, 19% more speed carried through the corner. Same corner,
faster craft, which was the intended outcome.

**Incidental finding worth keeping:** ordinary full-lock cornering already runs ~58 degrees of slip.
The craft is substantially sideways in every hard corner by design.

---

### Fall gravity and airtime
*Closed as M.6, `extraFallGravity` 13 -> 30, judged good 2026-08-14.*

**The headline killed the framing the work started with: gravity is a WEAK lever on hang time, and
the fall was never slow.** Dropped from 260m:

| Time | Descent | Measured accel |
|---|---|---|
| 0.5s | 26.0 m/s | 50.6 |
| 1.0s | 52.6 m/s | 52.3 |
| 2.0s | 104.9 m/s | 52.1 |
| 3.0s | 157.1 m/s | 53.9 |

Rock steady at **52.2 m/s^2, 5.3x Earth gravity**, with `maxLinearVelocity` effectively infinite and
zero drag, so the craft already accelerates without limit. **Fall time scales as the square root of
height over gravity**, so a 130% gravity increase buys only ~13% less airtime on an ~86m drop. **Do
not expect large hang-time changes from this knob at any value.**

Charge-jump measurements, flat ground, pinned starts, `jumpImpulseMax` 40:

| `extraFallGravity` | Apex | Airtime | Two rolls at | Margin |
|---|---|---|---|---|
| 13 (was) | 21.0m | 1.93s | 1.67s | 0.26s |
| **30 (shipped)** | 21.0m | 1.79s | 1.67s | **0.12s** |
| 45 | 20.9m | 1.74s | 1.67s | 0.07s |

**Apex never moves**, confirming fall gravity does not touch jump height (rise and fall run on
separate gravity). **Two rolls take 1.67s regardless**, confirming roll rate is independent of it.
One flip is slightly faster than two rolls, so two rolls is the binding case and is what was measured.

**Recorded disagreement, and it is the first thing to suspect if landing tricks starts feeling
inconsistent.** The assistant's position was that 0.12s is too tight (roughly 7 frames at 60fps
against typical human input variance of 50-150ms) and recommended 20, giving ~0.18s. The owner tested
30 directly and found the trick still lands. Direct play beat the estimate.

**Hard-landing ceilings, measured:** a flat-ground charge jump stays under `hardLandingMinSpeed` 58
up to `extraFallGravity` **43**; a charge jump plus air jump (~25.5m) up to **27**. So at the shipped
30, a stacked jump can hard-land. The owner had not decided whether a stacked jump counts as
"normal"; if it should not hard-land, the ceiling is 27. Hard landings off ramps and ledges are
explicitly welcome, since the effects are severity-scaled.

**Left unjudged:** whether the shorter fall is felt at all. ~13% on a big drop may be below
perception. If it is not felt, the cause is likely perceptual rather than physical (the doubled
environment halved apparent speed) and the lever is speed lines rather than more gravity.

---

### Aiming and the hover sensors
*Closed as 0.22, shipped 2026-08-17. Owner: "this seems like a good change."*

Reported as: raising `strafePitchLimit` to move the reticle up "breaks functionality" past some
value, and even at 35 "there's this weird gentle rubber banding up and down for a bit" when
strafing forward or backward at full aim.

**Nothing was compensating for aim pitch anywhere.** That is the whole finding. Four independent
defects fell out of one omission, and each was measured before it was touched.

**1. Thrust followed the nose.** Drive applied force along raw `transform.forward`, so in strafe
mode the chassis pitch that exists to AIM was also steering the engine.

| | accel | vertical component at 36 deg |
|---|---|---|
| forward, strafe-blended | `strafeAccel` 47 | 27.6 m/s^2 |
| **reverse, NOT strafe-blended** | `maxReverseAccel` 67 | **39.4 m/s^2** |
| grounded gravity | | 39.24 m/s^2 |

Reverse binds, because `maxReverseAccel` doubles as the brake and is deliberately never softened
for strafe. `67 sin(t) = 39.24` solves at **t = 35.85 degrees**, against a shipped
`strafePitchLimit` of 36. The owner found the edge at 35. Below that threshold it does not fly, it
oscillates: the spring clamps at zero and cannot pull down, so the craft rises, the spring bottoms
out, gravity returns it. `liftStrength` 16 against `liftDamping` 2.2 is a damping ratio of 0.275,
underdamped, ~1.6s period, decaying over about a second. That is the reported rubber banding.

**Fixed** by removing the commanded aim pitch from the drive axis, weighted by strafe blend.
Terrain pitch is deliberately preserved, so strafing up a ramp still thrusts along the ramp.
Verified: `driveAxis.y` is 0.0000 at every aim angle in strafe and equal to `forward.y` to four
decimals in drive.

**Forward drag had to move with it.** Full drag arrives at throttle 0, so fixing only the thrust
relocates the oscillation from the press to the release.

**2, 3 and 4 were all in the hover sensors**, which are bolted to the hull and tilt with it.
Measured on flat ground, settled:

| aim | vertical ride height | belly clearance | rear springs | sensors sample ahead |
|---|---|---|---|---|
| 0 | 7.00 m | 6.70 m | carrying | 0.00 m |
| 20 | 6.58 m | 4.87 m | **UNLOADED** | 2.32 m |
| 30 | 6.06 m | 3.72 m | **UNLOADED** | 3.69 m |
| 37 | 5.66 m | 2.97 m | **UNLOADED** | 4.81 m |

- **It measured a slant, not a height**, so the craft read itself as too high and settled 1.34m
  lower at 36 degrees.
- **`HoverSupport` read ZERO while parked and aiming.** Same distance, compared against a
  `supportMargin` band ending at 7.75 while `avgDistance` sat at 8.65. Leveling, drag and energy
  regen all scale by support; air control and fall gravity scale by `(1 - support)`. The craft was
  aiming in something close to flight mode while sitting on the ground. **This was almost certainly
  the larger half of the felt problem and nobody had looked for it.**
- **The front sensors looked past the nose**, 4.81m ahead at 37 degrees against a nose tip only
  4.17m forward of the origin.
- **The rear pair carried nothing from about 20 degrees.** Tilting lifts them above ride height and
  a spring only pushes. The craft balanced on two forward-looking probes with nothing behind to
  steady it, which is the "thump when you start moving" the owner reported next.

**Shipped fix:** the sensors are placed, pointed and loaded as though the craft were not aiming.
Verified at 0, 20, 37 and 40 degrees: sensor height spread 0.000, ray direction `.y` -1.0000,
sample offset 0.00m, and un-aim angle 0.0000 in drive mode.

**Two attempts were rejected on the way, and both are worth not repeating.**

- **Sizing the correction from the COMMANDED aim angle.** Shipped, and the owner immediately
  reported "a really big overcorrection going full stick up and full stick down". The chassis is a
  torque servo and lags the stick: command reached 36 while the hull was near 12, so the correction
  was sized for a tilt that did not exist. Measured error and its consequence:

  | commanded | actual hull | gap it computed | true gap | phantom lift |
  |---|---|---|---|---|
  | 36 | 12 | 5.79 m | 7.00 m | **+19.4 m/s^2** (gravity is 39.24) |
  | 0 | 20 | 7.45 m | 7.00 m | -7.2 m/s^2, springs dropping out |

  Up on the way out, down on the way back, which is exactly what both stick extremes produced.
  **Measure the achieved attitude, never the command.** Generalised as `Measuring.md` trap 41.

- **Option A: straighten the rays but leave the sensors at their real tilted positions.** Offered and
  declined by the owner in favour of the full version. It removes the forward sampling but leaves the
  rear pair unloaded, so the craft still balances on two points. **Kept on record because it is the
  middle ground if the accepted cost below ever bites.**

**Accepted costs, agreed rather than discovered.** Aiming no longer produces any sense of the craft
settling or leaning. The hull still tilts, so clearance still falls to about 4.31m at 36 degrees,
and **the nose can approach terrain the sensors no longer look at.** An unexplained scrape while
aiming down over a rise is that, and option A is the answer.

**One limit disappeared.** The interim version needed `hoverHeight / cos(aim)` of sensor reach,
capping aim at `acos(7 / 9.5)` = **42.5 degrees**. Vertical rays need ~7m at any angle, so that
ceiling is gone and `strafePitchLimit` is no longer bounded by sensor range.

**Shipped value: `strafePitchLimit` 36 -> 20.** Set by the owner after the fix, and the direction is
the interesting part: **20 now puts the reticle higher than 36 did before.** Consistent with the
springs having been eating the aim angle, since the old front-compressed/rear-unloaded state
generated a nose-up torque the aim servo had to overpower every frame. Hypothesis, not measured.

---

### Drift
*Closed as M.7, shipped and judged good 2026-08-13: "this really feels like what I was going for."*

Cause and the three-knob tuning model are in `CLAUDE.md` (drift speed runaway, Propulsion v1.9). What
follows is the tuning record only.

Target named by the owner: **the Crash Team Racing power slide, without the boost mechanics.**
Measured on flat ground at full throttle and full lock:

| | before | first attempt (rejected) | **shipped** |
|---|---|---|---|
| settled drift speed | 128.4 m/s | 62 -> 44.4 | **51 -> 44.2** |
| corner radius | 337 m | 27 -> 19.2 m | **26 -> 22.8 m** |
| slide angle | 51.7 deg | **29.5 deg** | **52 deg** |

`driftLateralDamp` 0.3 -> 1.5, `driftYawMultiplier` 1.5 -> **2.5**, `maxDriftAngle` 60 -> **90**, plus
three new tunables: `driftSustainSeconds` 1.5, `driftBleedSeconds` 2.5, `driftSustainedTopSpeed` 45.
The owner then took `driftLateralDamp` down to **0.3** for a much freer slide, which is the shipped
value.

**The rejected first attempt is the instructive part.** It bought the tight corner with
`driftLateralDamp` 4. `lateralDamp` is 1, so that gave drift FOUR TIMES the sideways grip of ordinary
driving: a grip-assist button, not a slide. The owner drove it and reported "no drift or slide here"
within minutes. **The disqualifying evidence was already in the regression table and went unread:
ordinary full-lock cornering slides -61.5 degrees at 29.2m.** A 29.5 degree drift slid LESS than
simply yanking the stick. The general rule is `CLAUDE.md` trap 29: put the OFF row in the results
table.

The fix was `maxDriftAngle`, the one knob that had not been swept. It throttles turn rate through the
authority fade, so raising it tightens the corner AND widens the slide at once, where the other two
trade one against the other.

**Live constraint at the shipped `driftLateralDamp` 0.3, and this one still binds.** The slide is
still WIDENING when the drift times out: it ran -67.8 / -72 / -79 / -83 / -87 degrees across four
seconds and never reached equilibrium. `maxDriftAngle` 90 is what bounds it, and it is asymptoting
hard against that ceiling with roughly 3% of yaw authority left. **The 4s timeout is doing real work
containing it, so lengthening `driftBleedSeconds` or raising `maxDriftAngle` at this damp is the
combination to be careful with.**

Other properties worth knowing before touching drift:

- **The corner tightens as the drift bleeds** (42m on entry, 26m through the free window, 22.8m fully
  bled). Radius is speed over turn rate and turn rate is speed-independent, so this falls out of the
  speed bleed rather than being tuned separately.
- **The speed floor is deliberately below `minDriftSpeed`**, so holding a drift to the end drops you
  under the speed needed to start another. You must rebuild before you can drift again.
- **Drift ends when it bleeds out** (`OnDriftSpent`). Without the latch, throttle carries the craft
  back over `minDriftSpeed` in well under a second and the drift chatters against its own floor.
- **`minDriftSpeed` MATCHES `strafeTopSpeed` by hand** (both 53). The rule is that outpacing strafe is
  what earns the drift. Nothing in code enforces it; `VehicleTuningProfileEditor` warns if they
  diverge.

**Two payouts explicitly rejected by the owner, both still standing:** no speed or boost reward on
drift exit (drift is about angle, not speed; the CTR reference was given without its boost mechanics),
and no spinout consequence (it is a hovercraft).

---

### Jump energy costs
*Closed as M.9, judged good 2026-08-14.*

Cost ramps on the **same `chargeT` that sets the impulse**, so what you pay and what you get can never
disagree. New tunable `jumpGroundedChargedEnergyCost`; the existing `jumpGroundedEnergyCost` became
the tap price.

| Charge | Cost | Impulse | Apex | Jumps per full tank |
|---|---|---|---|---|
| 0.0s (tap) | 10.0 | 20.0 m/s | 5.1m | 10 |
| 1.0s | 17.5 | 30.0 m/s | 11.5m | 5 |
| 2.0s (full) | 25.0 | 40.0 m/s | 20.4m | 4 |

Air jump **15**. Previously a flat 20 / 20 for everything.

Rationale: a tap jump is used constantly just to clear things on the ground and previously cost the
same as a 20m launch that buys a whole trick window. The air jump sits below a full charge because it
is a hang-time extender and a juke rather than a second full jump, and it competes with boost for the
same meter.

---

### Air jump
*Closed as M.8, judged good 2026-08-14: "air jump feels good now."*

**Fixed by a fall cancel, NOT by raising the impulse.** `airJumpFallCancel` 0.6 shipped,
`airJumpImpulse` left at 20. The obvious change was the wrong one: the ability did not need more
force, it needed to stop depending on when it was pressed.

Two separate causes were in play and they wanted different fixes:

1. **Cosine loss, which is intended.** Firing along craft up means vertical gain scales with how
   upright you are: 20 m/s at level, ~14 at 45 degrees, 0 at 90. That is the direct consequence of the
   directional-juke decision and cannot be removed without giving up the wall-shove.
2. **The impulse did not cancel the fall, which was the real complaint.** `AddForce(VelocityChange)`
   ADDS 20 m/s; it does not zero what you already had. Fall at 30, air jump, and you are still falling
   at 10, so the input feels like it did nothing. At the apex the same 20 is the full 20 and feels
   strong. **The ability's strength depended on WHEN it was pressed**, which is exactly the shape of
   "sometimes weak".

**This got worse the same week and nobody connected the two:** raising `extraFallGravity` 13 -> 30
means the craft is moving down much faster at any moment of a trick, so a fixed additive impulse buys
proportionally less. Both changes were judged good in isolation. See `CLAUDE.md`, the movement pass
entry, for the general lesson about neighbouring systems needing re-judging when a shared quantity
moves.

Direction verified in play with gravity subtracted via a no-jump control at each attitude:

| Roll | Craft up | Jump delta-v | Result |
|---|---|---|---|
| 0 deg | (0.00, 1.00) | (0.0, 23.0) | straight up |
| 45 deg | (-0.71, 0.71) | (-14.1, 16.8) | up and away |
| 90 deg | (-1.00, 0.00) | (-20.0, 0.0) | pure horizontal off the wall |
| 135 deg | (-0.71, -0.71) | (-20.0, 0.0) | clamped, horizontal |
| 179 deg | (-0.02, -1.00) | (-20.0, 0.7) | clamped, horizontal |

Continuous through 90 degrees. The clamp zeroes the vertical component rather than blending toward
world up, which keeps the FULL lateral direction at any attitude. **The 90-degree row is most of a
wall jump already** (see `TODO.md` 5.10).

Measurement trap that cost a first pass: `ForceMode.VelocityChange` is invisible until the next
physics step. Full form is `CLAUDE.md` trap 23.

---

### Trick energy economy
*Closed as M.10 / 2.10, built and judged good 2026-08-15.*

Mechanism, standing decisions and every measurement are in `CLAUDE.md` (the trick economy entry plus
the `HoverController_Tricks`, `HoverController_Energy` and `VehicleHUD` module rows). Nothing is
duplicated here.

One thing worth recording that is not there: **the original budget arithmetic that motivated the item
is now doubly stale and should not be reasoned from.** It assumed every jump cost 25 (jump costs are
now tap 10 / charge 25 / air 15) and `regenRate` 20 (the owner has since halved it to 10). The economy
is tighter than that arithmetic described AND has a new income source. Judge it as a whole.

---

## Camera

### Boost framing envelope
*Built 2026-08-10 as phase 3 of the camera plan. **The feel judgement is still open**, see `TODO.md` 2.6.*

Tuned against the design pillars, and the two that matter pull opposite ways. "Momentum is the
resource" makes boost a weapon, so committing to it should feel like firing one; "combat and traversal
are the same skill" means the player is expected to be FIGHTING while boosting, so the held state has
to stay readable. The resolution is **a big transient over a modest sustained**: `fovIncrease` 4
against `fovOvershoot` 12, `zPullBack` 0.7 against `zLagOnEngage` 2.5, `releaseSpeed` 3.5,
`settleSpeed` 3.5.

Measured live at 40 m/s: FOV 65 to a peak of **78.2 about 0.16s after engaging**, settling to **69.0**;
follow Z pulling back **2.58m** at the peak, settling to **0.71m**; release to 5% in **0.61s**, roughly
four times the entry. The transient is down to a fifth of its peak within half a second, which is the
readability requirement.

**Apparent longitudinal motion roughly triples under boost:** the rate the craft moves toward or away
from the camera in frame runs mean 1.01 m/s with boost off and **2.88 with boost on**, peaking at
155.3 during air plus boost. That is the framing working as authored. Nobody has judged whether 2.88
is the right number; the knobs are `zPullBack`, `zLagOnEngage` and `fovOvershoot`.

**`boostBlendSeconds` is 0.35 and has been in every commit since `4a34f21`.** It was believed to have
been cut to 0.125 and then to 0.15, and neither ever persisted, so everything "tuned against the cut"
was in fact tuned at 0.35. It is the highest-leverage knob in the boost system: time-to-peak on the
camera transient tracks the thrust ramp almost exactly, and the transient is measured as the gap
between the boost level and a slower copy of itself, so a snappier ramp opens a wider gap. Cutting it
to 0.15 took peak surge 0.56 -> 0.77 with no camera tuning at all. **It is also the one boost knob
that touches gameplay rather than presentation**, so if the craft ever feels twitchy to commit, revert
this before touching any camera value. The general form of the trap is `CLAUDE.md` trap 21.

---

## Closed investigations worth not repeating

### Floor collisions in a session are hard landings, not the craft catching on geometry
*Closed as 0.8, 2026-08-11.*

27 contacts in one session, essentially all with contact `normal.y` of 1.00, at impulses of 44k-91k
Ns against descent rates of 60 to 95 m/s. Exactly two were anything else: one wall graze at 11 m/s and
one mountain crash at 85-89 degrees of tilt that shows up as a flip recovery.

**The convex-hull theory was wrong and is dropped.** The suspicion was reasonable (the hull on
`car.fbx` does carry a simplification warning) but a bulging hull would catch on walls and edges, and
there are none in the data. What the numbers describe is a craft being flown off very large jumps and
landing hard, which is the game working.

Retained only because "26 severe velocity discontinuities" is alarming before it is attributable, and
the next person to see that number should not re-investigate it.

### The 78-degree slide from hard steering plus brake is not a rival to drift
*Closed as 0.10, 2026-08-13, no action.*

Measured at 84 m/s after a mountain landing: full steering lock for 0.6s while pulling full reverse
put the craft 78 degrees off its own nose, with `drift` at 0.00 throughout. It looked like the drift
pose reached for free, without the button.

It is not free, not holdable and not reachable in normal driving:

- **It costs 41 m/s of actual velocity** (84 -> 43 m/s), where drift costs acceleration and keeps speed.
- **It lasts 0.6 seconds** and decays on its own as lateral damping bleeds the sideways component.
- **It only happened above `topSpeed`**, because the craft had just landed from a drop still carrying
  speed.

Mechanism: braking kills the velocity component ALONG the nose and does almost nothing to the
component ACROSS it, since lateral velocity only bleeds through `lateralDamp`. So full reverse plus
full lock opens the heading-versus-velocity angle by removing one side of the ratio rather than by
adding slide. No extra turn rate is needed, which is why it looked impossible from the cockpit.

The drift work since has made the comparison moot: drift holds a stable angle in a 23m corner for as
long as you want it, where this is a decaying transient you cannot steer.

**What survives is one curiosity, not work to schedule:** above the normal cap a transient wider than
`maxDriftAngle` is briefly reachable, and whether a decaying 0.6s window is usable for aiming is worth
one test with weapons live.

### Flip recovery had a dead band on the arm side
*Closed as M.1, fixed 2026-08-12 in `af7d531`.*

Cause and fix are in `CLAUDE.md` (Foundation v1.8). **One caveat that did not make it there and is
worth keeping: the arming gap did not vanish, it moved down 10 degrees.** The specific reported
failure at 80 degrees is definitively fixed, but nothing has proved the craft can still drive at every
tilt below the new arm angle, which is what would make the band empty rather than smaller. No
occurrence has been seen since. Reopen with a gizmo reading if one ever is.

---

## System measurements

Numbers that justify a constraint recorded in `CLAUDE.md`. The constraint is the durable part; these
are here so the next person does not have to re-measure to trust it.

### The jump impulse table
*Measured 2026-08-08 at `hoverHeight` 7, rise gravity 39.24, fall 52.24. Fall gravity has since risen,
so airtime and landing speed are now shorter and faster; apex is unaffected.*

Apex fits `0.0130 x impulse^2` to within 1%.

| Impulse | Apex | Airtime | Landing speed |
|---|---|---|---|
| 15 | 2.94m | 0.76s | 11.4 m/s |
| 18 | 4.22m | 0.91s | 14.7 |
| 20 | 5.20m | 0.99s | |
| 22 | 6.28m | 1.09s | 19.7 |
| 26 | 8.74m | 1.28s | 25.4 |
| 40 | 20.59m | 1.93s | 43.4 |

Two things fall out of it. **No jump could trigger a hard landing at the time** (full charge landed at
43.4 against `hardLandingMinSpeed` 58), so that system was reachable only by falling off the mountain,
confirmed live at 58.0, 62.8, 83.3 and 87.6 m/s. That has since changed for a stacked jump only; see
Fall gravity above. And **small jumps are structurally floaty**, for the spring-catch reason recorded in
`CLAUDE.md`.

### The hover support fade
*Measured pinned at heights above resting.*

| Height above rest | `HoverSupport` |
|---|---|
| 0.00m | 1.000 |
| 0.25m | 0.667 |
| 0.50m | 0.333 |
| 0.75m | **0.000** |

Exactly linear. `IsHoverGrounded` stays true until between 2.40m and 2.60m, confirming the 2.5m band.
**That table is why the two signals had to be split:** at 60 m/s the craft stops behaving as grounded
~12m past a ledge instead of ~21m.

Honest scope note: on tap jumps specifically this is worth about 11% on landing speed and 19ms of
airtime, which is not perceptible. **The felt fix was raising `jumpImpulseMin`.** The real payoff is
ledge transitions. **A correct root cause is not the same as a significant one, and only the second is
worth promising.**

### Air control, closed form
Steady rate is `torque / airControlDamping`; spin-up time constant is `1 / damping`; usable rotation is
`rate x (window - 1/damping)`.

Verified end to end: at jump 40, damping 8, roll 68.5, pitch 36.5 the model predicts **2.135 rolls and
1.148 flips** against targets of 2.15 and 1.15. The model matched the owner's felt results exactly.

### Ceiling duck: the cliff is one metre wide
*Measured at `hoverHeight` 7 with a 2.35m hull, before the duck existed.*

| Clearance | Result |
|---|---|
| 9m | reached 59.9 m/s |
| 8m | **reached 0.7 m/s, travelled 0.4m in two seconds** |

The hover springs have no force ceiling, so geometry below ride height pinned the chassis and friction
ate the drive. After the duck: 6/7/8/9m all reach full speed, 10m and 12m still target the full ride
height, open ground still exactly 7.000m.

**Entrances still need ~10m**, because the probe looks straight up and cannot anticipate an overhang
ahead. Driving at 60 m/s into a 6m tunnel still collides with the leading edge, since the hull top is
9.05m.

### Micro-bumpiness: a 37x attitude spike, and a mean that says nothing
*Controlled at 28 m/s on one surface, grounded samples only.*

Raycasts return the normal of the TRIANGLE hit, never the interpolated normal the renderer draws.
Driving the mountain terrain, ground-normal steps averaged 2.36 degrees (worst 17.45) arriving 5.9 times
a second. The signature: **19 crossings in 7s with chassis pitch+roll at 44.99 deg/s in the 60ms after
each one, against 1.23 deg/s otherwise.**

With interpolated normals: **one crossing, which was a genuine 9-degree terrain feature**, and the
after-crossing signature disappears entirely.

**Read that result correctly: TOTAL attitude motion barely moved, near 8 deg/s either way.** The craft
still follows terrain, which it should. What changed is delivery, from sharp jolts three times a second
to continuous following, and jolts are the part that is felt. **A near-identical mean corresponded to a
large perceptual change.** Remember that the next time a mean says a fix did nothing.

Chosen over filtering the normal over time, because a time filter cannot tell a triangle edge from a
real ramp edge and would make genuine terrain arrive late.

### The drift flip
Past **15.5 degrees** of bank the craft swung its own rims into its own hover sensor rays. The ray
returned **0.04m instead of 6.83m**, compression read as nearly the whole `hoverHeight`, and one flank's
spring jumped **12.5 to 121 m/s^2** against 39.24 of gravity, applied along the rim's normal.

The live bank budget is `passiveBankAngle` 5 plus `maxBankAngle` 18 = **23 degrees**, sitting 7.5 past
the cliff. Reducing `maxBankAngle` to 12 was tried and never helped, because 17 is over the cliff too.
It has since been restored to 18, **which is safe only because the self-skip removed the cliff entirely
rather than because the budget moved.**

Verified on a flat test plane: max tilt over 8s of full drift went **179.1 to 0.5 degrees**, ride height
still 7.000m. Reproduced with a scripted input provider, not by hand.

### Missile knockback is linear and repeatable
`impactForce` measured 99.9 / 98.7 / 100.2 / 100.0 m/s for 100000, and 54.2 / 55.0 for 55000, across
nose, tail and flank approaches.

The current split lands the two weapons cleanly on opposite sides of the flip threshold, verified over
three reps each on an identical flank hit 1.2m above the centre of mass: **Dumbfire at 100000 rolls at
34.7 rad/s to 167 degrees and latches `IsDowned` every time; Soft Homing at 55000 rolls at 7.6 rad/s to
21.6 degrees and never flips.** That is the GDD split arrived at by measurement rather than by choice.

### Collision impact speed is ~0.6 of approach speed
A 45 m/s head-on into a static wall removes **27.1 m/s** (`Collision.impulse.magnitude` over chassis
mass), the rest going to bounce and slide; the craft was still doing 31.5 m/s afterwards. That is why
`collisionMaxSpeed` is 35 rather than the 60 top speed suggested: at 60 the top of the scale would be
unreachable and every real crash would bunch in the lower half.

### Impulse channel liveness
Measured at the camera through `CinemachineImpulseManager.GetImpulseAt`, which reads the signal itself
rather than inferring it from camera motion: landing peak 1.99 against an authored 2.0 with a 0.248s
tail, denied-grounded 0.32 against 0.35, denied-air 0.23 against 0.25, EMP 1.16 against 1.2 with a
0.343s tail.

**Every tail matches its source's authored duration, which is the check that proves the channels are
actually distinct** and not all firing the heavy envelope.

### The camera framing budget
At the tuning of 2026-08-09, `ClimbAtSpeed` put the craft's centre **33.2 degrees below the look axis
against a 32.5 degree half-frame**, fully out of frame. The guard scales look-ahead to 33% and restores
a 2 degree margin.

Booming the camera back is a weak lever by comparison: a comfortable frame at the same ceiling wants the
orbit radius pushed from 8.94m to roughly 16m, which is why `pitchUpDistanceGain` defaults to 0 and is
documented as a third lever rather than a fix.

### Editor against build
*Three minutes of ordinary driving, 29,852 frames.*

| | editor | build (vsync, 165Hz) |
|---|---|---|
| allocation | 4.47 MB/s | **0.01-0.08 MB/s** |
| gen-0 collections | every 24-26s | **2 in 182s** |
| cost of a collection | ~14.4ms spike | none visible |
| frame time | 4.4ms baseline, GC spikes | **6.07ms mean, 0.39ms sd** |
| managed heap | climbing | 28.6 to 29.6 MB, flat |

**99.98% of frames came in under 8ms; five frames out of 29,852 were over.** Owner's independent verdict
on the same run: "buttery smooth."

### Six vehicles: the mean was never the story
*Re-measured within one session, 1 vehicle then 5 AI spawned. AI driving, no combat.*

| | 1 vehicle | 6 vehicles |
|---|---|---|
| mean | 3.35ms | 4.51ms (+35%) |
| p50 | 3.27 | 3.92 |
| p90 | 3.71 | **7.35 (+98%)** |
| p99 | 5.06 | **10.47 (+107%)** |
| frames over 8ms | 0.1% | **7.5% (a factor of 75)** |

An earlier pass reported mean, p95 and max only, and so read as a comfortable +24%. **p95 genuinely does
move little; the degradation lives further out in the tail.**

Against frame budgets at six vehicles: 60Hz misses 0.05%, 90Hz misses 0.57%, 144Hz misses 12.0%, 240Hz
misses 32.7%.

**Physics is NOT the constraint** (max 2 catch-up steps, 0.12% of frames above one) and neither is
allocation. The cost is main-thread per-frame work, and **particles scale with it: 6 systems and 32
particles at one vehicle, 35 systems and 238 particles at six, all before a shot is fired.**
