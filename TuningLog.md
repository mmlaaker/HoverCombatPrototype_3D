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

#### Reopened as 0.21 and closed at 35, 2026-08-17

Owner, 2026-08-16: *"I could maybe go for another slight increase to fall gravity."* Shipped
`extraFallGravity` **30 -> 35** and **judged good in play the same day.**

Owner after driving it: *"I can still hit two barrel rolls so as long as that and the ability to do
one flip persists and air time went down by a little bit, that's good enough for me."*

**That is an acceptance criterion with a test attached, which is rare and worth keeping literally.**
Three conditions: two barrel rolls still land, one flip still lands, airtime measurably down. **The
second is implied by the first** — a flip is slightly faster than two rolls (`airRollTorque` = 2 x
`airPitchTorque` locks the ratio), so two rolls is the binding case and always fails first. **Any
future change that costs trick margin can therefore be tested against the two-roll case alone.**

**No new play measurements were needed, because the closed-form model reproduces the 2026-08-14
table exactly.** Built from the asset (`extraGravityMultiplier` 3, `jumpImpulseMax` 40,
`Physics.gravity` -9.81) it predicts the shipped apex at 20.4m against 21.0m measured, airtime
1.787s against 1.79s, and the hard-landing ceiling at 43.0 against the 43 recorded. Agreement that
close on three independent quantities means the sweep would only have re-measured arithmetic.
**Every input to the old ceiling was re-read from `VTP_Default.asset` first and none had moved.**

| `extraFallGravity` | Fall gravity | Charge-jump impact | Two-roll margin |
|---|---|---|---|
| 30 (was) | 69.2 | 53.2 m/s | 0.12s |
| **35 (shipped)** | **74.2** | **55.0** | **0.09s** |
| 38 | 77.2 | 56.1 | 0.08s |
| 40 (slider cap) | 79.2 | 56.9 | 0.07s |
| 43 (hard-landing ceiling) | 82.2 | 57.9 | 0.05s |

Roughly **25ms of trick margin per 5 units**. 35 keeps the charge jump 3 m/s clear of
`hardLandingMinSpeed` 58.

**Quote the total, never the knob.** 30 -> 35 is a 17% increase in `extraFallGravity` but only a
**7.2% heavier descent**, because `extraGravityMultiplier` 3 already contributes 39.24 m/s² that the
knob adds on top of. The assistant reported the 17% to the owner and it was wrong in the direction
that oversells the change. For scale: 13 -> 30 was +33% on the total, so **this increase is about a
fifth the size of the last one**, which is what "slight" bought.

**The hard landing was never the constraint, and the tracker had it as the headline.** Both of the
owner's rules survive the entire range: a flat charge jump is safe to 43, and charge-plus-air is
permitted to hard-land and already did at 30. The real constraint is the barrel-roll landing margin,
which has no ceiling to hit -- it just gets worse continuously.

**Two stale facts found while re-deriving, both worth keeping:**

1. **The charge-plus-air ceiling of 27 is dead.** `airJumpFallCancel` 0.6 was added afterwards (see
   the air jump entry below). Killing 60% of the fall before adding the impulse raises the stacked
   apex, so it lands harder than when 27 was measured and the true ceiling is lower still. This does
   not block anything, but **it moves which half of the owner's sentence binds**: the clause to watch
   is "not happening all the time", not "never for a charge jump".
2. **The roll tolerance and the airtime margin were nearly equal at 30, and that was luck.**
   `trickMaxLandingRollAngle` 50 is about 0.116s of grace at the measured roll rate, against 0.12s
   of airtime margin -- so at 30 you could keep rolling right into the landing and still pass the
   attitude gate. **At 35 airtime becomes the binding gate on its own** and the tolerance no longer
   covers for it. That is the mechanism to suspect first if landing tricks starts feeling
   inconsistent, and it is a different failure from the one the 2026-08-14 disagreement predicted.

**Not done, and deliberately:** suspending extra fall gravity while a trick is armed would decouple
descent weight from the trick window entirely and cost nothing measurable. Left unbuilt because it
adds a mechanic to solve a tuning problem the owner has not yet reported feeling.

---

### The downed window

*Measured 2026-08-17 to size `TODO.md` 0.13 before building anything. Two scripted runs, flat ground,
craft placed inverted with the rigidbody pose (setting `transform` alone is overwritten by physics).*

**Two different gates, and conflating them is the mistake to avoid.** Control cuts out on
`IsContactingGround && tilt >= flipRecoveryAngleThreshold` (80) — no delay, no speed gate, so you are
downed the instant an inverted hull touches anything, even at 10 m/s. Control comes back at
`flipRecoveryReleaseAngle` (35), **not** at 80. `flipRecoveryDelay` (1s) gates neither: it gates only
the righting torque, and it additionally needs tilt >= `flipRecoveryArmAngle` (70) and speed <
`flipRecoverySpeedThreshold` (2), held continuously.

| | At rest, inverted | Wipeout at 25 m/s |
|---|---|---|
| Lockout engages | immediately | immediately, then **chatters 3x** |
| Righting authorizes | 1.02s | 3.77s |
| **Control returns** | **1.96s** | **4.84s** |
| Tilt at handback | 34.8 deg | 32.6 deg |
| Speed at handback | — | **19.0 m/s** |

**Three findings, each of which changed a decision:**

1. **`IsDowned` chatters.** In the 25 m/s run it dropped and re-engaged three times inside the first
   0.9s, longest gap 0.25s, because bounces break ground contact and the lockout reads contact
   directly. **Anything that hands the player something while downed must latch**, which is why
   `downedCameraHold` exists rather than the camera reading `IsDowned` straight.
2. **The arming clock resets to zero, not partially.** It reached **0.98 of the required 1.00s** and
   was discarded; it had already lost a 0.30s attempt. The craft is pushed out of the gate by its own
   settling tumble at 161 degrees of tilt — **the recovery interrupts its own clock.** **Fixed the
   same day, see below.** Note the excursions were 352ms and 229ms rather than single frames, which
   this entry originally implied; the reset's defect is that it charged the same price either way.
3. **The handback is abrupt.** Control returns mid-swing at ~33 degrees with the craft still rotating
   at 4.6 rad/s, and in the sliding case being flung at 19 m/s as the hover springs re-acquire.

**The at-rest case cannot see finding 2 at all** — it never resets once. Flat ground at rest is not a
sufficient test here, exactly as it was not for the release-angle equilibrium.

---

#### Closed as 0.18: the arming clock now decays instead of resetting

*Shipped 2026-08-17. `flipRecoveryProgressDecay` added at 1, `flipRecoverySpeedThreshold` left at 2.*

**The threshold was never the defect, which is why 0.18's framing as "an unconfirmed guess" was the
wrong headline.** The defect was in what an excursion past it COST: `flipTimer = 0f` threw away
everything banked, and **that cost did not scale with the interruption.** A one-frame blip and a
229ms excursion were priced identically, at everything.

**A correction to the first write-up of this, which said a single frame was throwing away the
second.** Measuring the excursions directly rather than inferring them from the resets, the two that
mattered were **352ms and 229ms** — genuinely long, not single frames. The mechanism is unchanged and
so is the fix; the vivid version was simply not what the data said.

Same scripted 25 m/s wipeout, before and after:

| | Reset (was) | Decay 1 (shipped) |
|---|---|---|
| Cost of the 229ms excursion | **0.98s**, everything banked | **0.23s**, exactly its own length |
| Righting authorizes | 3.77s | **3.03s** |
| **Control returns** | **4.84s** | **4.06s** |
| At rest, control returns | 1.960s | **1.962s** |

**0.78s off the wipeout, and a provable no-op on the case that was already fine** — 2ms apart at
rest, which is one physics step. That was the property worth buying: the at-rest case never resets
once, so it had nothing to gain and everything to lose from a change here.

**Decay 1 is symmetric and that is what makes it safe.** An interruption costs exactly its own
duration, so a craft alternating in and out of the gate makes no net progress. It cannot arm anything
the reset would not have armed eventually; it only stops discarding work already done. **Raising the
speed threshold instead would have been a real behaviour change**, because the same number is what
stops a craft sliding at speed from arming mid-slide.

---

### Camera control while downed

*Shipped 2026-08-17 as the first half of `TODO.md` 0.13, on the owner's instruction. Not yet judged
in play.*

The measurement above is what justified building it: **2 to 5 seconds of looking at nothing**, which
is far longer than the item had assumed. Right Stick X swings the camera around the craft while
downed; Right Stick Y keeps doing camera pitch as normal.

**The axis is free rather than shared.** Propulsion suppresses commanded yaw entirely while
`IsDowned`, so there is no mode conflict to resolve — and this is a different situation from the
rejected trick-camera proposal, where the objection was that trick control on one stick plus camera
on the other is too much at once. Downed, the player is flying nothing.

`downedYawSensitivity` 1.5, `downedYawRange` 180, `downedYawRecenterSpeed` 1.5, `downedCameraHold`
0.5. The first three deliberately match the shipped pitch values (`pitchSensitivity` 1.5,
`pitchRecenterSpeed` 1.5) and run the same arithmetic, so the two axes feel identical by
construction rather than by tuning.

Verified in play mode against the live `Camera.main` pose, not a reconstructed one:

| | |
|---|---|
| Orbit rate at full stick | **90.0 deg/s**, against 90.0 predicted (1.5 x 60) |
| Horizon roll across the whole orbit | **0.000 deg**, at up to 122 deg off-axis and 54 deg of craft tilt |
| Latch behaviour | held 0.50s past `IsDowned` clearing, then released |
| Recentre | unwound 128 deg to under 6 deg with no discontinuity |

**Horizon roll being exactly zero is the load-bearing result.** The orbit rotates the follow offset
about `Vector3.up`, and the drive rig is bound `LockToTargetWithWorldUp`, so the offset lives in a
frame that yaws with the chassis but never pitches or rolls with it. Rotating about the CRAFT's up
would have rolled the horizon upside down at precisely the moment the player is trying to read it.

**Two things this does not do**, both deliberate. It leaves the look point alone, so the craft stays
centred and this is a look-around rather than a pan into space. And it leaves the strafe rig alone,
whose composer is the player's aim.

**Left open:** the handback. Control returns at 33 degrees of tilt with the craft already moving, so
the camera can be far off-axis at the moment the player needs to drive. It currently unwinds at
`downedYawRecenterSpeed`, the same spring the pitch axis uses, which is a starting point rather than
a considered answer.

---

### The drift hop was being suppressed, not weak
*Closed as 0.23, shipped 2026-08-17. Owner after driving it: "this already feels a good degree better."*

Reported as: "sometimes the hop doesn't feel punctual enough, it might be when I'm already turning...
it's missing that satisfying little thump that the drift has started."

**The item was written as a missing-cue problem and that was wrong.** `Propulsion.OnDriftHop` had no
subscribers, which was true and looked like the whole answer. The actual defect was that the hop was
**not firing at all**, gated behind `HoverSupport > 0.9f` alongside the impulse.

Measured on a scripted corner, 20s, throttle 0.85 and turn 0.7, 94-95% grounded:

| | entries | hop fired | lost | support at each entry |
|---|---|---|---|---|
| before | 3 | 2 | **1** | 0.99, **0.23**, 1.00 |
| after | 3 | 3 | 0 | 0.99, **0.00**, 1.00 |

**The shape is the useful part, not the rate.** Support is not chronically marginal: median 1.000,
and 80% of cornering frames above 0.9. What happens is that about a fifth of the time the craft is
genuinely off its springs, and an entry landing in one of those windows was silently dropped. Note
the failures read 0.23 and 0.00, not 0.89 -- these were not near misses, which is why it felt random
rather than weak.

**The window is wider than it looks, and it is not "airborne".** `heightFactor` falls to zero
0.75m above ride height while `IsHoverGrounded` stays true for 2.5m, so there is a **1.75m band where
drift will happily engage and support reads exactly zero.** Cresting a rise, the rebound after a
bump, or one sensor overhanging an edge all land in it, and none of them look dramatic on screen when
the belly already rides 4.7m up. This is `CLAUDE.md`'s "IsHoverGrounded stays true for 2.5m of free
fall" warning seen from the other side.

**The fix splits one gate into two, because it was doing two unrelated jobs.**

- **The EVENT fires on every entry**, rate-limited only by a 0.4s rearm. It marks a player action, so
  dropping it for a bump was the bug. Anything built on it later inherits the reliability.
- **The IMPULSE scales by support** rather than being gated on it. A 10 m/s VelocityChange applied
  while the springs are not carrying the craft is a jump, not a hop. Scaling follows what
  `HoverSupport` is for: Foundation already crossfades fall gravity, air control, leveling and drag
  on it so no feature has a cliff, and a boolean here would only move the cliff.
- **The 0.4s rearm replaces the support test as the rate limiter**, which the old comment says was
  its real job (stopping a wobbling stick re-firing across the turn threshold). Set against the hop's
  own flight: 10 m/s against 39.24 m/s^2 apexes at 0.25s. Short enough that exiting one corner and
  entering the next still earns a second hop.

**Two cues were proposed and both rejected by the owner, worth not re-proposing.**

- **A camera impulse channel.** Built, then reverted. The owner's challenge was the right one: drift
  exists to aim, so shaking the view at drift entry spends aim stability at the moment the player
  entered drift in order to aim. That is 2.9's open objection about shaking on weapon hits, applied
  to a manoeuvre that happens far more often.
- **Snapping the chassis bank at entry.** Rejected on a general principle worth keeping: **poses
  blend, they do not snap.** A snap on one axis reads as a different system rather than as emphasis.

**No cue was built at all, and the report was answered anyway.** The physical hop is sufficient once
it actually fires. The remaining visual work is 2.12.

---

### Drive and strafe framing
*Closed as 0.14, shipped 2026-08-17 for pre-alpha 1. Revisit deferred to pre-alpha 2 as 5.12.*

**Shipped by the owner after a long assisted search.** Read from the prefab: drive follow offset
(0, 4, -7) at `baseFov` 65; strafe follow offset (0, 3.5, -6) at `fovReduction` **8**, so strafe FOV
57; strafe composer `TargetOffset.y` 1; `reticleProjectionDistance` 200.

**Only two numbers actually moved**, and that is the headline. Drive dropped 0.5m, and the strafe FOV
reduction went 10 to 8. Everything else returned to where it started after wider candidates were
tried and rejected.

| | look-down | horizon | reticle | crosshair gap | hull |
|---|---|---|---|---|---|
| drive before | 23.2 deg | 0.836 | +455px | 0.166 | 51.6% |
| **drive shipped** | **19.7 deg** | **0.780** | **+382px** | **0.128** | **50.7%** |
| strafe before | 22.6 deg | 0.900 | +552px | 0.133 | 72.6% |
| **strafe shipped** | **22.6 deg** | **0.884** | **+383px** | **0.127** | **69.6%** |

**THE RESULT THAT MATTERS IS THE ONE THAT WAS REJECTED**, because it is the reason the shipped values
are so close to the originals.

**Two candidate framings were built, measured and thrown away.**

1. **Flatten the camera to bring the horizon toward centre.** Strafe at (0, 2.2, -7) FOV 58 put the
   horizon at 0.655 and the hull at 47.9%, both large improvements, and was **unusable**: the
   crosshair gap collapsed from 0.133 to **0.003**. The reticle sat four pixels above the roofline
   and welded itself to the cab. Owner: "the reticle looks fixed to the top of my cab in a way that
   I'm not aiming at what's out in front of me, and I can't even see my own bullets as a result."
   **The gap comes from how far the camera sits above the ROOFLINE** (2.05m above the craft origin),
   and flattening the camera to roof height merges the roof with the horizon. Every metre of height
   removed costs roughly 0.075 of gap.
2. **Pull back and narrow the lens to shrink the hull.** Strafe at (0, 4.2, -11) FOV 48 achieved
   everything on paper -- horizon 0.623, hull 42.3%, gap 0.136, biggest target size of any candidate
   -- and produced the **Vertigo effect**. Pulling back while narrowing cancels on subject size and
   adds on perspective, which is the dolly zoom, literally. Owner: "the FOV change making my car
   appear to squash... it just kinda looks weird flattening the world and car like that."

**Use the dolly index when judging any future framing change: apparent size of the craft in strafe
divided by its size in drive.** 1.00 means the car holds still while the perspective reshapes around
it, which is the artifact. The rejected candidate measured **1.10**. The original was 1.73 and never
bothered anyone. **The shipped 1.37 is the number to stay near.** Most third-person games avoid this
by keeping the FOV delta small and moving the camera toward the subject; this project's delta is now
8 degrees, down from 13.

**The trade curve, so it is not re-derived.** Crosshair clearance, horizon height and hull size cannot
all be improved at once with the hull fully on screen. Holding the original's 0.133 gap while getting
the horizon down to 0.623 required (0, 4.2, -11) at FOV 48, which is the Vertigo candidate. Nothing
inside 6-8m of distance satisfied all three.

**Also settled: `reticleProjectionDistance` has almost no authority at this framing.** Across 200m
down to 70m the reticle moves about 23px, because a flat forward ray converges near the horizon
regardless of distance. It regains authority only if the camera gets much flatter. **0.14's
instruction to "retune the view then set that number to match" is effectively a no-op now**, which is
a good state: one less coupled knob.

**What was NOT fixed and is the biggest number on the table: the hull still occupies 69.6% of screen
height in strafe.** Every candidate that improved it broke something judged more important. That is
5.12's first question.

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

**Values above have drifted since this was written.** Read live 2026-08-17 from the scene instance:
`fovIncrease` 4, `fovOvershoot` **6** (not 12), `zPullBack` **0.5** (not 0.7), `zLagOnEngage` 2.5,
`releaseSpeed` 3.5, `settleSpeed` 3.5, `forwardGateSpeed` 2. Re-measured peak at engage is FOV 65 to
**72.4** and follow Z to **-8.95** from -7, reached 0.35s after engaging. The prose above still holds;
only the magnitudes moved.

### The boost jolt at a standstill
*Shipped 2026-08-17, closing `TODO.md` 0.20. **The recorded mechanism was wrong and the measurement
disproved it.***

Owner, 2026-08-16: *"There is the occasion where I start a boost with zero or barely any speed, and
the camera movement can feel kinda jarring in that moment."*

**What 0.20 claimed, and why it was wrong.** The item had it that `ForwardGate` =
`Clamp01(travelSpeed / forwardGateSpeed)` with `forwardGateSpeed` at 2 delivered the whole camera
package as a step within ~15ms of engaging. That was arithmetic done without a run, and it is
backwards. `boostLerp` is a `MoveTowards` over `boostBlendSeconds` 0.35, so **the gate opens before
the envelope has any magnitude to gate**: measured, the gate reads 1.000 at the first frame sampled
after engage, 51ms in, when the boost envelope is at 6.6% of full. Raising `forwardGateSpeed` would
have changed behaviour, but not the behaviour that was reported. The gate is doing only its original
narrow job of killing the FOV kick when you boost in reverse.

**The discriminating measurement.** Three runs on one flat 340m lane, same start, scripted input, one
variable changed at a time. Sampled per frame off live `Camera.main` and the live camera transform,
not off the commanded values (`Measuring.md` traps 26 and 41).

| run | look-point swing | rig pull-back | lens | verdict |
|---|---|---|---|---|
| throttle from rest, no boost | **6.00m** over 0.92s | 1.53m | none | never reported |
| boost at speed | **1.91m** | 2.44m | +7.5 deg | judged good |
| **boost from rest** | **6.00m** over 0.70s | 2.59m | +7.5 deg | **the report** |

**The boost package is identical at rest and at speed** — 72.47 against 72.41 degrees peak, 2.59m
against 2.44m of rig travel — which is what rules it out as the cause. What is unique to the bad case
is that boost is the only thing that drags the look-ahead through its **entire** six-metre swing,
because at speed the term is already saturated and does not move at all. Each ingredient alone was
already accepted; only the standstill case carries both, inside the same third of a second, against
a world that is not yet flowing past to hide it in.

**The fix: a slew limit on the look point, `speedLookAheadSlew` = 8 m/s.** Peak rates over 50ms
windows, which is the quantity that separates the cases:

| | before | after |
|---|---|---|
| boost from rest | **11.49 m/s** | **8.03 m/s** |
| throttle from rest | 7.74 m/s | never reaches the limit |

Chosen so it cannot touch the case already judged good. Verified by logging the limited and unlimited
values inside the same run rather than comparing peaks across runs: flooring it from rest, **the
largest gap the limit ever opened was 0.05m**, and full look-ahead was reached at 0.918s against
0.921s unlimited. At cruise it never engages, the term being saturated.

Cost, and it is the only one: the look point reaches full extension at **0.85s instead of 0.70s**
under boost, so the craft sits fractionally higher in frame for that 0.15s. Raise toward 12 to hand
the jolt back.

**Rejected: raising `forwardGateSpeed` toward 30.** It was the item's own proposal and it does have
the shape of a fix — the package would ramp with speed and cruise is provably untouched, the gate
being saturated above 30. Rejected because the measurement showed the package is not what differs
between the good case and the bad one, and because delaying it breaks what it is for. The engage
transient is a "you just committed" cue; a cue that waits is a worse cue. **`forwardGateSpeed` stays
at 2 and its tooltip's "keep this LOW" still stands.**

**Side effect nobody has judged:** the limit is symmetric, so a collision that takes the craft from
top speed to a standstill no longer snaps the look point back six metres in one frame. It retracts
over roughly 0.75s instead.

**Why a slew limit rather than smoothing.** The quantity separating good from bad is a peak rate, and
a lerp cannot bound a peak rate: its speed scales with the gap, so a 0-to-6m gap starts fast whatever
constant is chosen. The limit also moved the look-ahead into the integrate stage, since a rate limit
is memory; `ContributeLookAhead` stays pure and `SpeedLookAheadTarget` is the single expression of
the speed-to-distance mapping, shared with the preview states so the two cannot diverge.

### The boost gate was a step against reversing
*Shipped and **judged good** 2026-08-17, closing `TODO.md` 0.26. **Found in marker data, then
reproduced.** The owner re-drove the exact manoeuvre that produced it — boosting backwards and
forwards — and accepted the result including its cost at engage.*

Owner, 2026-08-17: *"If I boost and flick the left stick forward and backwards, I can create a camera
jumping back and forth kind of bug."*

**How it was found is worth recording.** The owner pressed `M` twice during a 105s session for what
they described as hitches. Marker 1 (`t=47.04`) was the worst frame of the session at 25.04ms against
a 3.39ms baseline, and turned out to be the already-closed editor GC sawtooth: memory climbs ~4.5 MB/s
from 760MB to 825-857MB and collapses back roughly every 24s, and the marker sits at a peak. **Marker
2 (`t=74.34`) had no frame problem at all** — worst frame within 1.2s either side was 4.83ms — and was
instead this bug, caught live: a flick from +0.88 to -0.99 with boost held, decelerating from 103 m/s,
crossing zero at `t≈72.93` and reaching 83.75 m/s in reverse, which is exactly `reverseTopSpeed` 67 x
`boostSpeedMultiplier` 1.25. Travel direction swung from -2 to -179 degrees in about 60ms.

**Mechanism, then reproduced with boost pinned at 1.00 through a scripted flick:**

| | before | after |
|---|---|---|
| gate 1.000 -> 0.000 | **23ms** | **287ms** |
| FOV carried | 4.00 deg | 4.00 deg |
| commanded rig Z | -7.502 -> -7.000 | -7.502 -> -7.000 |
| rig travel rate | **21.2 m/s** | **3.2 m/s** |

Nothing was removed; the same swing now takes 0.29s instead of a frame and a half. With the engage
surge still live rather than decayed the untreated case would carry nearer 7.4 degrees and 3m in the
same 23ms, since `fovOvershoot` and `zLagOnEngage` are gated too.

**Fixed by `forwardGateSlew` 3.5, a limit on the RESULT rather than on the threshold.** That split is
the whole design. Widening `forwardGateSpeed` would soften the step as well, but it would buy that by
turning a direction test into a speed ramp, which its own tooltip argues against and which this file
already rejected for 0.20. The gate answers "am I backing up"; how fast is not its question.

**Symmetric**, because the artifact is an oscillation. Limiting only the closing edge would cost
nothing at engage but leave the re-opening snap intact, which is half the reported behaviour.

**THE COST, which is real and larger than predicted.** The arithmetic said 0.8 degrees because it
counted only the sustained term. The gate multiplies the overshoot as well, and the overshoot is
already substantial early, so the measured cost from a standstill is **up to 1.8 degrees of lens
through the first quarter second**, with the gate needing about 0.31s to open. **The peak is
untouched: 72.48 against 72.43**, because the overshoot crests at 0.35s once the gate has finished
opening. So the ramp changes shape and the transient still lands at full strength.

That cost pushes the same way 0.20 did — a softer first third to an engage from a standstill is what
that item wanted — and **the owner played it and accepted it the same day**, which closes the
question rather than leaving it tolerated. `forwardGateSlew` 3.5 is therefore a judged value, not a
provisional one, and the softer first third of the engage ramp is now the intended shape.

**Two alternatives, kept only in case the judgement is ever revisited**, in order of how much they
give back:

1. **Raise `forwardGateSlew` toward 8.** The reversal transit becomes ~125ms, still five times
   calmer than the 23ms step, and the engage cost roughly halves.
2. **Make it asymmetric**, opening instantly and closing at the limit. Zero engage cost, and it fixes
   the forward-to-reverse half. It does NOT fix reverse-to-forward, which snaps the package back on in
   the same 23ms, so it trades away half the bug for all of the cost.

A third was considered and not built: **change what the gate reads at rest.** At zero travel speed the
gate currently evaluates to 0, meaning "not going forward", when the honest answer to "am I backing
up" is no. Re-centring it so zero speed reads open would remove the engage cost outright. It also
changes what boosting from a standstill into reverse looks like, and that is a design question about
what the gate means rather than a tuning one, so it wants the owner rather than a measurement.

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
