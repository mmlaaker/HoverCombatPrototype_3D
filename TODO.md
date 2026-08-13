# TODO

Consolidated open work. Created 2026-08-07 from a full documentation-vs-code audit; the code was
treated as the truth and the docs were corrected to match it in the same pass.

**Updated 2026-08-08** from a movement tuning session driven by a full playtest. Reverse/strafe
symmetry, the tap-jump grounded band, air control entry, drift feel and the weapon-switch firing bug
were all closed and their outcomes moved to `CLAUDE.md`. Everything else the playtest raised is
filed below: 2.6 to 2.10 (feel), 0.6 (what that session did NOT verify), 5.9 to 5.11 (decisions
left open). The session's performance investigation is closed and lives in `CLAUDE.md`; the short
version is that the chop was background applications, not the game.

> **STANDING PRIORITY, set by the owner 2026-08-10: a movement-focused playtest with friends, as soon
> as possible.** Everything that serves how the craft moves and how the camera frames it comes first;
> weapons come after. This is why per-weapon recoil character (2.7) is written up and deliberately
> unbuilt, and why the weapon items in Tier 3 are not being worked despite being cheap. Weapons are
> placeholder, so tuning feel around them is tuning against a moving target. **Reassess this line
> once the playtest has happened, not before.**
>
> **SHARPENED 2026-08-11: `Tier M` now outranks everything in this file, including Tier 0.** It is
> the consolidated output of a full movement and camera playtest and the owner set it as top
> priority explicitly. Work it in the stated order, because M.5 (speed) moves numbers that most of
> the rest are tuned against.

**Updated 2026-08-09** from a camera session, then again the same day when the impulse router landed.
Two of 2.8's three complaints are closed and 2.6 is partly closed. **2.1, 2.2 and the shake half of
2.9 are now wired and measured, but none of the three has been JUDGED** -- every one is a feel
question and the numbers only prove the punch arrives, not that it reads right. The camera work, its
measurements and four new measurement traps are in `CLAUDE.md`. Two phases of the camera plan remain:
boost framing (2.6, 6.2) and the reticle (2.8). Note a reported "camera jitter" turned out to be no
antialiasing rather than anything in the camera, and the resolved entry for it is worth reading
before investigating any similar symptom. **Qualified 2026-08-11: do not read that as "it is never
the camera".** A second jitter report the same month WAS the camera. The two split on when they
appear -- the antialiasing one in a straight line at steady speed, the camera one after a stunt on
release or landing -- and `CLAUDE.md` now carries the separator alongside both entries.

**Updated 2026-08-11** from a movement/camera tuning session that started with a reported jitter and
"slight pulling". `MotionTrace` was built for it (see `CLAUDE.md`) because `FrameSpikeWatch` measures
frame DURATION and cannot see either a sub-12ms pacing wobble or a directional artifact. **The
headline is that frame delivery is not the problem**: p50 3.31ms, p90 3.71, p99 4.14 over 42151
frames, and outside collision ticks the craft is drawn within 2-3mm of where the physics has it. The
cause of the felt stutter was the camera heading proxy whipping on air-control release, recorded in
`CLAUDE.md` as `HoverCameraController` v2.6 then v2.7. Four new measurement traps (15-18) came out of
it, three of them about the instrument lying rather than the game. What that session did NOT close is
filed below as 0.7, 0.8 and 0.9.

**Worth reading before the next camera change:** v2.6 capped the catch-up RATE, measurably halved the
whip, and the owner could still barely feel it, because it converted a snap into 0.6s of pan without
touching the accumulation that caused it. v2.7 bounded the accumulation and is the real fix. The
lesson generalises: **a measurable improvement in the metric you chose is not evidence you chose the
right metric.** Both are still in place and both are needed, but only the second addresses the cause.
v2.7 was judged by feel and reported "noticeably better", with a residual stutter **on landing after
flips**. That observation was correct and found a hole v2.7 had opened: its speed gate disabled the
bound during steep post-flip descents, which is the one case it existed for. v2.8 latches the travel
heading instead. **v2.8 is now confirmed in real play**, not only by injection: across matched
sessions the share of air-control frames breaching the 40 degree bound fell from **9.9% to 0.5%** and
the worst divergence from **113 to 50.6 degrees**, the residual being the rate ceiling catching up,
which is what it is for. The owner's remaining F8 markers on landing resolved to one ordinary frame
hitch, one window with no signal in any channel, and one large uncommanded slide that is handling
rather than camera (see 0.10).

The owner's two feel-reports have now each identified a real mechanism ahead of the data (boost/air
authority, then flips over rolls, then landing specifically). Worth weighting accordingly.

**Updated 2026-08-11 (second entry, evening) from a full movement and camera playtest.** Weapons were
disabled and boost made free for the first half, then both restored so drift-and-fire and strafe
could be judged; the environment was doubled to match the 2x vehicle mesh. Output is **`Tier M`,
which the owner set as top priority above every other tier in this file.** Fifteen positive
judgements moved to `CLAUDE.md` > Judged By Play, six TODO items were closed or downgraded (5.9 keep
the air jump, 0.10 withdrawn, most of 2.9, the arena assumption in 6.2, plus slope parking and strafe
acceleration unification declined), and two documentation defects were corrected in place.

**The owner's instinct beat the record twice more this session, which now makes four times.** They
rejected 0.10's "drift is competing with something cheaper" framing on the grounds that no increased
turn rate and no banking meant a line should not be holdable that way, and they were right on every
point: it is a decaying 0.6s transient costing 41 m/s, only reachable above top speed. They also
called the dodge validator wrong (M.4) and it is. **Weight owner feel-reports above both the docs and
the derived readouts, and check the instrument before checking the game.**

**How this relates to the other docs.** Four documents, no overlap; a fact lives in exactly one.

| Question | Document |
|---|---|
| What is not done? | **`TODO.md`** (this file) |
| What exists, how does it work, why is it built that way? | `CLAUDE.md` |
| What should this be? Why? | `GameDesignDocument.md` |
| How were the physics numbers derived? | `PhysicsAudit.md` (frozen at `4a34f21`, not maintained) |

**When an item here is finished, its outcome belongs in `CLAUDE.md`, not here.** Two things are
deliberately absent from this file and live in `CLAUDE.md` instead: consciously accepted
limitations, and theories disproved by measurement. See the closing section.

**Rules that apply to everything below.**
- Live values come from `Assets/Data/VTP_Default.asset` and `Assets/Data/WD_*.asset`, never from a
  doc. Numbers quoted here are dated and may already be stale.
- Pair every derivation with one cheap runtime check. An entire audit section was once
  arithmetically perfect and described a branch that had never executed.
- Never trust a zero you have not proven can be non-zero.

Priority tiers are about what unblocks the prototype, not about effort.

---

## Tier M — Movement pass, 2026-08-11. HIGHEST PRIORITY

**Set by the owner at the end of the 2026-08-11 movement playtest: everything in this tier outranks
every other tier in this file, including Tier 0.** Weapons were disabled and boost made free for the
first half of the session, then both restored so drift-and-fire and strafe could be judged. The
environment was doubled to match the 2x vehicle mesh, so anything below dated to this session was
felt in a world twice the size of every earlier measurement.

Fifteen things were judged GOOD in this session and are recorded in `CLAUDE.md` > Judged By Play,
not here. Read that list before changing anything in this tier: several items below are bounded by
something already confirmed to feel right, and the confirmations are the acceptance criteria.

### M.1 Flip recovery has an unrecoverable band on the ARM side

The craft can come to rest at a tilt just under `flipRecoveryAngleThreshold` (80), too flipped to
drive and not flipped enough to authorize righting. Caught on screenshot 2026-08-08, gizmo reading:

```
FLIP 0.00/1.00s  BLOCKED: tilt below threshold
tilt 80deg (need 80)      speed 0.65 (need < 2.00)
contact True  hoverGrounded True  authorized False
UNSTICK FIRED   Unstick 0.09 / 0.20s
```

Speed is not the blocker (0.65 against 2.00). Unstick fires repeatedly and cannot help, because it
pushes **up** and the problem is rotational. Leveling torque at 12 evidently cannot climb out while
the hover springs hold the craft in that pose.

**`CLAUDE.md` v1.4 already records this exact hover-supported equilibrium at ~78 degrees**, and fixed
it on the RELEASE side only by adding `flipRecoveryReleaseAngle` 35. The arm side was never
addressed. This is the same defect at the other end of the same gate.

Rare in practice: one occurrence in several days of play, and the owner rates unstick and flip
recovery as consistent otherwise. It is in this tier because the mechanism is known and named, not
because it is frequent.

**This is the justification `HoverController_Foundation` requires for modification.** Caveat to check
first: the gizmo rounds, so 80 may be 79.6. That does not change the mechanism.

### M.2 Boost camera effects stack onto strafe mode. They must not

Owner rule, stated 2026-08-11: **strafe mode gets full crosshair authority with zero modifiers.**
Resting in strafe with throttle forward currently makes the camera look up.

`ContributeBoostLens` applies to BOTH modes while only `ContributeBoostPullback` is drive-only, so
the FOV half stacks on strafe by design. That design is overruled.

**This closes an open question rather than opening one.** 2.6 recorded that the boost forward-gate
reads forward speed only, so boosting sideways in strafe produced no lens change, and noted nobody
had judged whether that was right. It was right, and it should go further.

Two candidates for the look-up specifically, to be separated rather than guessed between: the speed
look-ahead term (`speedLookAheadMax` 6 against `speedLookAheadReference` 60) and the strafe aim
pitch. The first scales with throttle exactly as described.

### M.3 The vehicle briefly leaves frame while the camera pitches up — STILL OPEN, two theories killed

Part of the craft clips offscreen mid-transition as the camera pitches up to look down on it. Both
endpoints are fine; the motion between them is not. Requirement: **vehicle fully onscreen at all
times.**

**Confirmed real and quantified 2026-08-11.** Driving real terrain, hull margin ran +0.2086 of the
viewport before the pitch, **-0.0404 during it (fully off screen) at t+0.385s**, and +0.0806 settled
at full stick-up. Reproduced a second time at -0.1090. The BOTTOM edge is the one breached.

**Theory 1, orbit geometry: DEAD.** The settled pose at full stick-up fits comfortably, so
`pitchUpDistanceGain` was never the cause. It would have bought margin without touching the mechanism.

**Theory 2, camera lag: DEAD, and this one was tried and reverted.** The rig really does trail its
solved pose by up to **2.498m** during a fast stick-up (solved Y 9.552 against an actual 8.645), with
the aim lagging on top of that (composer damping 0.5s against position damping 0.1-0.2s). So the
guard genuinely is certifying a frame nobody is looking through. Feeding it the rig's REAL position
was implemented, measured, and backed out: on a pinned repeatable manoeuvre it changed the worst
margin by **-0.0009 at 60 m/s**, and at 45 m/s the guard performed slightly WORSE than being switched
off entirely. Correcting the position while still assuming an ideal aim makes the model less
self-consistent, not more.

**What the measurements actually point at, and the next thing to test.** The clip reproduces readily
on real terrain and **does not reproduce at all on flat ground with the craft held level, at any
speed, with the guard on, off, or modified.** So the missing variable is **chassis attitude**, not
camera lag. The likely mechanism: `Measure` treats the hull as a bounding SPHERE, which is
attitude-independent, while the thing that actually leaves frame is the **rear-bottom corner of an
oriented box**. `CLAUDE.md` > Vehicle Scale already warns that the rear-bottom corner is the first to
leave frame and that the collider union is not centred on the origin. A sphere model cannot see a
corner swinging down as the nose pitches up over a crest.

**Test before building anything:** reproduce with the craft pitched, on a slope or a crest, and
compare the sphere margin the guard computes against the true oriented-box margin. If they diverge,
the fix is in `Measure`, not in the guard's inputs.

**Verification harness worth reusing:** the oriented-box viewport check written for this
investigation. Eight corners of the measured collider union through `Camera.WorldToViewportPoint`,
reporting worst margin and which edge broke. It is the only instrument here that measures the frame
the player sees rather than the frame the solver intended.

**Measurement lesson, and it cost three bad conclusions:** the first runs let the craft accelerate
from wherever the previous test left it, so no two were comparable and one apparent regression was
pure starting-state noise. **Pin position, rotation and velocity before any camera framing test.**

### M.4 The dodge validator models an unopposed impulse and cries wolf

`VehicleTuningProfileEditor.DerivedDodge` computes delta-v as `force x (duration + fixedDeltaTime) / 2`
and divides by `strafeTopSpeed` 40. At `dodgeForce` 650 over `dodgeDuration` 0.15 that exceeds 100%
of the cap, printing "reads as a TELEPORT, hard to follow at speed".

**The owner disagrees and is right.** Judged 2026-08-11: it reads as the intended quick sidestep
juke, with possible room to push it further.

The formula is arithmetically correct about a quantity the player never experiences. `lateralDamp` 1
and the over-speed bleed oppose the burst DURING and AFTER it, so realized peak lateral velocity is
well below the open-loop figure. **Not yet measured; one play-mode run settles it** and gives the
validator a real denominator.

**Third instance of this exact mistake in this project**, after the Movement gizmo's drive/drag
warning and the camera framing verdict, both of which had to be corrected for firing during correct
play. A check that fires during good tuning gets ignored, which costs more than having no check.

### M.5 Speed. Raise top speed, lower the boost multiplier

Owner, 2026-08-11: top speed feels too slow. `VTP_Default` is the "Sedan", average in everything,
and it is still too slow. **Reported both before and after the environment rescale**, so it stands on
its own rather than being an artifact of the bigger world, though the rescale amplifies it.

Direction: **raise `topSpeed`, lower `boostSpeedMultiplier`** (currently 1.5). 1.5x is high as boost
multipliers go, most racing games sit nearer 1.15x to 1.3x, and 50% already reads as merely "okay",
which is close to proof that more multiplier is not the lever.

**Do this cluster FIRST. It has the widest blast radius and everything else gets retuned against it.**
Five couplings:

1. **`speedLookAheadReference` is 60, which IS `topSpeed`.** The camera's speed look-ahead normalizes
   against it and clamps at 1. Raise `topSpeed` without raising this and look-ahead saturates before
   you reach top speed, so the camera stops responding to your fastest driving. **A camera value
   pinned to a vehicle value with nothing enforcing the link.** Highest-risk item in this cluster.
2. **`minDriftSpeed` 40 and `strafeTopSpeed` 40 are held equal by hand**, the rule being that
   outpacing strafe is what earns the drift. Decide explicitly whether they follow `topSpeed`, or
   drift entry difficulty changes silently.
3. `hardLandingMaxSpeed` 85 is already saturated by boosted 90. A lower multiplier relieves this.
4. Lowering the multiplier also narrows 5.11, where boosted strafe reaches unboosted drive top speed.
5. Perceived speed is structurally low (8.7 body-lengths/sec against 15-16 for a fast road car) and
   **doubling the environment halved how fast the world goes past you.** See 6.2, and note the arena
   is now confirmed to be deliberately sparse, so props are NOT coming to fix this.

### M.6 Airtime. Raise fall force, and decide what that costs the trick economy

Owner: long airtime reads "a tad floaty". Wants more falling force. Also wants the tap jump slightly
less floaty.

**The rebalance the owner budgeted for is not needed.** Rise and fall run on separate gravity
(`extraGravityMultiplier` 3 on the way up, `extraFallGravity` 13 added only while descending), so
fall force does not change jump height at all, only descent time and landing speed.

**Owner's acceptance criterion, set 2026-08-11: a full charge jump must still land two barrel rolls
or one flip.** Computed budget against that, full charge jump (`jumpImpulseMax` 40, rise gravity
39.24, apex 20.39m, rise time 1.019s which is IMMUNE to fall gravity):

| `extraFallGravity` | Fall time | Total airtime | Landing speed | What binds |
|---|---|---|---|---|
| 13 (live) | 0.88s | 1.90s | ~46 m/s | nothing |
| ~43 | 0.70s | 1.72s | 58 m/s | **full charge jumps start hard-landing** |
| ~138 | 0.48s | 1.50s | 85 m/s | **`hardLandingMaxSpeed` saturates** |
| infinite | 0 | **1.02s floor** | - | the two-roll invariant, eventually |

**The invariant is the LAST constraint to bind, by a wide margin.** Hard landings arrive first at
roughly 3.3x the current value. More than half the trick window (the 1.02s climb) cannot be taken
away by fall gravity at any value.

**One measurement closes this: time two barrel rolls.** Under 1.50s and the saturation point is
reachable while keeping the invariant. Deliberately not estimated here, because the only rotation
rate in the docs comes from a different context (a downed craft levering off the ground) and is not
a safe basis for a budget.

**The conflict to decide before tuning:** tricks are meant to pay energy back (2.10), scaled by
rotations landed. Less airtime is fewer rotations, so falling faster shrinks the economy being built.
The air jump was also kept specifically as a hang-time extender (5.9). Owner's position 2026-08-11:
acceptable as long as the two-roll invariant holds. Options if it turns out to bite: front-load the
fall gravity so only long falls feel heavy, or pay trick energy by rotation COUNT rather than
airtime, so a faster tighter trick pays the same.

Note the crossover at ~43 means jumps would begin triggering hard landings, which 2.9 records as
never happening today. That may be desirable. It should be a decision, not a surprise.

### M.7 Drift. Add a time cost, and more path curvature control

**Target named by the owner 2026-08-11: the Crash Team Racing power slide, without the boost
mechanics.** That is now the reference. Confirmed working already: holding line of sight on a target
while maintaining a different velocity angle, which is v1.8's stated purpose.

Two changes wanted, and they are NOT the same kind of work:

**(a) Sustained drift should bleed speed over time, even on full throttle.** A long drift should
eventually cost you everything. **This needs a new term and is not a knob.** `driftForwardDamp` is a
coefficient, so forward throttle and drift drag settle at an equilibrium speed and hold there
indefinitely; raising it only lowers the plateau. What is wanted is a cost that GROWS with drift
duration, which does not exist.

**(b) More control over drift path curvature.** Tight around the outside of the figure-eight curves,
easing out on the straights. This half genuinely is existing knobs: `maxDriftAngle` 60,
`driftLateralDamp` 0.3, `driftYawMultiplier` 1.5. The v1.8 two-knob model applies, `maxDriftAngle` is
the pose and `driftLateralDamp` is the path.

**No spinout consequence wanted.** Deliberately excluded, it is a hovercraft. Do not add one as a
way of implementing (a).

Watch: the drift entry hop is live at `driftHopImpulse` 10, which is 1.70m of climb into a 2.5m float
band. That is the value the owner approved on 2026-08-08, not a regression, but it leaves little
margin and (a) will change how the entry reads.

### M.8 Air jump fires along world up. It should fire along the craft's local up

Owner, 2026-08-11: **the air jump is worth keeping** (this closes 5.9), and its value is as a mid-air
juke and trick-height extender. But the impulse should be relative to the bottom of the craft, so
tilting toward a wall propels you away from it.

**Cheapest single change on this list with the largest character payoff.** It converts the air jump
from a height extender into a directional juke, which is what the owner actually described wanting it
for. It also partially pre-builds 5.10 (wall jump / wall riding).

### M.9 Jump energy costs should differ per jump type

Currently flat: `jumpGroundedEnergyCost` 20 and `jumpAirEnergyCost` 20. The grounded jump is also
variable-height on a charge, and charges nothing extra for a full charge.

Pairs with M.10 and with 2.10. Do it after airtime settles, since the right cost depends on what a
jump buys.

### M.10 Trick energy economy, with the owner's risk/reward framing

Owner's plan, stated 2026-08-11 and consistent with what they described in earlier sessions:
**replenish boost energy by waiting for regen OR by landing tricks, with more flips and rolls paying
more.** Energy is a "constant feeling" of being short, which matches 2.10.

**New this session: the risk/reward framing.** Going for more rotation risks landing badly and
banking nothing, which is what makes it a decision rather than a payout. That is the part worth
protecting in implementation.

Mechanism is already specced in 2.10 and not repeated here. Note the coupling to M.6: if trick
energy is paid by ROTATION COUNT rather than airtime, the fall-gravity conflict mostly dissolves.

### M.11 Camera authority when the player has none. One feature, two triggers

Two requests from this session that are the same feature:

- **While flipped and helpless.** The camera settles to the side of a flipped craft. The owner wants
  to swing it where they like, since there is no other agency until the craft rights itself.
- **While mid-air doing tricks.** Right stick moves the camera during heavy air, so a long trick
  sequence can be used to set up for landing.

Both are moments where the player has input available and nothing useful to spend it on, and in both
the thing they want to do is look at where they are about to end up.

**The mode conflict is tractable for exactly that reason:** right stick X is yaw, and yaw is near
worthless in both states. Airborne yaw is confirmed working at half authority (`airTurnMultiplier`
0.5) and the owner is happy with that, but also confirmed it "doesn't really feel like it's
contributing to my direction" during tricks. Downed already suppresses commanded yaw entirely.

Note the coordinate-frame finding behind this, verified in code and confirmed in play 2026-08-11:
`ApplyTurning` uses `AddRelativeTorque(Vector3.up ...)`, the craft's LOCAL up, and is never scaled by
air-control weight. So air control does not suppress yaw, it **rotates the axis yaw acts about**.
Rolled 90 degrees, yaw input pitches you in world terms. That is why steering feels absent during a
trick while measuring as fully live.

### M.12 Boost presentation FX: vignette, speed lines, duration-based rumble

Owner: boost camera effects are liked in general but want small tweaks, hard to verbalize. Concrete
additions requested, **none of which exist today**: a vignette, speed lines, and a subtle camera
rumble that **fades in over sustained boost**.

All three are presentation rather than framing, so they sit outside the v2.5 boost envelope work and
do not touch the single-write-authority solver.

**The rumble is the interesting one.** It is the first camera effect requested that builds with
DURATION HELD rather than firing on an event, and the boost envelope already tracks exactly that in
`_boostHold`. Everything else on the impulse router is event-driven.

### M.13 Small, isolated, no dependencies

- **`yawAccel` 13 -> 12.** Steering judged tight and not twitchy; the owner wants it a single unit
  slower to turn. No other coupling.
- **Strafe camera view retune** after the camera update changed it. Owner's own task.
- **Tap jump slightly less floaty.** Falls out of M.6 rather than needing its own change.

---

## Tier 0 — Verification debt

### 0.1 ~~Play-mode verification of six bug fixes~~ — VERIFIED AND CLOSED 2026-08-08
**Actually run in play mode, not closed by judgement.** A second AI vehicle was spawned from
`HoverCar_AI.prefab` to provide the target the scene lacks, and `PlayerHoverInput` was disabled and
its backing fields written directly to drive the weapon system.

| Fix | Result |
|---|---|
| Projectile layer + matrix; missiles detonate on enemy hulls | **PASS.** Missile fired at a pinned target 45m out: target knocked from rest to **21.24 m/s**, zero missiles left in scene. Detonation on a hull, not the six-second lifetime fuse |
| `EmpProjectile` moved to `FixedUpdate`; still acquires, steers and freezes | **PASS.** EMP fired at the same target: `IsEmpFrozen` went true. Full flight path exercised, not just reflection |
| HardLock commit-and-hold | **PASS, and it was never broken.** Reaches `Locked` at **1.51s** against a configured `lockAcquireTime` of 1.5, `LockProgress` peaks at 1.00, release launches (1 missile in flight) and the missile hits: target knocked to **75.61 m/s** |
| Rocket self-splash owner exclusion | Unchanged. Already proven by direct call in both directions on 2026-08-07 |
| AI `lockTargetLayers` | Nothing to verify; the AI carries no missiles |
| Boost tick ordering | Already partly closed in play mode 2026-08-07 |

**`saw_Committed` reads false and that is a sampling artefact, not a failure.** `Committed` is a
one-frame pulse that fires, launches and resets to `Idle` inside a single `Update`, so an
`EditorApplication.update` sampler can miss it entirely. The launch is proven by the missile
existing and by the target moving, not by catching the state.

**What is still not verified, and cannot be by measurement:** whether `selfImpactScale` 0.5 is the
right rocket-jump feel, and whether the 10ms boost ordering shift is invisible. Both are feel
questions. See also 1.3: these weapons currently deal zero damage, so only knockback was exercised.

**A new measurement trap came out of this, added to `CLAUDE.md` > Recipes.** The first HardLock run
held fire for 400 editor ticks and reported a failure to lock. At ~285fps that is 1.4 seconds
against a 1.5 second requirement. **Tick counts are not time.** Anything gated on a configured
duration must be driven from `realtimeSinceStartup`, or the test invents a bug.

<details><summary>Original item, kept for the record</summary>

Six bugs were fixed on 2026-08-07 and every one was verified by **compilation plus edit-mode
probing only**. None has been run in play mode. Two are behavioural changes to physics-tick code
and deserve a live confirmation before the tuning pass leans on them.

| Fix | How it was verified | What is still unverified |
|---|---|---|
| Rocket self-splash owner exclusion | `TryHit` called directly: rocket inside own hull rejected, same rocket with `owner=null` still hits, enemy hull still hit | The rocket-jump feel; whether `selfImpactScale` 0.5 is the right number |
| `EmpProjectile` moved to `FixedUpdate` | Reflection: `Update` absent, `FixedUpdate`/`Steer` present | That an EMP still acquires, steers and freezes in flight |
| Projectile layer + collision matrix | Layer/mask read back; sweep mask `-1` -> `-311` | That missiles still detonate on terrain and on enemy hulls in motion |
| AI `lockTargetLayers` 0 -> 64 | Field read back after scene save | Nothing to verify; AI has no missiles |
| HardLock commit-and-hold | Not verified at runtime | Everything. Needs two targets to exercise, which the scene does not have |
| Boost tick ordering | **Partly closed 2026-08-07.** Play mode: holding boost raises the live forward cap 60 -> 60.86 -> 61.71 as `boostLerp` blends in, and `DRIVE` engages airborne only while boosting, both correct | That the 10ms shift is invisible in feel |

**Also confirmed in that session, incidentally:** hover holds 6.94-7.03m against a `hoverHeight` of
7 while driving; the over-speed bleed works airborne and is the only force acting above the cap;
below the cap while airborne genuinely nothing acts (pure inertia); and drive/forward-drag mutual
exclusion holds at full throttle.

**Two measurement traps found the hard way, worth adding to the recipes:**
- **Teleporting with `transform.position` silently does nothing.** The vehicle Rigidbody uses
  interpolation, so the solver overwrites the transform from its own pose every frame. Use
  `rb.position`. A test that appeared to show the bleed never firing was actually a teleport that
  never happened.
- **Wall-clock time between MCP calls is far longer than the eval itself** (thinking time), so any
  state set in one `eval` and read in the next has had seconds to evolve -- long enough for the
  craft to fall 400m or drive into a wall. Multi-phase measurements must run inside ONE eval via
  the `EditorApplication.update` sampler, pinning `rb.position` each tick if the craft must stay
  put.

**Done looks like:** one play session per row, or a scripted `AIHoverInput` rig (see
`CLAUDE.md` > Recipes) driving each case. HardLock needs a second target spawned to test at all.

</details>

### 0.5 ~~CSV capture for A/B tuning~~ — PARTLY DONE 2026-08-08
`FrameSpikeWatch` (see `CLAUDE.md`) now writes `PerfLogs/*.csv` automatically on stop, carrying
time, speed, forward/lateral, tilt, grounded, support, downed, seconds-since-recovery, the four
blends, particle counts, allocation rate, a CPU benchmark and GC deltas.

**What it does NOT do, and what the original item asked for:** it samples on spikes and on a
2-second heartbeat, not per physics tick, and it does not record the live forward/lateral caps or
the acting-force state. So it answers "what was happening when the frame died" but **not** the
tuning comparisons this item was actually created for: the boost ramp, drift speed bleed and jump
arc still cannot be graphed against each other.

Either extend it with a per-`FixedUpdate` tuning mode, or accept that the jump arc is now measured
well enough by the impulse table in `CLAUDE.md` and narrow this item to boost and drift.

**Partly overtaken 2026-08-11 by `MotionTrace`**, which does sample per `FixedUpdate` and does record
signed forward/lateral/vertical velocity, yaw rate and support. What it still does NOT record is the
live forward/lateral CAPS or which force branch acted, which is the half of this item that would let
the boost ramp and drift bleed be graphed against each other. Propulsion already captures both for
its gizmo in `_dbgFwdCap` / `_dbgLatCap` / `_dbgDrive` / `_dbgDrag` / `_dbgBleed`, so closing this is
now mostly a matter of exposing those rather than computing anything new.

### 0.6 Consequences of the 2026-08-08 tuning session
Two of the six original entries were verified in a play-mode pass the same day and are recorded
here with their numbers. **What remains is four items, and every one of them is a feel judgement or
needs a human, so no amount of scripting will close them.** That is the honest boundary of what
measurement can do for this list.

Verified 2026-08-08 in a play-mode pass:

| Claim | Result |
|---|---|
| Six vehicles at once, the real performance question | **RE-MEASURED 2026-08-11 with `MotionTrace`, and the verdict needs qualifying: the MEAN was never the story.** Within one session, 1 vehicle then 5 AI spawned: mean 3.35 -> 4.51ms (+35%) and p50 3.27 -> 3.92, which reproduces the original finding. But **p90 3.71 -> 7.35 (+98%) and p99 5.06 -> 10.47 (+107%)**, and frames over 8ms go from **0.1% to 7.5%, a factor of 75.** The 2026-08-08 pass reported mean/p95/max only and so read as +24%; p95 genuinely does move little, and the degradation lives further out in the tail. That matters here more than elsewhere because this project's felt problem has consistently been consistency, not throughput. Against frame budgets at six vehicles: **60Hz misses 0.05%, 90Hz misses 0.57%, 144Hz misses 12.0%, 240Hz misses 32.7%.** Physics is NOT the constraint: the solver never fell behind (max 2 catch-up steps, 0.12% of frames above one). Allocation and GC are not either (2.8-5.6 MB/s, one gen-0 collection per 30s window, against ~5 MB/s of editor idle). The cost is main-thread per-frame work, and particles scale with it: 6 systems / 32 particles at one vehicle, **35 systems / 238 particles at six**, all before a shot is fired. Original 2026-08-08 figures retained: **PASS, comfortably.** 1 vehicle 3.48ms mean / 4.08 p95; six vehicles **4.33ms mean / 5.13 p95 / 6.98 max**. That is +24% for 6x the vehicles, so roughly **0.17ms of marginal cost each** against a mostly fixed frame. Twelve would still sit near 5ms. **Caveat: AI driving only, no combat, because `AIHoverInput` never sets `FirePressed` (see 1.6). Particle collision at scale is still unmeasured** |
| The support fade, which is the mechanism behind the ledge claim | **PASS, exactly as designed.** Pinned at measured heights above rest: 0.00m support 1.000, 0.25m 0.667, 0.50m 0.333, **0.75m 0.000**. `IsHoverGrounded` stays true until between 2.40m and 2.60m, confirming the 2.5m sensor band it used to be gated on. So at 60 m/s the "still acting grounded" distance is ~12m rather than ~21m, which is the claim, within a metre |

Still genuinely open:

| Claim | Status |
|---|---|
| Bumpy terrain may have improved for free, since leveling now fades as you crest | **Untested.** Could equally read as less planted. Owner had deprioritised bumps because arenas are planned smooth |
| The drift angle ceiling reads as settling rather than hitting a wall | **Untested at the shipped `maxDriftAngle`.** Feel question. See 2.3 |
| Boosted strafe at 60 omni does not cause strafe camping | **Untested and untestable by script.** Needs a human under combat pressure. See 5.11 |
| Weapon physics beyond knockback | **Partly closed.** Missile, EMP and HardLock detonation and knockback are now verified (0.1). The Shotgun, Machine Gun and Chain Gun particle paths are not, and 1.3 means damage is zero on most of them anyway |

The rare 400-550ms stalls also remain unexplained. `FrameSpikeWatch` now prints a CPU-throttling
verdict on stop, which will settle whether they are thermal, and that costs nothing but one normal
playtest.

### 0.7 The periodic frame hitch is unexplained
A very regular doublet, a ~9ms frame followed 0.03-0.08s later by an ~18-20ms one, recurred at 25.2,
27.0 and 27.5 second gaps through the 2026-08-11 session, at unrelated speeds and locations.
Regularity that clean means a process on a timer rather than anything the craft does. Adobe
`CoreSync` / `AdobeCollabSync` were sampled and ruled out (trap 18). `FrameSpikeWatch` also reported
the CPU benchmark drifting +35.5% worst-case in the same session, so thermal or power state is still
live as a contributor.

**The decisive test is a build, and it is cheap.** Editor play mode is not a performance measurement
and the editor plus the MCP bridge are both untested suspects. `MotionTrace` runs unchanged in a
player. If the doublet vanishes in a build, this closes as tooling and stops being worth attention.

### 0.8 ~~Floor collisions, unattributed~~ — CLOSED 2026-08-11, they are hard landings
`MotionTrace` v1.1 added contact names and v1.3 added world horizontal speed, and together they
settle it. The 27 contacts in the 2026-08-11 session are **hard landings from big jumps, not the
craft catching on geometry.** They are against `flatarea`, `loop`, `bridgearea`, `8trackterrain`,
`mountainroad` and `mountain5`, essentially all with contact `normal.y` of 1.00, at impulses of
44k-91k Ns. At 1000 kg an 90k Ns impulse is ~90 m/s of arrested velocity, which matches the measured
descent rates: the craft was coming down at **60 to 95 m/s**. Exactly two contacts are anything else:
one `walls` graze at `normal.y` 0.00 and 11 m/s, and a `mountain4` crash at 85-89 degrees of tilt
that shows up as a flip recovery.

**So the convex-hull theory was wrong and is dropped.** The suspicion was reasonable (the hull on
`car.fbx` does carry a simplification warning) and the data simply does not support it: a bulging
hull would catch on walls and edges, and there are none. What the numbers actually describe is a
craft being flown off very large jumps and landing hard, which is the game working.

Nothing here needs fixing. Retained only because "26 severe velocity discontinuities" was alarming
before it was attributable, and the next person to see that number should not re-investigate it.

<details><summary>Superseded original entry</summary>

### 0.8 Nineteen floor collisions in one session, unattributed
`MotionTrace` v1.1 recorded 19 contacts in the 2026-08-11 session and **all 19 were floor-normal
(normal.y > 0.7), zero wall-ish**, hardest 89894 Ns against `mountain5`. The earlier session showed
26 velocity discontinuities past 1000 m/s^2, confirmed against `rb.linearVelocity` rather than
inferred, including one at t=42.82 where speed fell 83.19 -> 60.67 m/s in a single 10ms tick with
lateral velocity jumping 6.31 -> 32.51 **while fully grounded at support 1.00**.

**The open question is whether these are ordinary hard landings or the craft catching on terrain it
should be hovering over.** A hover craft riding ~4.7m up should not be making floor contact often,
and an all-floor split with no walls is not what aggressive racing into scenery would look like.
Suspect worth checking first: the Body collider is a convex `MeshCollider` off `Assets/RVP/Models/
Vehicles/car.fbx`, and the console baseline carries a convex-hull warning on that mesh. A hull
simplified past 255 polys can bulge beyond the visual mesh, which would make the craft catch on
geometry it visually clears. **Not investigated.** The data to settle it is already being captured.

</details>

### 0.9 Boost framing: measured, not judged
Boost roughly triples apparent longitudinal motion. `rel_along`, the rate the craft moves toward or
away from the camera in frame, runs mean 1.01 m/s with boost off and **2.88 with boost on**, peaking
at 155.3 during air+boost. This is the v2.5 boost framing doing exactly what it was authored to do,
not a defect, and it was ruled out as the cause of the air-control whip (the two worst whips both had
`boost` exactly 0).

**But nobody has judged whether 2.88 is the right number.** It is a feel question and the knobs are
`CameraBoostTuning.zPullBack`, `zLagOnEngage` and `fovOvershoot`. Worth deciding deliberately now
that the whip no longer masks it, since the owner's own hypothesis blamed boost and that instinct may
be tracking this rather than nothing.

### 0.10 Hard steering plus brake at speed produces a 78 degree slide. Decide if that is wanted
Found while chasing a landing marker that turned out not to be a camera fault at all. Measured
2026-08-11 at t=88.47: the craft lands at **84 m/s**, the player holds **full steering lock for 0.6s**
while pulling **full reverse** (which doubles as the brake), and the craft ends up travelling **78
degrees off its own nose**, lateral velocity building to **-56.7 m/s**, still doing 43 m/s.

**`drift` was 0.00 for the entire manoeuvre.** So this is the drift pose without the drift button,
reached by steering and braking alone, and it costs none of what drift costs. Compare `HoverController_Propulsion`
v1.8, where drift was deliberately given an equilibrium (`maxDriftAngle` fading yaw authority as the
slide widens, `driftLateralDamp` closing it) precisely so the angle could not run away. Ordinary
cornering has no such limiter, and at these speeds it apparently does not need the button.

**DOWNGRADED 2026-08-11. The "drift is competing with something cheaper" framing was wrong and is
withdrawn.** The owner challenged it on the grounds that there is no increased turn rate and no
banking, so a line should not be holdable this way, and re-reading the measurement shows they are
right on every point that matters.

- **It is not free. It is the most expensive thing in the game.** The craft went 84 -> 43 m/s during
  the manoeuvre. It costs 41 m/s of actual velocity, where drift costs acceleration and keeps speed.
- **It is not holdable.** 0.6 seconds, and it decays on its own as `lateralDamp` bleeds the lateral
  component and yaw brings the nose round.
- **It is not reachable in normal driving.** It happened at 84 m/s, above `topSpeed` 60, only because
  the craft had just landed from a mountain drop still carrying speed.

Mechanism, for the record: braking kills the velocity component ALONG the nose and does almost
nothing to the component ACROSS it, since lateral velocity only bleeds through `lateralDamp` 1. So
full reverse plus full lock opens the heading-versus-velocity angle by removing one side of the ratio
rather than by adding slide. No extra turn rate is needed, which is exactly why it looked impossible
from the cockpit.

**What survives is narrow:** at speeds above the normal cap, a transient wider than `maxDriftAngle`
60 is briefly reachable. Whether a decaying 0.6s window is usable for aiming is a genuine question
and worth one test with weapons live, but this is a curiosity rather than a rival to drift.

**It does NOT gate the drift work in M.7.** The owner has never experienced it in play. The camera
behaves correctly throughout (it tracks the nose, which is what grounded framing is supposed to do),
so nothing here argues for a camera change either.

Knobs if it should ever be narrowed: `lateralDamp`, and whether `maxDriftAngle`'s equilibrium should
apply to ungated slides too.

### 0.2 Nothing is committed
Everything from the audit session is uncommitted on `master`: 4 docs, 11 scripts, 1 new script,
7 data assets, 3 prefabs, 2 ProjectSettings files, 1 scene. **Commit before the tuning pass**, so
tuning changes are separable from structural ones in history.

**Grown since, and still uncommitted as of 2026-08-09.** The camera overhaul added
`HoverCameraController` v1.2 -> v2.4, five `Tuning/Camera*Tuning.cs` classes,
`CameraPreviewState.cs`, `Editor/HoverCameraControllerEditor.cs`, plus scene and prefab changes
(camera tuning values, SMAA on the Main Camera) and a Quality setting (VSync every V blank). The
longer this waits the more the structural audit work and the tuning work are tangled in one
history, which is the exact thing this item was created to prevent.

### 0.3 There are no tests, at all
No asmdef, no test assembly, no EditMode or PlayMode tests anywhere in `Assets/` outside the RVP
third-party folder. Every regression so far has been caught by hand-driven measurement, which is
why several bugs survived multiple sessions. Not urgent, but the projectile helpers
(`ProjectileSweep`, `WeaponImpact`, `TargetingScan`, `HoverMath`) are pure static functions and are
the cheapest possible place to start.

---

## Tier 1 — Blocks the prototype checklist

The checklist item "Basic combat loop (primary fire + pickups)" is blocked by 1.1 through 1.4
together. None of them is independently sufficient.

### 1.1 Death is terminal; respawn is a stub
`VehicleHealth.HandleDeath()` sets `IsAlive = false`, fires `OnDeath`, and calls
`gameObject.SetActive(false)`. Once disabled the vehicle's own MonoBehaviours stop, so it cannot
revive itself; something external must hold the reference and call in. **Nothing does.** There is
no match manager, round system, or spawn controller.

`VehicleHealth.Respawn()` exists but has no callers, and is incomplete even if called. It sets
health, `IsAlive`, and re-enables the object. It does NOT:

- reset position or velocity, so you respawn where you died at whatever velocity you had, possibly
  inside geometry
- fire `OnDamaged`, so `VehicleHUD`'s health bar stays at 0 until the next damage event
  (`HandleDeath` set it to 0 and nothing sets it back)
- reset energy or ammo
- grant invulnerability (see 1.2)
- reset `AIHoverInput._state`, which `HandleDeath` latches to `Dead` and nothing clears. **A
  respawned AI would be inert** -- its `Update` hits `case AIState.Dead: ZeroAllOutputs()` forever.

`EnemyHealthBar` is the only thing already respawn-safe (re-syncs in `OnEnable`).

**Done looks like:** a decision on who owns respawn (match manager vs the vehicle itself), spawn
point selection, and a `Respawn` that resets the full vehicle state including AI FSM state and HUD.

### 1.2 Post-respawn invulnerability does not exist
`VehicleHealth.SetInvulnerable(float)` has no callers. The machinery is entirely live --
`_isInvulnerable`, `_invulnerableTimer`, the `Update` tick, the `TakeDamage` early-out, and an
`[INVULN]` gizmo label -- it simply can never be switched on. Both `CLAUDE.md` and `VehicleHealth`'s
own header describe a post-respawn grace period that has never run.

Separate clearly: **shield invulnerability works.** It goes through a different path
(`TakeDamage` checks `_shield.IsActive` directly). Only the timed grace is dead.

Pairs with 1.1: respawning without it is spawn-camping by construction.

### 1.3 Four of six implemented weapons deal zero damage
`WD_Missile`, `WD_SoftHomingMissile`, `WD_HardLockMissile` and `WD_Shotgun` are all at
`combat.damage: 0`. Only `WD_ChainGun` (1) and `WD_MachineGuns` (0.5) do damage.

Not cosmetic: both delivery paths are live and both are being handed zero. `HoverController_Weapons`
passes `def.combat.damage` to projectiles, and `ParticleWeaponCollision` reads it directly.

The combat loop can therefore neutralise a player completely but never finish them. That is a
coherent design if chosen deliberately and a hole if not. The inspector now warns on any definition
sitting at 0.

**Watch out when picking numbers:** for ParticleSystem weapons `damage` counts PER PARTICLE, so the
Shotgun's 30-pellet burst applies it thirty times.

**Blocks:** 1.1 is pointless without this (nothing can die), and self-damage suppression added in
the 2026-08-07 rocket fix only matters once this is non-zero.

### 1.4 Every weapon has unlimited ammo
All six definitions have `maxAmmo: 0`, which `WeaponSlot.Initialize` turns into `currentAmmo = -1`
(unlimited). The design specifies limited pickup ammo for everything except the Machine Gun.

Knock-on effects while this stands:
- `RefillAmmo` is a no-op (see 3.1)
- `OnAmmoDepletedForSlot` is **unreachable**, not merely unsubscribed -- both invocations sit inside
  `if (slot.currentAmmo > 0)` and `currentAmmo` is permanently `-1`
- pickups have nothing to restore

### 1.5 `PickupManager` is unwritten
The only planned-and-entirely-absent gameplay module. Design calls for Ammo, Energy Cell and Health
Repair pickups, with placement as the primary mechanism for forcing engagement (powerful pickups in
exposed positions). Depends on 1.3 and 1.4 to have anything meaningful to give.

### 1.6 The AI cannot fire anything except slot 0
Deferred on 2026-08-07 because the AI is currently a punching bag, but recorded because it is real
and will block AI work.

`AIHoverInput` never sets `FirePressed`, so it cannot fire any `SingleShot` or `Missile` weapon --
including the Shotgun it actually carries in slot 1. It also never sets `CycleWeaponNext/Prev`, so
it never leaves slot 0 regardless. Two independent causes; fixing one alone changes nothing.

Net effect: the AI's entire offense is the Machine Gun at 0.5 damage per pellet, 8 pellets/sec.
It carries no Chain Gun, which is the only weapon with a reasonable time to kill.

Also deferred inside `AIHoverInput`: strafe, drift, jump, shield, EMP. All hardcoded false/0.

---

## Tier 2 — Feel and feedback

### 2.1 ~~A denied jump is completely silent~~ — WIRED 2026-08-09, feel unjudged
`OnJumpDenied(bool)` is raised in both `FireGroundedJump` and `FireAirJump` (the bool distinguishes
grounded from air) and used to have **no subscribers**. The player pressed jump, nothing happened,
and there was no way to tell "out of energy" from "input didn't register" or "still in the 0.2s
post-land lockout".

`jumpGroundedEnergyCost` is 25 against a 100 pool, so denial arrives on the fourth consecutive jump.
This is a common state, not an edge case.

This matters more than a normal missing cue because **"Energy as Tempo" is one of the six design
pillars.** A resource the player is meant to manage as their primary tempo tool currently gives no
feedback when it runs out. Cheapest real improvement to feel on this list.

**Shipped mechanism:** `HoverCameraImpulseRouter` (phase 2 of the camera plan) subscribes it to the
`Impulse_JumpDenied` channel: `deniedJumpGroundedVelocity` 0.35 and `deniedJumpAirVelocity` 0.25, thrown down the
chassis axis so a denial while banked still reads as failing to rise. The air value is smaller
deliberately, because the token is not consumed and the failure is not final. Audio is out of scope,
which is exactly why this landed on the camera.

**Still open: whether it reads as "out of energy" rather than as a glitch.** Owner's first pass on
2026-08-10: "just feels bad". Unlike the crash and EMP, this one was NOT the direction bug (a denial
punches down, and world-down and screen-down are nearly the same vector), so it is a genuine shape
problem. Changed from Recoil over 0.12s to **Bump over 0.18s**: 0.12s is seven frames and reads as a
frame hitch, and Bump returns to where it started, so it lands as a lurch rather than a one-way
shove. **Unjudged.** If it still reads wrong the next lever is direction rather than duration: a
denial is a failure to LAUNCH, so a brief up-twitch that falls back may be more legible than a
downward sag. Judge it by draining under 25 and pressing jump, not with the F11 hotkey, since the
hotkey fires without the empty meter that gives it meaning.

### 2.2 ~~EMP launch has no acknowledgement~~ — WIRED 2026-08-09, feel unjudged
`HoverController_EMP.OnEmpFired` is raised once and used to have no subscribers. No audio, no HUD
cue, no cooldown indicator. The projectile carries its own particle visual so the shot is visible,
but EMP costs 70 of 100 energy -- the most expensive ability in the game, and one that empties the
meter you need in order to disengage. A commitment that large should confirm itself.

**Shipped mechanism:** its own impulse channel on the router, `Impulse_EMP`, a Recoil over 0.22s so
the launch reads as a one-way shove. `empVelocity` 1.2, thrown backward along the chassis.

Owner's first pass on 2026-08-10 was "feels bad", and **most of that was the screen-space direction
bug** (see `CLAUDE.md` trap 11): the craft's world heading was being applied as a screen direction,
so the recoil went sideways instead of backward. Now measured at screen fwd -0.93, up -0.37, right
0.00, which is a clean pull away from the craft. Duration also cut from 0.35s, which was a wallow.
**Needs re-judging after the fix.** Whether 1.2 is the right weight for the game's largest single
energy commitment is still open; it is currently a little over half the landing punch.

### 2.3 ~~Drift feel is untuned~~ — DONE 2026-08-08, with two open questions
Tuned in the 2026-08-08 session and confirmed good by the owner. Drift is now an **aiming tool**:
the gap between heading and velocity is the product, bought with acceleration. Causes, fixes and
the two-knob tuning model are in `CLAUDE.md` > Propulsion v1.8. `driftLateralDamp` is 0.25,
`driftForwardDamp` 0.3, plus `maxDriftAngle` and `driftHopImpulse`.

**SUPERSEDED 2026-08-11 by M.7, which carries the live drift work.** Judged this session: the aiming
purpose works (line of sight held while velocity runs at a different angle), and the target is now
named as **the Crash Team Racing power slide without the boost mechanics.** Two new requests, a
duration-based speed bleed and more path curvature control, live in M.7. Note the live values are
`driftLateralDamp` **0.3** and `driftHopImpulse` **10**, not the 0.25 and 5.5 written below; the
higher pair is what was committed on 2026-08-08 and approved, so the numbers below are the stale
ones.

Still open, both feel questions rather than bugs:
- **Is the angle ceiling settling or hitting a wall?** At low `driftLateralDamp` the slide runs
  right up to `maxDriftAngle` and stops. Raising damping pulls the balance point below the cap so
  it eases in instead: at a 45 cap, damping 0.25 settles at 42 and damping 1.0 settles at ~34.
  Same cap, completely different character.
- **No exit payout exists and that is deliberate.** The owner rejected a boost reward: drift is
  about angle, not speed. Revisit only if the angle stops feeling worth the acceleration. **Still
  true 2026-08-11**, and reconfirmed: the CTR reference was given explicitly WITHOUT its boost
  mechanics. A spinout consequence was also explicitly rejected, since it is a hovercraft.

Invariant still maintained by hand and still unenforced in code: `minDriftSpeed` MATCHES
`strafeTopSpeed`, so outpacing strafe is what earns the drift. Both 40. `VehicleTuningProfileEditor`
warns if they diverge.

### 2.4 Hard-landing lands with no dust
`landingDustPrefab` and `landingDustHeavyPrefab` are both null on `HoverCar_Prototype` and
`HoverCar_AI`. They pointed into `Assets/JMO Assets/` (WarFX), which is no longer on disk.
`HoverVehicleVFX.HandleHardLanding` null-checks and returns.

The whole hard-landing feature therefore currently delivers only the camera punch and the spring
give. The event, the severity split, the heavy/standard threshold and the ground raycast are all
live and correct; only the two prefab slots are empty. **Blocked on art, not code.**

### 2.5 Missile detonations have no explosion VFX
`blast.explosionPrefab` is null on all three missile definitions, same removed WarFX package.
`RocketProjectile` null-checks before instantiating, so this fails silently rather than erroring.
Detonations currently produce no visual at all. **Blocked on art, not code.**

Note the dependency now runs the other way round from how it used to: the definition states
`splashRadius` and the VFX is authored to match it, not the reverse.

### 2.6 Boost reads flat — BUILT 2026-08-10, UNJUDGED
Owner note, 2026-08-08 playtest: "the boost feels somewhat mid". The multiplier was never the
problem: `boostSpeedMultiplier` and `boostAccelMultiplier` are both 1.5, a real 50% increase, so
this was always presentation.

> **DOCUMENTATION DEFECT, found 2026-08-11. The cut below never happened in the asset.**
> `boostBlendSeconds` has been **0.35 in every commit from `4a34f21` (2026-08-01) through today**. It
> has never been 0.125 and never been 0.15. So the bullet beneath, including its measurement of peak
> surge moving 0.56 -> 0.77, describes a value that was set live in the inspector and never
> persisted. Everything downstream that was "tuned against" the cut was in fact tuned at 0.35.
>
> **This is the second instance of the serialized-value-wins trap**, after `travelHeadingMinSpeed`
> being lowered to 3 in code while the prefab kept 8. Both times a real change was believed to be in
> effect and was not. When a tuning value matters, read it back from the ASSET, not from the doc and
> not from the inspector during play.
>
> Owner decision 2026-08-11: **leave it at 0.35 and judge boost as-is.** If boost still reads flat
> after M.5 lowers the multiplier, 0.15 is the first thing to try, ahead of any camera value.

Done, and not repeated here (see `CLAUDE.md`):
- ~~**`boostBlendSeconds` cut to 0.125** by the owner.~~ That did not stick: the live value was
  found at **0.35** on 2026-08-10, which is the "took a third of a second to arrive and read as
  gradual rather than as a kick" case this item opened with. ~~**Now set to 0.15.**~~ It is the
  highest-leverage knob in the whole boost system for two reasons: time-to-peak on the camera
  transient tracks the thrust ramp almost exactly, and the transient is measured as the gap between
  the boost level and a slower copy of itself, so a snappier ramp opens a wider gap. The cut took
  peak surge from 0.56 to 0.77 with no camera tuning at all. **This is the one boost change that
  touches gameplay rather than presentation** (it is how fast thrust arrives), so if the car now
  feels twitchy to commit, revert THIS before touching any camera value.
- **FOV kick added**, scaled by `BoostLerp` so it inherits the blend and can never disagree with the
  thrust about when boost started. Written to both vcams against their own bases, since mode
  switching is by priority and the brain blends them. Later gated on forward motion, because it was
  also firing while reverse-boosting.

**The rest of the boost language is BUILT 2026-08-10, and unjudged.** All four planned terms shipped
as phase 3 of the camera plan: a sustained Z pull-back, an FOV overshoot that settles rather than a
step, extra camera lag at the moment of engaging, and a return slower than the entry. Mechanism and
measurements are in `CLAUDE.md`; the short version is that the camera now keeps its own boost
envelope instead of reading `BoostLerp` directly, because an overshoot and an asymmetric release
cannot be written as a function of a value that only says how far into boost you currently are.

**Tuned 2026-08-10 against the pillars**, as a starting point for the owner rather than a final
answer. Pillar 1 makes boost a weapon, so the commit should feel like firing one; pillar 2 expects
the player to be FIGHTING while boosting, so the held state has to stay readable. Hence a big
transient over a modest sustained: `fovIncrease` 4 with `fovOvershoot` 12, `zPullBack` 0.7 with
`zLagOnEngage` 2.5, `releaseSpeed` 3.5, `settleSpeed` 3.5.

Measured live at 40 m/s: FOV 65 to a peak of **78.2 about 0.16s after engaging**, settling to
**69.0**; camera pulling back **2.58m** at the peak and settling to **0.71m**; release to 5% in
**0.61s**, roughly four times the entry. The transient is down to a fifth of its peak within half a
second, which is the pillar-2 requirement: dramatic to commit, fightable again almost immediately.

**Judge it on how boost reads at the MOMENT OF ENGAGING**, which is entirely camera. Absolute sense
of speed is not, and is partly blocked on arena props (6.2).

Two things to watch while judging. **The reverse gate reads FORWARD speed only**, so boosting
sideways in strafe produces no lens change at all; that may be right, since the gate exists to
suppress the reverse case, but nobody has judged it. And **`boostBlendSeconds` at 0.35 is currently
capping the kick** (see above).

Also unaddressed and arguably the real ceiling: **perceived speed is structurally low** on this
chassis (see 6.2). 60 m/s over a 6.88m hull is 8.7 body-lengths per second against 15-16 for a fast
road car.

If the multiplier does go up afterwards, mind two interactions: boosted top speed is already 90
against `hardLandingMaxSpeed` 85, so severity saturates; and boosted strafe already reaches 60 in
every direction (see 5.11).

### 2.7 Automatic weapon fire rates are far too low
Owner note: both automatics "need to increase the firing speed a bunch to keep up with the reticle".
Live emission is **8/sec per Machine Gun barrel and 20/sec on the Chain Gun**.

Related: 3.6 records `WD_MachineGuns.combat.fireRate` at 0.01, which is a separate value from the
emitter rate. Both need to move together, and the definition is the single author since
`ParticleWeaponCollision.ApplyDefinitionToEmitter` pushes it.

**Camera recoil is now wired, per weapon and opt-in.** `WeaponDefinition.combat.recoilVelocity`,
0 on all six assets, read by `HoverCameraImpulseRouter` on `OnWeaponFired` (raised by both the
projectile and the ParticleSystem paths, so either kind of weapon can opt in). The router keeps only
a master switch and a global multiplier.

**Do not tune recoil before this item and 3.6 are fixed.** Recoil lands once per shot, so its right
value is a function of fire rate, and both automatics are currently misconfigured in opposite
directions: the Chain Gun at 20/sec would blur any kick into a tremor, and the Machine Guns at 0.01
would fire one kick every 100 seconds. Any number picked now gets invalidated when the rates move.
The single-shot weapons (`WD_Missile`, `WD_HardLockMissile`, `WD_SoftHomingMissile` and `WD_Shotgun`,
all at 1.5/sec) are tunable today and are the sensible place to start.

#### Deferred: per-weapon recoil CHARACTER, not just strength
Found by playtest on 2026-08-10 and **deliberately not built**, because the weapons are placeholder
and shaping camera feel around them now would be tuning against a moving target. Recorded so the
reasoning is not re-derived.

Only strength is per weapon today. Shape and duration live on the single shared
`Impulse_WeaponRecoil` source, so every gun that opts in shares one character. The owner found that
insufficient immediately: **the shotgun wants a short snap and a missile wants a longer envelope with
a wind-up dip**, which is a difference in kind rather than in size.

**Do not infer the split from `projectileMode`.** It was considered because it happens to separate
the current six correctly, and rejected: mode describes how a weapon spawns damage, not how it
should feel. A railgun would be Instantiated and want a sharp crack; a plasma stream would be
ParticleSystem and want a sustained push; a grenade lob would be Instantiated and want almost
nothing. Worse, the failure would be silent, and the only way to author around it would be to change
the projectile mode, which changes gameplay.

**And do NOT solve it by writing shape and duration onto the shared source before each shot.** That
is the obvious design and it is unsafe: an impulse reads its shape and duration off the live source
component for its whole life, so a still-playing impulse adopts whatever the next shot wrote. Full
mechanism in `CLAUDE.md` measurement trap 14. The consequence for this item is simply that
**different characters need different sources.**

**The shape it should take when it is worth building:** a `recoilCharacter` enum next to
`recoilVelocity`, named for FEEL rather than for camera objects (`Snap`, `Swell`), with one source
per character on the camera and the router picking by name. The weapon says what kind of event it
is; the camera owns how that is expressed, so the weapon definition never learns a camera concept.
Start with exactly the two that were empirically found. `recoilVelocity` 0 stays the off switch, so
the enum only ever picks character. Cost per character is one GameObject, one serialized field and
one enum value, all visible, so do not create them speculatively.

### 2.8 Camera, three separate complaints — TWO CLOSED 2026-08-09, ONE OPEN
From the 2026-08-08 playtest. The first two were closed by the camera overhaul; outcomes are in
`CLAUDE.md` and not repeated here.

- ~~**Drive cam cannot pitch high enough.**~~ **Closed.** It was not a vcam configuration problem.
  Raising the camera also angled it DOWN at the roof, because height and look direction were the
  same knob, so more pitch showed more ground and less horizon -- the opposite of the complaint.
  Split into an orbit that preserves the authored framing plus a look point the stick pushes up and
  forward.
- ~~**Strafe cam feels wrong on jumps.**~~ **Closed.** Vertical-only position damping, so jumps
  soften without touching aim.
- **The reticle jars when aiming on slopes.** STILL OPEN. The HUD ray hits geometry at
  discontinuous depths, and because the ray originates at the vehicle while the projection origin is
  the camera behind and above it, a change in hit depth slides the world point along the aim line and
  **parallax throws it across the screen**. `reticleFollowSpeed` 15 smooths the symptom without
  removing the cause. Decided approach: keep the vehicle-origin ray so the reticle stays honest about
  chassis pitch, and remove the screen position's dependence on hit depth. Confined to
  `VehicleHUD.SyncReticle`; no camera coupling. This is the last phase of the camera plan.

### 2.9 Impacts are under-intense, and big drops do not cost enough control — MOSTLY CLOSED 2026-08-11

**JUDGED 2026-08-11, and the complaint that opened this item has been withdrawn by the owner.** Three
separate confirmations, all by feel in the movement playtest:

- **Crash camera effects: liked.** That judges the crash-shake half, wired 2026-08-09 and unjudged
  since.
- **Hard landing camera punch: liked, keep as-is.** Unchanged from the original note.
- **Tumble on a bad-angle mountain landing: good.** "If I jump off the mountain top and land at a bad
  angle, I can really get knocked around, which is great."

**That last one probably removes the physics half entirely.** This item's remaining work was scaling
`levelingTorqueStrength` by `hardLandingSeverity`, and it existed ONLY because the owner previously
said a mountain drop should tumble more. They now say it tumbles well. The environment rescale is the
likely cause, since doubling it made every drop taller.

**Do not make that change without a fresh complaint.** It was also the only queued reason to modify
`HoverController_Foundation`, which is marked do-not-modify-without-justification; M.1 is now that
justification instead, for an unrelated reason.

Still genuinely open, and NOT judged this session: **taking a weapon hit still produces no camera
feedback at all** (see the paragraph below). Vehicle-on-vehicle collision is also still untested, and
the owner has an easy path to it: park the AI craft and drive into it.

<details><summary>Original item, kept because the reasoning behind the deferred physics change is worth preserving</summary>

Owner: crashes into walls, ground and other vehicles should hit harder; a mountain drop "should lose
more control and tumble more than it does". The hard-landing camera punch is liked and should stay.

The hard landing system today is **feel-only by design**: it suppresses LIFT on a 0.35s taper and
fires the camera impulse, with no damage and no lockout. Leveling torque stays at full 12 throughout,
which is why a badly angled high-speed landing snaps flat instead of tipping you.

**Cheapest real change:** scale `levelingTorqueStrength` by the same `hardLandingSeverity` and timer
that already scale `liftFactor`. Reuses existing machinery rather than adding a system, and it makes
a bad landing angle actually cost something.

**The crash-shake half is DONE 2026-08-09.** `VehicleCollisionRelay` adds the vehicle's first
`OnCollisionEnter` and hands it to the router, which fires the crash channel at
`collisionMaxVelocity` 2.5 scaled between
`collisionMinSpeed` 8 and `collisionMaxSpeed` 35 m/s. Verified with a 45 m/s head-on into a temporary
wall: one shake, impact measured at 27.1 m/s, severity 0.52, and no double-fire on a separate 70 m/s
drop that produced exactly one landing punch and zero collision callbacks. The mass normalisation,
the 0.6 approach-to-impact ratio behind the 35, and the finding that hard landings never generate a
collision callback at all are in `CLAUDE.md`. **Untested by anyone: vehicle-on-vehicle.** The AI
`HoverCar` is on the AIVehicle layer with its own mesh colliders and should route identically, but
that has not been driven.

**Also open: TAKING A WEAPON HIT still produces no camera feedback at all.** Raised by the owner on
2026-08-10, and the answer is no, the crash path does not cover it. Crash shake hangs off
`OnCollisionEnter`, which needs a physical contact, and weapon damage does not arrive that way:
rockets detonate from `ProjectileSweep` before touching, and particle weapons never had a collision
to begin with. So being shot is currently as silent as the denied jump was. The hook already exists
and has no camera subscriber: `VehicleHealth.OnDamaged(currentHealth, maxHealth)`. Note the
signature carries HP rather than damage dealt, so the router would need to difference it to get a
magnitude, or the event needs a third parameter. **Undecided, on purpose:** a shake on every hit is
a real design choice, since it costs aim stability at exactly the moment the player most needs it,
and some games deliberately refuse it. Worth deciding before wiring.

**Still open here: the physics half, which is what the complaint was actually about.** Shake tells
you that you crashed; it does not make a bad landing angle cost anything. That is the
`levelingTorqueStrength` change above, and it is deliberately still unmade because it lives in
`HoverController_Foundation`, which is marked do-not-modify-without-justification.

**Measured context that changes the framing:** no jump can trigger a hard landing at all
(full charge lands at 43.4 against `hardLandingMinSpeed` 58); the system fires only on mountain
drops, confirmed live at 58.0 to 87.6 m/s. If landings should matter more generally, that threshold
is the knob, and it needs the measured pass the owner already asked for.

**Superseded 2026-08-11 by M.6:** the threshold is no longer the only knob. Raising `extraFallGravity`
past roughly 43 brings a full charge jump's landing speed up to 58 and starts triggering hard
landings from jumps for the first time, without touching `hardLandingMinSpeed` at all.

</details>

### 2.10 Energy is permanently tight, and tricks should probably pay it back
Owner: "I feel like I'm always in need of more energy, mostly for boosting and jumping", plus the
suggestion to reward landed aerial tricks with energy.

The budget is tight by construction: boost drains 20/sec against a 20/sec regen with a 1s delay, so
sustained boost is roughly a 45% duty cycle, and every jump costs 25, or 1.25 seconds of boost.

**This is the highest-upside item on the list** because it closes a loop rather than adding a
feature: tricks and drift both feed the resource that powers boost and jumps, which buys more
tricks. It is on-pillar for momentum as the primary skill expression, and Tony Hawk is already in
the inspiration list.

The pieces exist. Foundation tracks attitude, `AirControlWeight` distinguishes deliberate control
from tumbling, the airborne-to-grounded edge is already detected in `ApplyHoverForces`, and
`IsDowned` gives a clean forfeit condition. Integrate rotation while airborne, bank it on landing
upright, lose it on a flip.

Note the owner explicitly rejected a **speed** payout for drift (2.3). An energy payout is a
different question and was left open.

---

## Tier 3 — Known traps and code quality

These are not currently causing visible problems. Each is recorded because it will be misdiagnosed
as something else when it does.

### 3.1 `RefillAmmo` secretly resets cooldown and wind-up
`HoverController_Weapons.RefillAmmo` is implemented as `slot.Initialize()`, and `Initialize()` also
resets `cooldownRemaining`, resets `windUpProgress`, and rebuilds `baseEmitterRates`.

So picking up ammo will **cancel your cooldown and dump your Chain Gun spin-up** -- a 3-second
wind-up erased by driving over a pickup. Nothing in the method name suggests it. Harmless today
because there are no pickups and no ammo; a confusing "minigun bug" the day both exist.

**Fix:** split ammo restoration out of `Initialize`.

### 3.2 `ParticleWeaponCollision` walks the hierarchy per collision event
In `OnParticleCollision(GameObject other)`, both `other.GetComponentInParent<IDamageable>()` and
`other.GetComponentInParent<Rigidbody>()` are called **inside** the per-event loop, although `other`
is invariant across the whole loop. A 30-pellet shotgun burst landing on one target does 60
hierarchy walks instead of 2.

The file's own NOTE defers this to "pooling debt", but the hoist is independent of pooling and is a
two-line change. Do the hoist now; keep the pooling note for later.

### 3.3 The AI steers away from its own target
`AIHoverInput.obstacleMask` defaults to `~0`, so the 5-ray avoidance fan treats the player vehicle
as a wall. At close range avoidance urgency fully overrides waypoint steering, meaning the AI
actively backs away from the thing it is trying to shoot.

Also casts from `transform.position + Vector3.up * 0.5f`, which is inside its own hull.

**Fix:** restrict `obstacleMask` to terrain layers.

### 3.4 ~~`AIHoverInput.OnDrawGizmos` re-casts the whole ray fan~~ — DONE 2026-08-07
Fixed while building the tuning instrumentation. It now draws from hits captured during
`ApplyObstacleAvoidance` and casts nothing; it also gained the `HoverDebugSettings` gate it was
missing (it was the only unguarded debug draw in the project), and its tessellated roam disc is now
edit-mode only, since a filled disc is only useful while placing the area. The equivalent problem in
`HoverController_Weapons` -- an 80-115m `OverlapSphere` per repaint for the soft-homing preview --
was fixed the same way, moved to the game tick and throttled to 10Hz.

**Left open: 3.3, the `obstacleMask = ~0` behaviour bug.** That one is unrelated to cost.

### 3.5 `RocketProjectile.Explode` allocates
Uses `Physics.OverlapSphere` (allocating) while the rest of the project is scrupulously non-alloc:
`RaycastNonAlloc` in the hover springs, `OverlapSphereNonAlloc` in `TargetingScan` and the lock
scan, static reusable buffers in `ProjectileSweep`. Detonations are bursty, so this is garbage
generated at exactly the worst moment.

### 3.6 `WD_MachineGuns.combat.fireRate` is 0.01
That is the `[Min]` attribute floor, giving a **100-second cooldown**. Almost certainly a slider
dragged to the bottom rather than an intended value. Compare `WD_ChainGun` at 20.

Currently invisible: for a ParticleSystem Automatic weapon the emitters play regardless of
`IsReady`, and `maxAmmo` is 0 so no ammo decrements. The only observable effect is that
`OnWeaponFired` fires once per 100 seconds. It breaks the moment the Machine Gun gets ammo (1.4).

### 3.7 `SetActiveSlot` does not skip empty slots
Unlike `TickCycleWeapon`, which walks past slots with a null definition, `SetActiveSlot(int)`
accepts any in-range index. Pointing it at an empty slot leaves the vehicle silently weaponless,
because `Update` early-returns on `definition == null`.

**Still true after Weapons v1.1.** Both paths now share `SwitchToSlot`, so the empty-slot skip only
needs writing once and belongs there. The duplication that hid the emitter-shutdown bug is gone;
this gap survived it.

### 3.8 `SetRecoveryEnabled` has a sharp edge for its first caller
`HoverController_Foundation.SetRecoveryEnabled(bool)` has no callers today, and `recoveryEnabled` is
permanently true. EMP does not use it -- Foundation's `FixedUpdate` early-returns wholesale on
`energy.IsEmpFrozen`, which skips `HandleRecovery` anyway.

Before anything wires it up, note: `IsDowned = recoveryEnabled && (...)`. So
`SetRecoveryEnabled(false)` on a downed craft immediately clears `IsDowned` and hands full control
back to a vehicle lying on its back -- the exact bypass that was closed in Propulsion v1.3. That is
arguably correct (disabling recovery should disable the lockout recovery exists to enforce) but it
is a trap worth knowing.

Minor: it also does not clear `unstickForceTimer`, so an in-flight unstick push finishes.

### 3.9 Scene wiring noise
Cosmetic, no runtime effect, but misleading when reading the inspector:
- Weapon slots 2/3/4 (the three missiles, all `Instantiated` mode) each have a particle emitter
  assigned, which that mode ignores. **No longer purely cosmetic since Weapons v1.1:** switching
  away from a slot now stops its emitters and rewrites their emission rate, so slot 2 will reach
  into the **Shotgun's** emitter (which is what it actually points at) on every switch. Harmless
  today because the Shotgun is burst-only, but it is now a live coupling rather than dead data.
  Slots 3 and 4 additionally hold a null element in the list
- Slot 5 (Chain Gun, `ParticleSystem` mode) has a muzzle point assigned, which that mode ignores
- `blast.damageLayers` is `Everything` on `WD_ChainGun`, `WD_MachineGuns` and `WD_Shotgun`, none of
  which read the blast section

(`WD_Shotgun`'s stale `missileFireMode: 2` looks like it belongs here but does not -- it is a closed
decision, recorded in `CLAUDE.md`.)

---

## Tier 4 — ~~Deletion candidates~~ — DONE 2026-08-08

All three deleted. Compiles clean. Outcome recorded in `CLAUDE.md`; not repeated here.

`OnMissileLocked` (duplicated `OnMissileLockStateChanged`), `Propulsion.StrafePitchLimit` (no
callers), and the `OnRegenStarted` event plus its empty `VehicleHUD` handler, both sides.

If a lock tone or a HUD pitch indicator is ever wanted, re-add deliberately rather than restoring
these: each was dead because the consumer that justified it never existed.

---

## Design decisions with no engineering blocker

These need an owner call, not code. Listed with what is already measured so the decision is cheap.

### 5.1 Hard Lock is not designed through
**The mechanism works and is verified** (0.1): acquires at 1.51s, commits on release, launches, and
the missile connects. What follows is about what the weapon SHOULD be, not whether it functions.

Stated intent (2026-08-07): a volley of **small, low-damage, low-impact missiles**, lockable onto a
single target or distributed across several. Currently implemented as the degenerate case -- one
lock, one missile -- and tuned as though it were a Rocket Launcher.

Current values are the wrong shape for that intent: `impactForce` 100000 and `splashImpactForce`
50000, identical to the Dumbfire, on a weapon whose identity is that it cannot miss. `turnRate` is
120, below the 160 measured as the threshold for landing a clean hit. `lockConeAngle` is 45 degrees
at `lockRange` 115 -- wide for a single lock, but plausibly right for sweeping across several
targets in a cascade.

The lock now COMMITS to one target and holds it (fixed 2026-08-07), which is the primitive a
multi-target version needs. Extending it means promoting `LockTarget` to a list and looping
`Scanning` back on itself with already-committed targets excluded from the scan; the validity rule
and the release-to-fire trigger carry over unchanged.

**Do not tune this weapon until the design is settled.** Any number picked now is for a different
weapon.

### 5.2 The knock-around pass is only partly applied
The missile force cut sits on Soft Homing, not Dumbfire, by owner decision:
`WD_SoftHomingMissile` is at 55000/28000; `WD_Shotgun` went 3000 -> 1800; `WD_Missile` (Dumbfire)
and `WD_HardLockMissile` both keep the original 100000/50000.

That is GDD-aligned -- soft homing is specced as "low damage and force", the Rocket Launcher as the
primary momentum-disruption tool. But it does mean the Dumbfire keeps the over-scaled value the
audit flagged: 100000 measured a 100.01 m/s delta-v against a `topSpeed` of 60, and a flank hit
fully inverted a stationary craft and threw it about 80m.

**The open question is only whether the Dumbfire keeps that value.** The split itself is already
verified and working as designed; see `CLAUDE.md` > "Missiles are now tunable" for the measured
flip-threshold behaviour of both weapons rather than restating it here.

### 5.3 `destabilizeFraction` has never been tested against a landing hit
Held at 0.15 across all three missiles. It was set **before** detonation worked, so its rationale
was never verifiable -- every apparent hit at the time was really the lifetime fuse firing
splash-only. Detonation now works, so this is measurable for the first time.

**New coupling, recorded 2026-08-07:** it now also governs self-inflicted spin from a rocket jump,
and **self-flips become reachable past roughly `destabilizeFraction` 0.36.** The measurement behind
that number is in `CLAUDE.md` > "Rocket-jump spin is safe at the current `destabilizeFraction`".
Recheck it before raising this value.

### 5.4 `selfImpactScale` default is a guess
Added 2026-08-07 at 0.5 on all six weapons, sized against existing mobility (`airJumpImpulse` 25,
`jumpImpulseMax` 40) so a rocket jump lands between a tap jump and a charged one. Unmeasured in
play. Gives 10.8 m/s from a standstill and 20.1 m/s at full boost.

Note the self-boost scales with launch speed (21.5 -> 40.1 m/s before scaling) because the rocket
does not inherit vehicle velocity, so at 70 against your 60 you close on your own blast. That reads
as skill -- shooting a wall you are charging kicks harder -- but it means the mechanic is strongest
exactly when you are already fastest. If you would rather it were flat, measure falloff from the
launch position instead of the detonation point.

### 5.5 Homing cannot catch a runner, by construction
Missiles fly at 70 against a vehicle top speed of 60. A target running flat out in a straight line
is closed on at only 10 m/s, so **no `turnRate` makes homing reliable against a fleeing craft.**
Homing weapons are therefore tools for punishing players who are turning, fighting or cornered.

If that is not the intent, the value to change is `speed`, not `turnRate`.

### 5.6 `flipRecoverySpeedThreshold` is an unconfirmed guess
Currently 2. The tooltip says so explicitly: 0.5 was too tight for this chassis, 2 is a guess, and
the failure mode is subtle -- a hull that comes to rest on a curved face and micro-rocks keeps
resetting the arming timer and never recovers, which looks identical to recovery being broken.

### 5.7 Vehicle roster is one profile
`VTP_Default` is the only `VehicleTuningProfile` in the project. The GDD proposes a five-vehicle
framework on two independent axes (momentum character, stability vs agility): Bruiser, Interceptor,
Chaos Vehicle, Precision Drone, Transition Specialist. None of the other four exists.

The architecture supports it already -- one asset per archetype, scene refs stay on the
MonoBehaviours -- so this is authoring work, not engineering.

### 5.8 Vehicle scale: deferred, with two gates
The craft is a ~1.6x sedan. Measured dimensions, the consequences, and the reasoning behind not
rescaling are in `CLAUDE.md` > Vehicle Scale; not repeated here.

**Deferred deliberately, not forgotten.** The handling is good and there is nothing built for the
craft to look wrong against, so there is no case for touching it now. Two points where the decision
should be re-opened, because both produce work that INHERITS the scale and is expensive to redo:

1. **Before arena blockout.** Geometry authored at one scale and rebuilt at another is far more
   costly than rescaling a single vehicle. Note the alternative is equally valid and cheaper:
   build the arena TO the craft (corridors ~20m, entrances ~10m per the ceiling-duck measurement)
   and nothing needs retuning at all.
2. **Before the five-vehicle roster** (5.7). Authoring five handling profiles against an unsettled
   scale means tuning five vehicles twice.

If it is ever revisited, the cheapest single improvement is **narrowing the hull**. The 1.75x width
against 1.46x length is what makes it read stubby, and width is the least coupled dimension to the
tuning, since the attitude torques ignore the inertia tensor.

### 5.9 ~~The air jump has never been justified or cut~~ — DECIDED 2026-08-11: KEEP

Owner, 2026-08-11: "I questioned the air jump before, but I think it's worth keeping." Its value is
as a **mid-air juke and trick-height extender**, which is the argument this item was holding open.

Two consequences moved to Tier M rather than closing here:

- **M.8**: the impulse should fire along the craft's LOCAL up rather than world up, so tilting toward
  a wall propels you away from it. That is what makes it a juke rather than only a height extender,
  and it was the owner's one caveat on keeping it.
- **M.9**: its energy cost should differ from the grounded jump's, rather than both sitting at 20.

The old advice to decide this against 2.10 still holds for the COST, not for the existence.

### 5.10 Wall jump / wall riding, speculative
Owner idea: count as grounded when deliberately oriented against a wall with sensors in range.

**Partly pre-built by M.8, 2026-08-11.** Making the air jump fire along the craft's LOCAL up means
tilting toward a wall already propels you away from it, which is most of a wall jump reached from a
different direction and for a different reason. Do M.8 first and re-read this item afterwards; the
remaining gap may be small enough that this stops being speculative.

**Test before building.** It may already partly work: hover points cast along `-point.up`, which
rotates with the craft, so orienting against a wall does put rays on it and the springs will push
off. `IsDowned` is the thing that would block it, and only on CONTACT past `flipRecoveryAngleThreshold`
-- a craft hovering the wall without touching it keeps full control by design, which is already
documented as deliberate in `CLAUDE.md`.

### 5.11 Boosted strafe reaches unboosted drive top speed
Measured 2026-08-08. Strafe caps are boost-scaled, so **strafe + boost + forward gives a 60 m/s cap
in every direction**, equal to unboosted drive-mode top speed, while keeping free aim. The only
thing limiting it is the energy meter, which is a resource constraint rather than a design one.

The owner's stated goal is that drive mode is for fast forward and backward travel and strafe is for
aiming at a reduced, consistent omnidirectional speed, and specifically that players should not
"play the whole game in strafe mode". This is the strongest incentive to do exactly that.

**Left as-is deliberately** pending evidence of actual camping under combat pressure. The lever if
it does happen: stop boost scaling the strafe ceiling. Note that alone would recreate the
paying-for-nothing bug already fixed for reverse boost (energy drained for acceleration into an
unchanged cap), so it would need boost in strafe to become dodge-only, which is a larger behaviour
change than it first appears.

---

## Unimplemented features

### 6.1 Seven of thirteen weapons
Implemented: Machine Gun, Minigun (Chain Gun), Shotgun, Rocket Launcher, Soft Homing, Hard Lock.

Planned, no asset and no code: Sniper / Lightning Bolt, Laser Cannon, Gravity Well / Repulsor,
Bouncing Disc Blade, Floating Proximity Mine, Directional Remote Mine, and the per-vehicle Special.

Note `WeaponType.Mine` and `ProjectileMode` already exist and `TickMine` is implemented (fires at
the muzzle with no launch velocity), so the two mine weapons are closer than the rest.

### 6.2 Arena
The GDD has a full arena philosophy -- track with rooms, three vertical layers, parallel routes at
every major node, corridors as momentum builders, falling as a tactic -- and prototype scope calls
for "a small test arena with ramps and obstacles". `Prototype_Scene.unity` is the only project scene.

One measured constraint to design against: **tunnel and doorway entrances want about 10m of
clearance.** Interiors are safe because the ceiling duck squats the craft; entrances are not. The
reason is a closed limitation recorded in `CLAUDE.md`.

**Blocking a feel question, not just a content one.** Owner, 2026-08-08: "it's hard to tell how high
my ride sits in a mostly empty arena without props". Ride height is 7m and the belly sits ~4.7m up,
higher than a double-decker bus, and there is currently nothing in the scene to read that against.
Perceived speed has the same problem: 60 m/s over a 6.88m craft is 8.7 body-lengths per second
against roughly 15-16 for a fast road car, so it reads about half as fast as 216 km/h sounds. **Both
are judgements that cannot be made until props exist,** and both feed 5.8 (vehicle scale), which is
explicitly gated on arena blockout.

> **REVISED 2026-08-11, and this changes the plan rather than adding to it. The arena is
> deliberately SPARSE.** Owner: "I hadn't really imagined an arena to be dense with props, maybe just
> some environmental hazards and exploding barrels or something, but by no means full of objects on
> the ground."
>
> **So props are NOT coming to fix speed and height readability, and this item can no longer be
> treated as the blocker for either.** The paragraph above assumed the opposite. What is left to read
> speed and height against is ground texture density, road markings, the height and spacing of the
> large structures, and screen-space effects such as the speed lines requested in M.12.
>
> Ride height itself is now **judged good** independently of this (see `CLAUDE.md` > Judged By Play),
> so only the speed half remains open. Note the environment was doubled on 2026-08-11 to match the 2x
> vehicle mesh, which halved how fast the world goes past you at a given m/s and makes M.5 more
> urgent, not less.
>
> Two constraints from this session that any blockout must respect: the owner wants **platforms you
> can bump from underneath, not spaces to duck under**, so the ceiling duck is not a design
> dependency; and the existing figure-eight loop is the reference for the drift curvature work in
> M.7.

### 6.3 No match flow
No menu, no match start or end, no scoring, no round structure. Combined with 1.1, there is
currently no way for a match to conclude.

### 6.4 Campaign layer
Per-vehicle pilot stories with intro and outro cinematics, Twisted Metal style. Explicitly a full
product layer rather than a test harness. Nothing exists.

### 6.5 Multiplayer
Explicitly a future phase. The solo game is specced to stand alone.

### 6.6 GDD diagrams
Section 15 lists two planned diagrams -- Control Flow and Interaction Map -- to be exported to
`/Docs/`. Neither exists, and neither does the folder.

---

## Closed. Not open work, and not repeated here

Two categories deliberately live in `CLAUDE.md` > **Resolved Work & Standing Decisions** rather than
in this file, because they are knowledge about the system rather than work to do, and because
`CLAUDE.md` is the file loaded into every session:

- **Consciously accepted limitations** -- the ceiling duck's inability to anticipate an overhang
  ahead, landing on another vehicle counting as floor, the four rim colliders staying, the chassis
  bank / yaw coupling not being decoupled, `WD_Shotgun`'s stale `missileFireMode`. Each was
  investigated and closed. Reopening any of them needs a new reason, not a rediscovery.
- **Theories disproved by direct measurement** -- eight of them, covering the drift flip, the
  projectile detonation bug and "bullets eaten". Read that list before investigating any of those
  three symptoms; it exists specifically to stop a session being spent re-deriving a dead end.

Do not copy either list here. If something on them reopens, it moves to this file and out of that
one.
