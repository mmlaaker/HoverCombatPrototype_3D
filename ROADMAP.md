# Hover Combat Prototype: Roadmap

Owner's milestone definitions, recorded 2026-08-14. **This file owns the definition of "done" at
each stage and the order the work happens in. It owns no facts of its own:** every item below is a
pointer into `TODO.md` by number, so nothing here can go stale independently of the item it names.

The distinction against the other documents: `GameDesignDocument.md` says what the game *is*,
`TODO.md` says what is *unfinished*, and this file says **which unfinished things stand between here
and the next stage.** `TODO.md` has no stage gating and is ordered by tier, not by milestone; that
is what this exists to supply.

**Art vocabulary, owner's definition:** *greybox* is untextured blocking, *placeholder* is
stand-in art that reads correctly, *non-final* is more refined than placeholder but explicitly not
ready to ship. Nothing on this roadmap reaches final art.

---

## PRE-ALPHA 1 — movement, one gun, nothing to shoot at

> Movement physics and machine gun. Greybox art. Test environment. No enemies. Bare minimum VFX.

**Status: playtested 2026-08-19, scope re-set 2026-08-20, and FOUR OF THE FIVE closed the same day.**
The test environment, greybox art and the machine gun are all in place; the first build runs at
6.07ms with a 0.39ms spread. **`0.27`, `0.33`, `0.32` and `0.36` are all done and judged good.
`0.12` is the only item left.**

**The build-three-then-judge-once sequencing worked, and that is worth recording as a method rather
than as a one-off.** `0.33`, `0.32` and `0.36` all landed on 2026-08-20 and were judged together in
**one drive** later the same day, because they move the same subsystem and judging them in turn would
have meant re-tuning each as the next arrived. Owner: *"genuinely better than the playtest build ...
still really good control but less arcade-y."* Outcome in `TuningLog.md` > The speed pass; acceptance
criteria in `CLAUDE.md` > Judged good 2026-08-20.

**The pass closed on a knob none of the three items had on the table.** All three were written as
tuning-the-approach problems and `0.33` explicitly ruled `topSpeed` out of scope, on the grounds that
no tester had asked for more speed. **The complaint was the ceiling.** `topSpeed` 80 to 105, with
`maxForwardAccel` untouched, delivered the longer ramp the owner asked for AND a quicker approach to
every speed below it. Six companion values moved with it to hold their ratios. **The general lesson
is filed in `CLAUDE.md`: absence of a request is not a bound.**

**So PRE-ALPHA 1 now turns on a single decision, and no remaining build.** `0.12` is an accepted
residual: it violates a hard requirement on paper and the owner has already said "no big deal right
now". **What stands between here and the milestone is the owner either accepting it explicitly or
asking for the fix** — not engineering time.

**One acceptance criterion is outstanding and is not a blocker: the two-barrel-roll landing test has
not been re-driven since the speed pass** (see below, and `TODO.md` Tier 0 for why it is very likely
unaffected). Settle it on the next drive rather than scheduling a run for it.

**The milestone now has a completion criterion rather than a percentage.** The owner's 2026-08-14
estimate was movement at ~88% wanting ~95%, which was never going to resolve into a decision.
**PRE-ALPHA 1 is complete when the five Tier 0 items below are built AND judged good in play by the
owner.** Owner's decision, 2026-08-20. **Four of the five are struck, all on the day the criterion
was set. The fifth is the one that was never going to be built.**

**The scope was expanded deliberately, and bounded just as deliberately.** The four-tester playtest on
2026-08-19 produced three new movement items. They are in because **they are directly about how the
craft moves**, which is this milestone's subject. Everything else the playtest produced was deferred
rather than absorbed: the camera and altitude items behind a representative arena, the control-scheme
items behind a complete control set, and two design questions filed with no action. **This is a scope
expansion of exactly three items and it is not an invitation to a fourth.**

**`TODO.md` Tier 0 IS this milestone's work list, and the two are now the same five numbers.** In the
owner's working order:

| | |
|---|---|
| ~~**0.27**~~ | **DONE. Shipped and judged good in play 2026-08-20**, the owner driving both repro cases and unable to get stuck in either. The soft lock is closed and this milestone no longer has an item that can end a session. Outcome in `TuningLog.md` > Closed as 0.27. **The dead-man timer was the right shape and the obvious implementation of it was not:** forcing the righting on and suppressing the hover push are each necessary and neither is sufficient, which only showed up by building half of it and watching a craft sit at 177 degrees for twelve more seconds |
| ~~**0.33**~~ | ~~Acceleration is flat and top speed arrives in about 0.92s.~~ **DONE, judged good 2026-08-20.** `accelCurve` shipped first and took the ramp to 2.35s; the item did not close until the closing drive found the real lever was `topSpeed` 80 → **105**, which gave a 3.02s ramp AND a faster approach to every speed under it |
| ~~**0.32**~~ | ~~Steering authority does not fade with speed.~~ **DONE, judged good 2026-08-20.** `yawSpeedFade` and `boostYawMultiplier` shipped and the owner re-eased the curve in the closing drive to hold authority further up the range. `yawAccel` 15 and `yawDamping` 8 never moved |
| ~~**0.36**~~ | ~~Left stick X does nothing in drive mode.~~ **DONE, judged good 2026-08-20.** `driveLateralPush` shipped at 8 and the owner raised it to **10** in the closing drive. Its one open question — whether the lean arrives quickly enough — was answered by distance, so **no response term was needed and `lateralDamp` was never touched** |
| **0.12** | A sliver of bumper still clips on the pitch-up. **Last, and still an accepted residual rather than a blocker.** Confirmed present 2026-08-17 after four separate drive-camera changes |

**What the playtest confirmed, and it is the larger half of the result.** Four testers who had never
played judged the hover read, drive mode, the twin-stick scheme and the learning curve all good, and
all four improved sharply in a second run they asked for. Recorded as acceptance criteria in
`CLAUDE.md` > Judged good in the pre-alpha 1 playtest. **A change that degrades one of those is a
regression even if it closes an item above.**

**Deferred out of this milestone by the owner on 2026-08-20**, all renumbered out of Tier 0 by the
sorting rule: **2.16** (altitude cue, was 0.28) and **2.17** (a more active camera) behind a
representative arena; **2.18** (binding ergonomics) and **2.19** (teaching air control and tricks)
behind a complete control set; **5.13** (aim feel, was 0.31) behind weapons. **5.14** and **5.15** are
filed as feedback with no action. See `TODO.md` > Retired numbers for what each number became.

**A performance and optimization sweep of the movement and camera code ran 2026-08-20, after all three
items landed and before the feel pass, and it does NOT change the milestone.** The owner's reason for
its timing was that three changes had just touched both load-bearing pillars. **Nothing it found was
behaviour-affecting**: the hot paths were already allocation-free and correctly cached, the one real
risk (the camera's execution order against the Cinemachine brain being unpinned) measured as already
correct and was pinned preventively, and the rest was math and memory. **The feel pass ran later the
same day and closed all three items** — and the sweep's claim that it changed nothing behavioural
held up under the one test that matters, the owner driving it and reporting an improvement they
attributed entirely to the tuning. Outcome
in `TuningLog.md` > The movement and camera performance sweep; three findings that need owner decisions
rather than code are parked in `TODO.md` > 4.4.

**Also worth doing during the next long run, though it gates nothing: 4.4**, now narrowed to
allocation under sustained weapon fire alone. **Its marker half closed 2026-08-20** — the marker key
and the pad are both confirmed working in a build, which also retired the standing worry that
`PlaytestSession` had never been driven by a physical button. **The sweep added a constraint on how it
can be measured:** allocation figures gathered while remote-driving the editor are worthless, so this
needs a focused editor or a build.

Closed on 2026-08-15: the trick energy economy, built and judged good in one session. Everything else
this milestone carried was closed across 2026-08-17 and 2026-08-19; the full list and where each
outcome went is in `TODO.md` > Retired numbers.

**`0.21` carries its own regression test**, which is unusual and is recorded in `CLAUDE.md` > Judged
good: the owner accepted the value on the condition that two barrel rolls still land. That makes the
trick margin, not the hard landing, the thing to re-check after anything that touches fall gravity,
roll rate or jump impulse. **`0.33` and `0.32` were flagged as qualifying and the test was NOT
re-driven after they closed.** It is recorded as outstanding rather than passed. The argument that it
is unaffected is in `TODO.md` Tier 0 — the speed pass moved horizontal caps only and left
`extraFallGravity`, `airRollTorque` and all three jump impulses alone — but **an argument is not the
test the owner asked for.** Roll twice on the next drive and it is settled.

Note the AI already exceeds this milestone's bar: it drives, roams, flees and shoots, where
PRE-ALPHA 1 asks for no enemies at all.

---

## PRE-ALPHA 2 — weapons become real, and something takes the hits

> Movement and weapon physics. Grey-box art. Test environment. Enemy punching bag. Placeholder VFX.

The step is **weapon physics** and **a target worth hitting**. The AI punching bag largely exists
(`1.6` records it as deliberately passive), so most of the work is on the weapons themselves.

| | |
|---|---|
| **1.3** | four of six working weapons deal zero damage. **The root item** -- nothing can die until it moves |
| **1.4** | every weapon has unlimited ammo |
| **2.7** | automatic fire rates are far too low |
| **2.2**, **2.15** | EMP launch, and a denied jump, have no acknowledgement. Both wired, both unjudged, and **neither judgeable in this build**: all three impulse channels are deliberately off and come back together. `2.15` succeeds `0.16` |
| **2.4**, **2.5** | hard landings have no dust, missile detonations have no explosion. This milestone's "placeholder VFX" |
| **2.11** | boost presentation FX: vignette, speed lines, duration-based rumble. Deferred here 2026-08-14, and **confirmed as the remaining boost gap** by the owner 2026-08-16 |
| **2.12** | drift has no VFX. Now the whole of what remains from the drift report: `0.23` closed as a suppression bug and no cue was built |
| **2.13** | charge jump wind-up VFX, energy gathering under the craft. **The movement half shipped as `0.25`**, and hands this a hard constraint: the squat must not add impulse, so the discharge has to read as release without pushing |
| **2.14** | thruster VFX and the boost camera fall out of step on release, measured at 2.2x. Deferred here 2026-08-17 because the thrusters it targets are placeholder; succeeds `0.24` |
| **5.11** | whether players camp in strafe. Unanswerable until `1.3` gives combat something to pressure with |
| **5.12**, **5.13** | revisit the aim framing, and re-judge the aim feel, once there is something to aim at. `5.12` succeeds `0.14`, same reasoning as `5.11`. **`5.13` was `0.31` and was deferred out of PRE-ALPHA 1 on 2026-08-20**: three tuning values whose meaning changed when `0.29` closed and which have never been judged for what they now do. It wants the same session as `5.12` |
| **3.1**, **3.6**, **3.7** | weapon-side traps worth clearing while the code is open |
| **5.2**, **5.3**, **5.4** | the knock-around pass, only partly applied |

---

## ALPHA — a loop you can lose

> Core loop and AI Enemy. Placeholder art. Placeholder arena. Placeholder VFX.

The step is **consequence**: death, respawn, pickups, and an AI that can actually threaten.

| | |
|---|---|
| **1.1**, **1.2** | respawn is a stub and post-respawn invulnerability does not exist. Both blocked by `1.3` |
| **1.5** | `PickupManager` is unwritten. The only planned-and-entirely-absent module |
| **1.6** | the AI cannot fire anything except slot 0 |
| **3.3** | the AI steers away from its own target |
| **6.3** | no match flow |
| **5.1** | Hard Lock is not designed through |
| **2.16**, **2.17** | the altitude cue and a more active camera, **both deferred out of PRE-ALPHA 1 on 2026-08-20 behind exactly the arena this milestone starts.** Sequenced here because neither can be judged in the RVP demo city: an altitude cue compensates for a tiled ground and no props, and an automatic camera earns its keep by anticipating terrain. **Judge whether they are still needed once `6.2` has landed, rather than assuming they are** |
| **2.18**, **2.19** | binding ergonomics, and teaching air control and tricks. Deferred out of PRE-ALPHA 1 on 2026-08-20 until the control scheme is complete. **2.19 is FTUE work and this is the first milestone with a loop to teach inside of.** 2.18 may slip to BETA: "all the controls in" is not strictly true until `6.1` finishes the weapon roster |

Placeholder arena work begins here (`6.2`), replacing the RVP demo city. **It now gates two deferred
movement items as well as its own quality bar**, which raises its value above what this table showed
before 2026-08-20.

---

## BETA — five opponents, and art that means it

> Core loop vs 5 AI enemies. Non-final art. Non-final arena. Non-final VFX.

The step is **scale and presentation**. Five simultaneous AI is a performance question as much as a
design one, and nothing has ever been measured with more than two craft in the scene.

| | |
|---|---|
| **6.2** | the arena, at non-final quality |
| **6.1** | seven of thirteen weapons still unimplemented |
| **5.7** | vehicle roster is one profile |
| **5.8** | vehicle scale, deferred behind two gates |
| — | a performance pass against five AI. The instruments exist; the measurement does not |

---

## What this roadmap does not cover

Final art, campaign (`6.4`), multiplayer (`6.5`) and the GDD diagrams (`6.6`) all sit beyond BETA
and are not sequenced here.
