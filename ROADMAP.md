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

## ~~PRE-ALPHA 1~~ — **COMPLETE 2026-08-20**

> Movement physics and machine gun. Greybox art. Test environment. No enemies. Bare minimum VFX.

**Owner's declaration, 2026-08-20: the milestone is complete and a build was cut the same day.** Four
of the five Tier 0 items were built and judged good in play — `0.27` in the morning, then `0.33`,
`0.32` and `0.36` together in one drive — and **`0.12` was deferred as trivial rather than fixed**.
The two-barrel-roll acceptance test was re-driven after the speed pass and holds.

**The completion criterion was met on its own terms, with one owner-granted exception.** The bar was
"the five Tier 0 items built AND judged good in play". `0.12` was never built; the owner weighed it
against shipping and declined it. **That is the criterion being exercised, not bypassed** — it was
always the owner's call to make, and the item stays on the books in `TODO.md` rather than being
marked done.

**What that took, and it is worth reading before setting the next milestone's bar.** The criterion
was set on 2026-08-20 replacing a percentage estimate ("movement at ~88%, wanting ~95%") that the
owner judged would never resolve into a decision. **It resolved the same day.** Five items, four
drives, one deferral. The percentage had been sitting unresolved since 2026-08-14.

**The milestone build, 2026-08-20.** `StandaloneWindows64`, release (not development), one scene,
**0 errors and 4 warnings** — the same four pre-existing benign ones as the first build on
2026-08-14, unchanged: the RuntimePipelineManager notice, one obsolete `FindObjectOfType`, and two
assigned-but-never-used debug fields (`_dbgBleed`, `firedFlashWasFlip`). 118 MB, 23.3s, output to
`Build/` which `.gitignore` covers. **Player version is still `0.1.0`** — not bumped, because nobody
asked and the version string has never been used to identify a build here. If builds are ever handed
to testers who report back by version, that is the moment to start.

---

## The record of how PRE-ALPHA 1 closed

The test environment, greybox art and the machine gun are all in place; the first build runs at
6.07ms with a 0.39ms spread.

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

## PRE-ALPHA 2 — weapons, and how they feel to use · **CURRENT**

> Movement and weapon physics. Grey-box art. Test environment. Enemy punching bag. Placeholder VFX.

**The completion criterion, set with the owner 2026-08-21: the work below built AND judged good in
play by the owner.** Same shape as the one that closed PRE-ALPHA 1 in a day, and set before the work
starts rather than six days into it.

**The owner's framing, and it is narrower than this milestone's old table was.** *"Damage and health
are not really a factor for this milestone, other than validating that damage is being registered.
We are going to build out the entire weapon set... make sure weapons are working -- firing and
colliding properly, hits are registering and dealing physics forces, knocking around the punching bag
in a readable way... VFX are communicating what the weapon is and does. We will nail down rate of
fire and forces but not damage."*

**Three things fall out of that and they are the reason this milestone is shaped the way it is.**
Damage is validation only, so `1.3` is a cheap item rather than a tuning pass. Health, death and
respawn are out entirely, so `1.1`, `1.2` and `1.4` all move to ALPHA. And the weapon set grew: `6.1`
was a BETA item and is now the bulk of the work here.

### The roster is settled and lives in the GDD

Twelve weapons, specified by the owner 2026-08-21, replacing the thirteen-weapon table the GDD
carried. **The Special is dropped as a slot but kept as a concept** — every vehicle gets a unique one
and what they are is TBD, so nothing needs reserving. Two are conceptual and explicitly undecided:
**Sniper** and **Flamethrower**, each with a gate below that answers it in play rather than by
argument. Full spec in `GameDesignDocument.md` > Weapons.

**Six of the twelve are built.** The other six collapse into five build jobs, and two of those jobs
pay for themselves twice — see `TODO.md` 6.7 through 6.12.

### Two design rules the owner set here, both of which reversed a documented position

**Projectiles are faster than vehicles.** Weapons should generally catch a straight runner, and
clever movement — dodges, jumps — is what evades them. **Turn rate becomes the evasion dial**, where
the GDD previously said speed was the value to change and turn rate was not. This resolves `5.5`,
which had been written as an open decision. The number to clear is the boosted ceiling.

**Weapons fire along the vehicle's nose.** Chase in drive mode where the nose points where you are
going; fight in strafe where you trade a third of your speed for visibility and precision. **That
disincentivises strafe camping structurally**, which is what `5.11` was waiting to observe, so it
drops to a watch.

### The order, and the one principle behind it

**Fix the baseline on the six weapons that exist before building six more.** Every new weapon is
authored against the force model, the speed rule and the fire rates. Settle those first and you
author twelve weapons once; leave them and you retune twelve instead of six.

| Phase | Work | Judged on |
|---|---|---|
| **1** · baseline | `1.3` damage off zero, validation only · `3.6` + `2.7` fire rates, together · `3.7` empty-slot skip · `5.5` projectile speed above the boosted ceiling · **`5.16` re-baseline force against `topSpeed` 105** · `5.2`, `5.3`, `5.4` authored across the six built weapons | — |
| **2** · impact VFX for the six | `2.5` missile detonations · `2.20` per-weapon identity, including the Chain Gun's missing prefab | **Judged with phase 1 in one drive:** does the explosion and the push read as one event, and can a good driver dodge what now catches them? |
| **3** · cheap builds, both gates | `6.9` mine · **`6.12` Flamethrower → gate: is sustained push fun?** · `6.8` multi-target lock → Hard Lock volley and Chain Lightning | The gates go early so their answers arrive while there is still time to act on them |
| **4** · the rest | `6.7` arced launch → Lob Bomb · `6.10` ricochet → Bouncing Disc · Gravity Bomb · **`6.11` sustained beam → Laser Cannon, only if `6.12` passed** | — |
| **5** · VFX for the new weapons | `2.20`, wired to both vehicle prefabs | Name the weapon from its effect alone |
| **6** · camera | **Gate: which weapons move the camera, firing one and being hit by one.** Then build it. `2.7`'s deferred recoil-character half and `2.9` are the same decision | The gun has weight without costing you the fight |
| **7** · aim | `5.12` framing · `5.13` feel · **`0.12`** folds in, closing the last PRE-ALPHA 1 item | Target size and engagement range are real quantities now |

**Two sequencing calls worth seeing rather than discovering.** The **Gravity Bomb sits mid-phase-4
deliberately, not last**: it is the most on-pillar weapon in the roster and the biggest single build,
and last place is where things get cut. And **aim lands after every weapon has been judged**, so a
material framing change at phase 7 shifts some earlier judgments under it. The docs sequenced it that
way on purpose — aim cannot be judged without targets — but it is a trade, not a free ordering.

### Depends on the owner

**Greybox props around the stationary AI.** The scene currently holds **two Rigidbodies, both craft**,
so a blast radius has exactly one thing that can respond to it and force readability cannot be judged.
The owner is setting these up. `TODO.md` 4.6.

**Placeholder VFX for all weapons**, at least enough for readability. Note the dependency runs
blast-radius-first: the weapon states its radius and the VFX is authored to match, not the reverse.

### Not gating, but first possible here

**`4.4`'s remaining half — allocation under sustained weapon fire** — becomes measurable for the first
time, because it needs weapons that fire in anger. `3.5` and `3.2` are the two that will contaminate
the figure and should land before it is taken. **The constraint on how to measure it stands: a focused
editor or a build, never remote-driving the editor.**

### Deferred out of this milestone, on the record

`1.4` limited ammo and `3.1` its refill bug → ALPHA, with the pickups (`1.5`) that make ammo mean
something. Ammo that can never be refilled degrades a weapons-feel session rather than informing it.
**One consequence to accept: the Chain Gun's design cost is ammo, so this milestone can only judge its
wind-up.** · `1.1`, `1.2` death and respawn → ALPHA, by owner decision: this milestone is weapons, not
the loop · `2.11`, `2.12`, `2.13`, `2.14` → all four are movement presentation, the largest block on
this milestone's old table and the least connected to weapons · `2.4` hard-landing dust → movement
too, and cheap to pick up if the sourced VFX happens to cover it.

---

## ALPHA — a loop you can lose

> Core loop and AI Enemy. Placeholder art. Placeholder arena. Placeholder VFX.

The step is **consequence**: death, respawn, pickups, and an AI that can actually threaten.

| | |
|---|---|
| **1.1**, **1.2** | respawn is a stub and post-respawn invulnerability does not exist. Both blocked by `1.3` |
| **1.4**, **3.1** | every weapon has unlimited ammo, and `RefillAmmo` secretly resets cooldown and wind-up. **Both deferred out of PRE-ALPHA 2 on 2026-08-21** and sequenced here because ammo only means something once `1.5` can refill it |
| **1.5** | `PickupManager` is unwritten. The only planned-and-entirely-absent module |
| **1.6** | the AI cannot fire anything except slot 0 |
| **3.3** | the AI steers away from its own target |
| **6.3** | no match flow |
| **5.1** | Hard Lock is not designed through |
| **2.16**, **2.17** | the altitude cue and a more active camera, **both deferred out of PRE-ALPHA 1 on 2026-08-20 behind exactly the arena this milestone starts.** Sequenced here because neither can be judged in the RVP demo city: an altitude cue compensates for a tiled ground and no props, and an automatic camera earns its keep by anticipating terrain. **Judge whether they are still needed once `6.2` has landed, rather than assuming they are** |
| **2.18**, **2.19** | binding ergonomics, and teaching air control and tricks. Deferred out of PRE-ALPHA 1 on 2026-08-20 until the control scheme is complete. **2.19 is FTUE work and this is the first milestone with a loop to teach inside of.** 2.18 is no longer gated on the weapon roster: `6.1` finishes in PRE-ALPHA 2, so "all the controls in" is true by the time this milestone starts |

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
| **6.1** | **the roster is built in PRE-ALPHA 2 and what lands here is what those gates deferred**: the Sniper and Flamethrower if either was held, the Laser Cannon if `6.12` failed, and the per-vehicle Specials, which need `5.7` before "one per vehicle" means anything |
| **5.7** | vehicle roster is one profile |
| **5.8** | vehicle scale, deferred behind two gates |
| — | a performance pass against five AI. The instruments exist; the measurement does not |

---

## What this roadmap does not cover

Final art, campaign (`6.4`), multiplayer (`6.5`) and the GDD diagrams (`6.6`) all sit beyond BETA
and are not sequenced here.
