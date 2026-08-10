# TODO

Consolidated open work. Created 2026-08-07 from a full documentation-vs-code audit; the code was
treated as the truth and the docs were corrected to match it in the same pass.

**Updated 2026-08-08** from a movement tuning session driven by a full playtest. Reverse/strafe
symmetry, the tap-jump grounded band, air control entry, drift feel and the weapon-switch firing bug
were all closed and their outcomes moved to `CLAUDE.md`. Everything else the playtest raised is
filed below: 2.6 to 2.10 (feel), 0.6 (what that session did NOT verify), 5.9 to 5.11 (decisions
left open). The session's performance investigation is closed and lives in `CLAUDE.md`; the short
version is that the chop was background applications, not the game.

**Updated 2026-08-09** from a camera session. Two of 2.8's three complaints are closed, 2.6 is
partly closed, and 2.1, 2.2 and 2.9 now name the mechanism that will close them. The camera work
itself, its measurements and four new measurement traps are in `CLAUDE.md`. Two phases of that plan
remain: the impulse router (2.1, 2.2, 2.9) and the reticle (2.8). Note a reported "camera jitter"
turned out to be no antialiasing rather than anything in the camera, and the resolved entry for it
is worth reading before investigating any similar symptom.

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

### 0.6 Consequences of the 2026-08-08 tuning session
Two of the six original entries were verified in a play-mode pass the same day and are recorded
here with their numbers. **What remains is four items, and every one of them is a feel judgement or
needs a human, so no amount of scripting will close them.** That is the honest boundary of what
measurement can do for this list.

Verified 2026-08-08 in a play-mode pass:

| Claim | Result |
|---|---|
| Six vehicles at once, the real performance question | **PASS, comfortably.** 1 vehicle 3.48ms mean / 4.08 p95; six vehicles **4.33ms mean / 5.13 p95 / 6.98 max**. That is +24% for 6x the vehicles, so roughly **0.17ms of marginal cost each** against a mostly fixed frame. Twelve would still sit near 5ms. **Caveat: AI driving only, no combat, because `AIHoverInput` never sets `FirePressed` (see 1.6). Particle collision at scale is still unmeasured** |
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

### 2.1 A denied jump is completely silent
`OnJumpDenied(bool)` is raised in both `FireGroundedJump` and `FireAirJump` (the bool distinguishes
grounded from air) and has **no subscribers**. The player presses jump, nothing happens, and there
is no way to tell "out of energy" from "input didn't register" or "still in the 0.2s post-land
lockout".

`jumpGroundedEnergyCost` is 25 against a 100 pool, so denial arrives on the fourth consecutive jump.
This is a common state, not an edge case.

This matters more than a normal missing cue because **"Energy as Tempo" is one of the six design
pillars.** A resource the player is meant to manage as their primary tempo tool currently gives no
feedback when it runs out. Cheapest real improvement to feel on this list.

**Planned mechanism:** the impulse router, phase 2 of the camera plan. `HoverLandingCameraImpulse`
generalises into the owner of every camera punch, with explicit per-channel
`CinemachineImpulseSource` references. A denied jump gets the light channel: small and slightly
unpleasant, because it is a failure signal. Audio is deliberately out of scope, which is exactly why
this lands on the camera.

### 2.2 EMP launch has no acknowledgement
`HoverController_EMP.OnEmpFired` is raised once and has no subscribers. No audio, no HUD cue, no
cooldown indicator. The projectile carries its own particle visual so the shot is visible, but EMP
costs 70 of 100 energy -- the most expensive ability in the game, and one that empties the meter you
need in order to disengage. A commitment that large should confirm itself.

**Planned mechanism:** its own impulse channel on the router (see 2.1). Both this and the denied
jump are wiring rather than new events -- `OnEmpFired` and `OnJumpDenied` are already raised and
have no subscribers at all.

### 2.3 ~~Drift feel is untuned~~ — DONE 2026-08-08, with two open questions
Tuned in the 2026-08-08 session and confirmed good by the owner. Drift is now an **aiming tool**:
the gap between heading and velocity is the product, bought with acceleration. Causes, fixes and
the two-knob tuning model are in `CLAUDE.md` > Propulsion v1.8. `driftLateralDamp` is 0.25,
`driftForwardDamp` 0.3, plus `maxDriftAngle` and `driftHopImpulse`.

Still open, both feel questions rather than bugs:
- **Is the angle ceiling settling or hitting a wall?** At low `driftLateralDamp` the slide runs
  right up to `maxDriftAngle` and stops. Raising damping pulls the balance point below the cap so
  it eases in instead: at a 45 cap, damping 0.25 settles at 42 and damping 1.0 settles at ~34.
  Same cap, completely different character.
- **No exit payout exists and that is deliberate.** The owner rejected a boost reward: drift is
  about angle, not speed. Revisit only if the angle stops feeling worth the acceleration.

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

### 2.6 Boost reads flat — PARTLY ADDRESSED 2026-08-09
Owner note, 2026-08-08 playtest: "the boost feels somewhat mid". The multiplier was never the
problem: `boostSpeedMultiplier` and `boostAccelMultiplier` are both 1.5, a real 50% increase, so
this was always presentation.

Done, and not repeated here (see `CLAUDE.md`):
- **`boostBlendSeconds` cut to 0.125** by the owner. A boost that took a third of a second to
  arrive read as gradual rather than as a kick.
- **FOV kick added**, scaled by `BoostLerp` so it inherits the blend and can never disagree with the
  thrust about when boost started. Written to both vcams against their own bases, since mode
  switching is by priority and the brain blends them. Later gated on forward motion, because it was
  also firing while reverse-boosting.

**Still open: the rest of the boost language.** FOV alone is a sustained cue, and sustained cues are
adapted to within about a second, so transients and periphery are what actually sell speed. Planned:
pull-back on Z, an FOV overshoot that settles rather than a step, camera lag on engage, and a return
slower than the entry. This is phase 3 of the camera plan and it is deliberately easier to judge now
that definitive states can be frozen and inspected rather than caught in a 0.125s window.

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

### 2.9 Impacts are under-intense, and big drops do not cost enough control
Owner: crashes into walls, ground and other vehicles should hit harder; a mountain drop "should lose
more control and tumble more than it does". The hard-landing camera punch is liked and should stay.

The hard landing system today is **feel-only by design**: it suppresses LIFT on a 0.35s taper and
fires the camera impulse, with no damage and no lockout. Leveling torque stays at full 12 throughout,
which is why a badly angled high-speed landing snaps flat instead of tipping you.

**Cheapest real change:** scale `levelingTorqueStrength` by the same `hardLandingSeverity` and timer
that already scale `liftFactor`. Reuses existing machinery rather than adding a system, and it makes
a bad landing angle actually cost something.

For wall and vehicle crashes there is no shake at all. Planned as part of the impulse router (see
2.1), with two details established while planning it. **One impulse source cannot serve every
channel:** shape and duration are authored on the source, so a heavy landing thud and a light denial
tick need separate sources, and the existing `[RequireComponent]` plus `GetComponent` pattern picks
whichever it finds first. **And collisions must filter out floor-like contacts** using the same
surface-angle idea Foundation already applies for `unstickMaxSurfaceAngle`, or every landing
double-fires against the existing `OnHardLanding` path and the punch the owner likes regresses.
There is currently no `OnCollisionEnter` on the vehicle at all; Foundation only has
`OnCollisionStay` for contact tracking.

**Measured context that changes the framing:** no jump can trigger a hard landing at all
(full charge lands at 43.4 against `hardLandingMinSpeed` 58); the system fires only on mountain
drops, confirmed live at 58.0 to 87.6 m/s. If landings should matter more generally, that threshold
is the knob, and it needs the measured pass the owner already asked for.

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

### 5.9 The air jump has never been justified or cut
Owner, 2026-08-08: "I'm not sure if the double jump is fully justified, I might consider removing
it." No decision was reached.

The argument for keeping it is that it does not function as a second jump. It is a **hang-time
extender** that buys the window to finish a trick, and tricks are the part of the build the owner
rates highest. The argument against is the energy budget (2.10): 25 per air jump against a 100 pool
that already feels short.

Decide it against 2.10 rather than on its own. If tricks start paying energy back, the air jump
stops competing with boost and the case for cutting it weakens considerably.

### 5.10 Wall jump / wall riding, speculative
Owner idea: count as grounded when deliberately oriented against a wall with sensors in range.

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
