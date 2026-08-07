# TODO

Consolidated open work. Created 2026-08-07 from a full documentation-vs-code audit; the code was
treated as the truth and the docs were corrected to match it in the same pass.

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

### 0.1 Play-mode verification of six bug fixes
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
| Boost tick ordering | Not verified at runtime | That the 10ms shift is invisible in feel |

**Done looks like:** one play session per row, or a scripted `AIHoverInput` rig (see
`CLAUDE.md` > Recipes) driving each case. HardLock needs a second target spawned to test at all.

### 0.2 Nothing is committed
Everything from the audit session is uncommitted on `master`: 4 docs, 11 scripts, 1 new script,
7 data assets, 3 prefabs, 2 ProjectSettings files, 1 scene. **Commit before the tuning pass**, so
tuning changes are separable from structural ones in history.

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

### 2.2 EMP launch has no acknowledgement
`HoverController_EMP.OnEmpFired` is raised once and has no subscribers. No audio, no HUD cue, no
cooldown indicator. The projectile carries its own particle visual so the shot is visible, but EMP
costs 70 of 100 energy -- the most expensive ability in the game, and one that empties the meter you
need in order to disengage. A commitment that large should confirm itself.

### 2.3 Drift feel is untuned and testable for the first time
`driftLateralDamp` is 0, so there is no lateral resistance during a drift at all. Scripted full-turn
drift reached 60-76 m/s laterally and spun out, bleeding forward speed to roughly zero.

The drift *flip* is fixed and verified. How drift should FEEL is an open owner decision, not a bug.
This is the item most likely to change the character of the game.

Related invariant to maintain by hand: `minDriftSpeed` is meant to MATCH `strafeTopSpeed`, so that
outpacing strafe is exactly what earns the drift. They currently do match, at 40 each. Nothing in
code enforces it.

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

### 3.4 `AIHoverInput.OnDrawGizmos` re-casts the whole ray fan
Five more `Physics.Raycast` calls every gizmo pass, and unlike every other script in the project it
has no `HoverDebugSettings` gate. Editor-only cost, but it is the only unguarded debug draw left.

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
  assigned, which that mode ignores
- Slot 5 (Chain Gun, `ParticleSystem` mode) has a muzzle point assigned, which that mode ignores
- `blast.damageLayers` is `Everything` on `WD_ChainGun`, `WD_MachineGuns` and `WD_Shotgun`, none of
  which read the blast section

(`WD_Shotgun`'s stale `missileFireMode: 2` looks like it belongs here but does not -- it is a closed
decision, recorded in `CLAUDE.md`.)

---

## Tier 4 — Deletion candidates

Dead surface area. Each is harmless; the cost is that they make the API look wired when it is not.

- **`OnMissileLocked`** -- raised once, no subscribers, and **genuinely duplicative**.
  `VehicleHUD` already subscribes to `OnMissileLockStateChanged`, which fires on the same
  `Scanning -> Locked` transition and sets the "LOCKED" text and green fill. The only argument for
  keeping it is that a dedicated lock-tone event reads better than filtering a state switch.
- **`Propulsion.StrafePitchLimit`** -- read-only accessor, no callers. Its XML doc used to claim
  `HoverController_Aim` read it; corrected 2026-08-07 (Aim derives everything from actual euler
  angles). Delete, or use it for a HUD pitch indicator.
- **`OnRegenStarted` subscription in `VehicleHUD`** -- the event is raised and IS subscribed, but
  `HandleRegenStarted` is an empty method. v1.2 removed the local flag in favour of reading
  `IsRegenerating` directly and left the subscription behind. Either delete both sides or make it
  do something.

---

## Design decisions with no engineering blocker

These need an owner call, not code. Listed with what is already measured so the decision is cheap.

### 5.1 Hard Lock is not designed through
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
