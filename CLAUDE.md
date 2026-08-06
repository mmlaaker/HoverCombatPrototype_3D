# Hover Combat Prototype: Claude Context

## Behavioral Rules
- Read existing files before writing. Do not re-read unless the file has changed.
- Thorough in reasoning, concise in output.
- Skip files over 100KB unless the task explicitly requires them.
- No sycophantic openers or closing fluff.
- No em-dashes.
- Do not guess APIs, versions, flags, commit SHAs, or package names. Verify by reading code or docs before asserting.

---

## Project Summary
Ground-level aerial dogfighting game. Hover vehicles behave like low-altitude jets dogfighting above the ground. Combat and traversal are the same skill expression: momentum management determines fight outcomes, not aim or loadout. Inspired by Twisted Metal, Rocket League, Unreal Tournament, Wipeout, F-Zero GX, and Jak 2. Unity 6.3 URP. Solo campaign against AI opponents; multiplayer is a future phase.

**Design motto:** "The fun comes from control."

---

## Engine & Stack
- Unity 6.3 URP (no HDRP)
- Rigidbody-based hover physics
- Unity Input System (Vector2 action; axes split in code)
- Cinemachine 3.1.6 third-person camera
- GitHub + Fork + VSCode

---

## Module Status

| Module | Version | Notes |
|---|---|---|
| `HoverController_Foundation.cs` | v1.3 | **Do not modify without explicit justification.** Spring-damper hover lift (`ForceMode.Acceleration` = mass independent; sensors ignore triggers), leveling torque as the **single attitude authority** (ground normal + aim pitch target; aim strength applies to pitch axis only; `aimPitchDamping` adds aim-only settle damping independent of ride stiffness), two-path recovery (upright-stuck vs flipped; unstick arms only while vertically settled so belly scrapes don't get pulse trains). Ground contact tracked by timestamp, not exit callback. v1.1: hard landing spring give -- airborne-to-grounded edge above `hardLandingMinSpeed` suppresses spring forces on a front-loaded taper so the chassis slams its collider, then pops back to ride height (feel only, no damage/lockout). v1.2: air control torque -- Propulsion pushes airborne pitch/roll intent via `SetAirControl(pitch, roll, weight)`; per-axis torque authority (roll > pitch), pitch/roll damping lerps toward `airControlDamping` by weight; still the single attitude authority. Public: `IsHoverGrounded`, `AverageGroundNormal`, `SetRecoveryEnabled(bool)`, `SetAimPitch(degrees, weight)`, `SetAirControl(pitch, roll, weight)`, `OnHardLanding(severity 0..1)` event. v1.3: **gravity feedforward** -- each hover point feeds forward `gravityMagnitude * hit.normal.y / N` along the surface normal, so the spring corrects error only instead of also holding the craft up. `hoverHeight` is now the literal resting height on flat ground and on slopes (verified 7.000m on both), while the craft still slides downhill because only the normal component is supported. This decouples ride height from spring stiffness, which were previously the same knob inverted. **Asymmetric fall gravity**: new `extraFallGravity` applies only while airborne and descending, so the rise stays generous for air tricks while the landing reads decisively (measured fall 52.18 vs designed 52.24). Slope lift compensation **removed** as redundant. Flip-recovery gate diagnostics added to `OnDrawGizmos` (inside the existing `#if UNITY_EDITOR`) |
| `HoverController_Propulsion.cs` | v1.2 | Drive/drag mutually exclusive; top speed in `ApplyDrive` with exact-cap clamp (crossing tick lands exactly on the cap, no ripple); lateral damp excludes intended strafe velocity (`strafeTopSpeed` is the real lateral ceiling); lateral drive gated at the cap with a gentle over-speed bleed, so dodge bursts at the cap are **additive** (surge, then ~1s glide back down); strafe dodge burst; continuous boost requires >= 0.15 throttle deflection. Strafe pitch applies **no torque** -- it hands the accumulated target to `Foundation.SetAimPitch`. v1.1: drift-held airborne air control (left stick Y/X = pitch/roll, right stick X stays yaw; strafe-aim precedence; no energy cost; reuses the `Drift` bool, free airborne). Public: `DriftLerp`, `StrafeModeBlend`, `StrafePitchLimit`, `AirControlWeight`, `OnJumpDenied` event. v1.2: **strafe forward dead band fixed** -- `ApplyOverSpeedBleed` was bleeding against the unblended `effectiveTopSpeed` while `ApplyDrive` clamped to the strafe-blended cap, leaving a band where drive was suppressed for being over cap, drag was suppressed for held throttle, and the bleed had not engaged, so the chassis coasted there indefinitely. Both now use the same expression. **Boost works in reverse**: the boost gate always accepted reverse throttle, so holding boost while reversing drained energy every tick for zero effect, because only the forward branch read the boost-scaled values. Reverse accel and cap are now scaled by the same ratios, and the reverse bleed matches so the two cannot oppose each other |
| `HoverController_Energy.cs` | v1.0 | `TryConsume` fail also resets regen lockout (intentional anti-spam). EMP freeze additive. Public: `Energy`, `EnergyNormalized`, `IsEmpFrozen`, `IsRegenerating` |
| `HoverController_Weapons.cs` | v1.0 | Weapons **never spend energy**. EMP only gates fire via `IsEmpFrozen`. `WeaponDefinition` SO (shared); `WeaponSlot` (per-vehicle). Four `WeaponType`s: SingleShot, Automatic, Missile, Mine. Two `ProjectileMode`s: Instantiated, ParticleSystem. Public: `ActiveSlot`, `ActiveSlotIndex`, `CurrentLockState`, `LockProgress`, `LockTarget`, `RefillAmmo()`, `SetActiveSlot()` |
| `HoverController_Aim.cs` | v1.0 | **LateUpdate only; no `DefaultExecutionOrder` needed.** Reads actual vehicle pitch from Rigidbody (no independent accumulation). Only rotates active slot's `vfxMount`; Instantiated-mode weapons unaffected. Called by Weapons via `NotifySlotChanged(WeaponSlot)` |
| `HoverController_Shield.cs` | v1.0 | Fixed-duration invulnerability burst. Flat energy cost on activation; cannot be cancelled by player. Active shield absorbs an incoming EMP (shield deactivates, no freeze). New activation still blocked while already EMP-frozen. Public: `IsActive`, `TimeRemaining`, `OnShieldActivated`, `OnShieldDeactivated`, `TryActivate()`, `TryAbsorbEmp()` |
| `VehicleHealth.cs` | Active | HP pool, `OnDamaged`/`OnDeath` events, invulnerability (post-respawn grace and shield-driven), `Respawn()` |
| `VehicleHUD.cs` | v1.2 | Event-driven; `SyncAll()` in Start; trailing `_wasRegenerating` flag |
| `HoverCameraController.cs` | v1.2 | Cinemachine 3.1.6; strafe cam uses `LockToTargetNoRoll`. v1.2: drive cam follows a runtime `CameraHeadingProxy` (position + stable yaw only) instead of the vehicle -- yaw read from the transform while upright within `maxStableTilt`, integrated from Rigidbody world-Y angular velocity while tilted, so air-control flips no longer swing the camera 180 degrees; heading tracking fades to zero entirely while `Propulsion.AirControlWeight` > 0 (stable frame during stunts, reconverges on release); LookAt stays on the vehicle |
| `HoverLandingCameraImpulse.cs` | Active | Cinemachine impulse camera punch on `Foundation.OnHardLanding`, scaled by severity. Impulse shape/duration authored on the `CinemachineImpulseSource`; listeners on both vcams (strafe gain lower -- zero-damping cam reads impulse harder) |
| `WeaponDebugDraw.cs` | v1.0 | Scene-view draws for the impact half of the weapon system (the targeting half is covered by gizmos on `HoverController_Weapons`). Three draws: **impulse split** (contact point, centre of mass, the lever arm between them in magenta, and the two impulse shares as scaled arrows) so `destabilizeFraction` is tuned against its actual mechanism; **splash attribution** (blast sphere, plus a line per victim to the point falloff was measured from, coloured green-to-red by the resulting scale) which makes the compound-collider class of bug self-evident; **particle contacts** colour-coded green/yellow/red by whether the pellet found something damageable, a Rigidbody, or plain geometry. Uses `Debug.DrawLine` with a duration, not `OnDrawGizmos`: impacts are instantaneous and need to linger, and `RocketProjectile` destroys itself at the end of `Explode`. Every method is `[Conditional("UNITY_EDITOR")]`, so calls and their arguments compile out of player builds. No text labels: `Handles.Label` only works from `OnDrawGizmos`/`OnGUI` and these run from collision code, so magnitudes are encoded as colour and length, with `RocketProjectile.logSplash` for exact numbers |
| `WeaponImpact.cs` | v1.0 | Static helper. The single implementation of "apply a knockback impulse split between the contact point and the centre of mass". Both delivery paths call it, so a particle hit and a rocket hit destabilize identically for the same `destabilizeFraction`. Also guards the degenerate cases the old inline copies did not: null rigidbody, and a blast centred exactly on the centre of mass (zero direction, which would otherwise normalize to NaN) |
| `IProjectileImpactCarrier.cs` | v1.0 | `SetImpact(directHitForce, splashForce, destabilizeFraction)`. Sibling of `IProjectileDamageCarrier`; lets a `WeaponDefinition` own knockback and push it onto the spawned prefab. Called once in `FireAllMuzzles` right after `SetDamage`, before the projectile's first tick |
| `ParticleWeaponCollision.cs` | v1.3 | Particle-based weapon hits. v1.3: gained a debug flag (it had none) plus `HoverDebugSettings` wiring, and marks every particle contact point colour-coded by what it found. v1.1: off-centre impact via `AddForceAtPosition` at `ParticleCollisionEvent.intersection`, so hits rotate the target. v1.2: the old `destabilizeOnImpact` bool became the shared `destabilizeFraction` (0..1) and the inline force code became a `WeaponImpact.Apply` call. **Keep at 0 for sustained fire** (per-bullet torque accumulates every frame); Shotgun runs at 1, where 40 pellets at 40 contact points across a 20-degree cone supply the rotation from spread geometry |
| `RocketProjectile.cs` | v1.4 | Dumbfire/homing rocket, splash with falloff. v1.4: debug draws. Now honours `HoverDebugSettings` like the hover scripts instead of a local flag only; draws the blast sphere, per-victim splash attribution, and the impulse split on every hit. Optional `logSplash` prints distance, falloff scale, damage, and force per victim, for comparing two shots precisely. v1.3: knockback is no longer authored here. `directHitImpactForce`, `splashImpactForce`, and `destabilizeFraction` stopped being `[SerializeField]` and now arrive from the `WeaponDefinition` via `IProjectileImpactCarrier`; the private `ApplyImpact` helper moved to the shared `WeaponImpact`. Flight and blast geometry stay per-prefab. v1.2: impact destabilization. `destabilizeFraction` (0..1) splits the impact impulse between the contact point and the centre of mass, so hit location determines rotation while total knockback stays fixed and the two tune independently. `AddForce` alone is centre-of-mass by definition, so lever arm and angular velocity were always exactly zero. Splash now measures falloff distance and push direction from `hitRb.worldCenterOfMass`, not the hit collider's transform: children of a compound collider all resolve to one `IDamageable`, so iteration order previously decided both and swung splash damage/force by 3x |
| `Editor/WeaponDefinitionEditor.cs` | v1.0 | Custom inspector for `WeaponDefinition`. Draws only fields the selected `WeaponType`/`ProjectileMode` actually read: wind-up is Automatic only, lock settings are Missile only (`lockAcquireTime` HardLock only, range/cone hidden for Dumbfire), `startingAmmo` hidden when `maxAmmo` is 0, and `splashImpactForce` shown for Instantiated mode only. Section headings come from the existing `[Header]` attributes, which `PropertyField` draws with their field, so a heading hides along with its group. Hiding is view-only; serialized values are untouched. Also surfaces validation: null `projectilePrefab` in Instantiated mode, `damage: 0`, `startingAmmo > maxAmmo`, `destabilizeFraction` above 0 on a sustained-fire weapon, and impact forces set on a projectile prefab that doesn't implement `IProjectileImpactCarrier` |
| `PickupManager.cs` | Planned | Not yet implemented |
| AI FSM | Planned | Phase 2; chase/evade/shoot |

## Inter-Module Wiring

- **Input:** All HoverController scripts acquire input via `GetComponent<IHoverInputProvider>()` in Awake. No inspector wiring. Swap `PlayerHoverInput` for an AI component to change who drives the vehicle. Key input props: `ThrottleInput`, `StrafeX`, `TurnInput`, `Boost`, `Drift`, `Jump`, `StrafeHeld`, `CameraLookY`, `FirePressed`, `FireHeld`, `CycleWeaponNext/Prev`, `ShieldPressed`.
- **Foundation → Propulsion:** Propulsion reads `Foundation.IsHoverGrounded` each FixedUpdate to gate throttle, drag, and drift.
- **Energy → Propulsion/Weapons/Shield:** Propulsion calls `TryConsume` per frame for boost (continuous) and once for jump/dodge (instantaneous). Shield calls `TryConsume` once on activation. Weapons only reads `IsEmpFrozen` -- no energy is spent on firing.
- **Shield → Health:** `VehicleHealth.TakeDamage` short-circuits when `_shield.IsActive`. Shield owns its own invulnerability state; the existing `_isInvulnerable` flag remains for post-respawn grace.
- **Energy → Shield:** `EmpProjectile.Consume` calls `Shield.TryAbsorbEmp` on hit; if true the freeze is skipped. Shield also subscribes to `Energy.OnEmpFreezeApplied` as a defensive fallback. New activation is blocked while `IsEmpFrozen`.
- **Weapons → projectiles (spawn-time push):** `FireAllMuzzles` hands the freshly instantiated prefab three optional interfaces in order: `IProjectileDamageCarrier.SetDamage`, `IProjectileImpactCarrier.SetImpact`, `IHomingTarget.SetTarget`. All are null-conditional, so a prefab implements only what it needs. This is why weapon tuning lives on the `WeaponDefinition` and not on projectile prefabs: the definition is the single author, the prefab is the delivery mechanism.
- **Weapons → Aim:** Weapons calls `aim.NotifySlotChanged(slot)` on slot change. Aim caches the new `vfxMount`.
- **Propulsion → Foundation (aim pitch):** Propulsion calls `foundation.SetAimPitch(accumulatedDegrees, strafeBlend)` every FixedUpdate; `(0, 0)` on strafe exit and EMP freeze. Foundation's leveling torque drives toward the target (`aimPitchTrackingStrength`, pitch axis only); Propulsion never applies pitch torque itself. One attitude authority, no competing forces.
- **Propulsion → Aim:** Aim reads `transform.eulerAngles` directly (actual vehicle orientation); no reference to Propulsion needed.
- **Propulsion → Foundation (air control):** Propulsion calls `foundation.SetAirControl(pitchInput, rollInput, weight)` every FixedUpdate; weight = air-control blend x (1 - strafe blend) so strafe-aim wins airborne; `(0,0,0)` when inactive and on EMP freeze. Foundation applies the torque and crossfades pitch/roll damping toward `airControlDamping`. One attitude authority, no competing forces.
- **Foundation → VFX/Camera (hard landing):** `Foundation.OnHardLanding(severity)` fires on the airborne-to-grounded edge above `hardLandingMinSpeed`. `HoverVehicleVFX` spawns a one-shot ground dust burst (heavy prefab at severity >= threshold); `HoverLandingCameraImpulse` generates a downward Cinemachine impulse. Both subscribe via GetComponent in Awake, paired OnEnable/OnDisable.
- **`sensorRange` defines GROUNDED, not just sensing.** Between `hoverHeight` and `sensorRange` the springs produce zero lift but the craft still counts as grounded, so drag and leveling torque apply, air control stays off, and the grounded jump path is used. Two consequences: do not widen `sensorRange` casually, and hard-landing speed is sampled on that edge rather than at ride height, so measured landing speed runs several percent below the closed form and raising `sensorRange` makes hard landings *less* likely.
- **`maxReverseAccel` doubles as the brake.** Pulling reverse while moving forward decelerates at that rate, so it is set by stopping distance, not by reverse speed.
- **`HoverDebugSettings` ScriptableObject:** Optional global gizmo toggle. The four hover scripts plus `RocketProjectile` and `ParticleWeaponCollision` check it first, then fall back to their local `drawDebug` bool. Assign it on `Missile.prefab` and the weapon emitters to bring impact draws under the same switch as everything else.

---

## Architecture Principles
- **Opposing forces cause jitter.** Prefer mutually exclusive force application over tuning competing forces against each other.
- **Timestep mismatch is a jitter source.** Default 50Hz Fixed Timestep against high-refresh displays produces visible jitter. Done: 100Hz (0.01) + Rigidbody interpolation on both vehicle prefabs.
- **Update/FixedUpdate ordering is a real bug surface.** Use timestamps (`Time.unscaledTime`) over frame-scoped booleans for cross-boundary state.
- **ScriptableObjects for shared data.** Vehicle-specific scene references belong in slots, not definitions.
- **`DefaultExecutionOrder` is a last resort.** Only use when there is a clear, justified architectural reason.
- **`ForceMode.Force` and `ForceMode.Acceleration` are timestep-independent.** Confirmed.

---

## Known Issues & Pending Tuning

> **Active handoff: see `HANDOFF.md`.** Drift and flip recovery are the two open bugs, with everything already ruled out (and the evidence) recorded there so it is not re-investigated. `PhysicsAudit.md` holds the derivations but is a snapshot of `4a34f21` -- its method is current, its values are not. **Always read live values from `Assets/Data/VTP_Default.asset`, never from a doc.**

- **Drift is untestable: the vehicle flips immediately**, at `maxBankAngle` 18 and at 12. Ruled out by direct measurement: the banked hull cannot reach the ground (~6.9m belly clearance), bank shifts the centre of mass only 2.7cm, inertia magnitudes are unchanged, and a 3.0 rad/s yaw produces no roll coupling (tilt stayed at 0.22 deg). Still open: the bank writes to `meshRoot` (`3D/HoverCar`) which **owns all five mesh colliders**, so a system documented as "visual only" does rotate the inertia tensor basis by the full bank angle; and `driftLateralDamp` is 0, meaning drift removes lateral resistance entirely. **Not reproducible without a live controller** -- `driftLerp` and `_strafeModeBlend` are recomputed every FixedUpdate from input, so they cannot be forced via reflection.
- **Flip recovery fails in at least one resting pose** (nose-down on a rounded hull face; HUD showed the timer counting but never completing). Recovery works from synthetic 88 and 95 degree poses, so it is not simply broken. Leading hypothesis: `flipRecoverySpeedThreshold` was too tight at 0.5, and a curved resting face micro-rocks above it, resetting `flipTimer` forever. **Raised to 2 speculatively; unverified.** The gizmo now names the blocking gate live -- capture that label in the stuck state before changing any code.
- Bullets eaten on flat ground: **the layer-mask theory is disproved.** Every scene emitter overrides the prefab with a correct per-vehicle mask (player 129 = Default+AIVehicle, AI 65 = Default+PlayerVehicle), and self-collision is impossible because the own-layer bit is never set. The prefab asset was separately misconfigured (mask 55, no vehicle layers) and has been fixed, but that only affected newly created instances. **Cause still open.**
- Weapon knock-around pass is queued but not applied; see Phase 3 of `HANDOFF.md`.
- Resolved: hover springs no longer sag. Gravity feedforward eliminated the ~1m offset entirely, on flat ground and slopes.
- Resolved: strafe forward dead band; boost in reverse; `strafeTopSpeed` raised (now 40 against a top speed of 60).
- Resolved: the audit's open question on the spin ceiling. Inertia is `(3294, 3756, 1091)`, so roll is ~3x easier to induce than pitch, and `destabilizeFraction` saturation does begin just above 0.3. The tooltip was correct.
- **`damage: 0` on `WD_Missile`, `WD_SoftHomingMissile`, `WD_HardLockMissile`, `WD_Shotgun`.** These four apply knockback but never reduce health. Not cosmetic: `HoverController_Weapons` passes `def.damage` to the projectile via `SetDamage`, and `ParticleWeaponCollision` reads it directly, so both delivery paths are live and both are being handed zero. Only `WD_ChainGun` (1) and `WD_MachineGuns` (0.5) deal damage today. Needs a damage pass, not a one-line fix. The inspector now warns on any definition sitting at 0.
- **Blast geometry still lives on the projectile prefab.** `splashRadius`, `splashFalloff`, and `damageLayers` did not move to `WeaponDefinition` with the impact forces, because splash radius has to match what the explosion VFX draws and those VFX are placeholder. Blocked on final explosion art, not on design. When it moves, `RocketProjectile` keeps only flight values and `IProjectileImpactCarrier` grows the extra parameters.
- `WD_Shotgun` serializes `missileFireMode: 2` (HardLock). Meaningless on a `SingleShot` weapon and never read; it is the C# default landing in YAML after the `useMissileLock` rename. Now hidden by the custom inspector. Left as-is rather than hand-editing a serialized value.
- Resolved: `driftLateralDamp` already 0 in VTP_Default; Fixed Timestep already 0.01 (normalized the rational count to exact).
- Resolved: weapon/tuning assets moved from `Assets/Scripts/` to `Assets/Data/`, matching Workspace Conventions. All seven via `AssetDatabase.MoveAsset`, GUIDs preserved.
- Resolved: stale `useMissileLock` key (removed from the class in the `missileFireMode` rename) was still serialized in `WD_MachineGuns`. Cleared with `AssetDatabase.ForceReserializeAssets` across `Assets/Data/`.
- Resolved: impact force no longer lives in two places. `WeaponDefinition` now owns `impactForce`, `splashImpactForce`, and `destabilizeFraction` for both delivery paths, pushing them onto spawned projectiles via `IProjectileImpactCarrier`. Migration preserved behaviour: the three missile definitions were seeded with the values `Missile.prefab` held (100000 / 50000 / 0.15), and `WD_Shotgun`'s `destabilizeOnImpact: true` became `destabilizeFraction: 1`, which is the same thing. Verified live by instantiating each missile prefab and reading back the private fields after the interface call.

---

## Design Pillars (Guardrails)
Reject or redesign any feature that violates these:

- **Hover is persistent.** Always airborne. Not a modifier or special state.
- **Momentum is the primary skill expression.** Positioning and momentum management determine outcomes, not loadouts, stats, or aim in isolation.
- **Combat and traversal are the same skill.** There is no mode switch. Map reading, momentum management, and weapon use happen simultaneously.
- **Hit disruption over flat damage.** Being hit should disrupt momentum first, health second. Weapons that manipulate momentum vectors are prioritized over weapons that simply deal damage.
- **No asymmetric loadouts.** All vehicles share the same weapons and abilities. Identity comes from handling profile and one unique special weapon only.
- **Vehicles change how, not whether.** Every vehicle can execute every action. Tuning changes the expression of those actions, not their availability. The moment a player feels they need a specific vehicle for a specific situation, the design has crossed into hero shooter territory.
- **Special weapons are exclamation points, not answers.** A special weapon must not be the solution to a specific tactical problem. It expresses the vehicle's identity spectacularly. It does not define what the vehicle can do.
- **Pickup placement creates vulnerability.** Powerful pickups live in exposed positions. Collecting them costs positional safety. Map control is the mechanism for forcing engagement.
- **Clarity at high speed.** Visual readability is a gameplay requirement. Spectacle that reduces readability is a design failure.
- **Feel first.** Mechanics must be fun before they are beautiful.

### Arena Design Guardrails

Reject or redesign any arena that violates these:

- **Track with rooms.** Corridors build momentum and create pursuit pressure. Rooms are where fights happen. Both must be present.
- **Three vertical layers.** Ground, mid, and high level must each serve a distinct tactical purpose. Verticality is a choice with consequence, not scenery.
- **Parallel routes at every major node.** At least two routes of roughly equal travel time. Single-path layouts kill the head-off play and collapse the meta.
- **Corridors are not transitions.** They are where momentum builds. Width must accommodate vehicle turning radius at combat speed.
- **Falling from height is a tactic.** Descent must be dramatic enough that following is genuinely risky. The escaping player chooses it intentionally because they know the map. The pursuer does not.

---

## Systems Reference

### Energy System
Shared non-damaging ability resource. Governs mobility/utility only, not weapons. Regenerates over time when idle. Depleted by: Boost, Jump, Shield, EMP.

### Shared Abilities
| Ability | Input | Cost | Contract (when it succeeds) |
|---|---|---|---|
| Boost | Hold | Continuous drain | Repositioning, not permanent speed |
| Jump | Tap / Hold to charge | Flat cost | Verticality meaningfully disrupts targeting |
| Shield | Tap | Flat cost | Timed, not spammed. Absorbs one incoming EMP hit (shield deactivates, no freeze) |
| EMP | Tap | ~70-80% of meter | Soft-homing projectile, direct hit only. Unshielded hit applies freeze; shielded hit destroys shield, no freeze. Denies tempo, not control |

### Weapons
All vehicles share the full roster. Vehicle identity = handling profile + one unique special. All weapons use limited pickup ammo except the Machine Gun (infinite).

| # | Weapon | Input | Notes |
|---|---|---|---|
| 1 | Machine Gun | Hold | Infinite ammo. Low DPS, always available, requires exposure commitment |
| 2 | Minigun | Hold | Wind-up/wind-down. Fire rate scales via AnimationCurve |
| 3 | Shotgun | Tap | Short range burst; succeeds only at lethal proximity |
| 4 | Rocket Launcher | Tap | High damage, blast radius, high outward force. Primary momentum disruption tool |
| 5 | Soft Homing Projectile | Tap | Homes nearest target. Low damage/force, medium fire rate. Harassment tool |
| 6 | Hard Lock Projectile | Hold to lock, tap to fire | Single guided projectile. Expandable to multi-target cascade |
| 7 | Sniper / Lightning Bolt | Tap | Zoom scopes view; blind outside scope. Instant hit, high damage. Strafe mode during zoom |
| 8 | Laser Cannon | Hold to charge, release | Sustained beam, pierces targets. Short charge = weak; full charge = peak window that tapers |
| 9 | Gravity Well / Repulsor | Tap to deploy | Lobbed deployable. Pulls/pushes and drains caught vehicles. One active; does not affect deployer |
| 10 | Bouncing Disc Blade | Tap | Ricochets in 3D space. Rewards map literacy and spatial prediction |
| 11 | Floating Proximity Mine | Tap to deploy | Suspended at hover height. Omnidirectional trigger; visible detection field |
| 12 | Directional Remote Mine | Tap to deploy, tap to trigger | Attaches to surfaces. Fires outward from placement angle on manual trigger |
| 13 | Special | n/a | One unique per vehicle. Allowed to bend shared rules; provides spectacle |

#### Weapon Implementation Status
Definition assets are `Assets/Data/WD_*.asset`, alongside `VTP_Default.asset`. Moved there from `Assets/Scripts/` via `AssetDatabase.MoveAsset`, so GUIDs and every existing reference are intact.

| Weapon | Status | Definition asset | Delivery |
|---|---|---|---|
| Machine Gun | Implemented | `WD_MachineGuns` | Particle emitter + `ParticleWeaponCollision` |
| Minigun ("Chain Gun") | Implemented | `WD_ChainGun` | Particle emitter + `ParticleWeaponCollision` |
| Shotgun | Implemented | `WD_Shotgun` | Particle emitter, 40-pellet burst, `destabilizeOnImpact` ON |
| Rocket Launcher | Implemented | `WD_Missile` | `Missile.prefab` + `RocketProjectile` |
| Soft Homing Projectile | Implemented | `WD_SoftHomingMissile` | `SoftHomingMissile.prefab`, variant of `Missile.prefab` |
| Hard Lock Projectile | Implemented | `WD_HardLockMissile` | `HardLockMissile.prefab`, variant of `Missile.prefab` |
| Sniper / Lightning Bolt | Planned | | |
| Laser Cannon | Planned | | |
| Gravity Well / Repulsor | Planned | | |
| Bouncing Disc Blade | Planned | | |
| Floating Proximity Mine | Planned | | |
| Directional Remote Mine | Planned | | |

The three missile prefabs are a base plus two variants. The split of authority is:

- **`WeaponDefinition` (per weapon):** damage, `impactForce`, `splashImpactForce`, `destabilizeFraction`. Pushed onto the spawned projectile via `IProjectileImpactCarrier`. All three currently carry identical values (100000 / 50000 / 0.15) because that is what the prefab used to impose on all of them; they are now free to diverge, which the GDD wants (soft homing is specified as "low damage and force").
- **Prefab (per projectile):** flight (`speed`, `lifetime`, `armingDelay`, `turnRate`) and blast geometry (`splashRadius` 10, `splashFalloff`, `damageLayers`), which must stay matched to the explosion VFX. Only `turnRate` differs per variant (0 / 90 / 120); everything else is inherited from `Missile.prefab`.

Blast geometry is the one weapon-shaped thing the definition does not own, and that is a workaround, not a boundary. It stays on the prefab only because splash radius has to agree with whatever the explosion VFX draws, and the current explosions are strictly placeholder. When the real VFX land it should move onto the `WeaponDefinition` with the forces, leaving the prefab as pure delivery.

### Pickups
Ammo (restores secondary ammo), Energy Cell (rapid recharge), Health Repair.

---

## Workspace Conventions
- Core logic: `Assets/Scripts/*.cs`
- Data-driven values: `Assets/Data/` (ScriptableObjects)
- Input: Unity Input System actions
- Camera: Cinemachine

## Prototype Checklist
- [x] Project, repo, IDE integration
- [x] Hovercraft movement & physics
- [x] Camera and input control
- [x] Energy + ability system
- [ ] Basic combat loop (primary fire + pickups)
- [ ] AI opponent

For detailed design, weapon contracts, market positioning, and visual style: see `GameDesignDocument.md`.

For the current engineering state, the two open bugs, what has already been ruled out, and the MCP recipes that reproduce each measurement: see `HANDOFF.md`. For physics derivations and method (values superseded): see `PhysicsAudit.md`.