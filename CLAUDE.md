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
| `HoverController_Foundation.cs` | v1.0 | **Do not modify without explicit justification.** Spring-damper hover lift (`ForceMode.Acceleration` = mass independent; sensors ignore triggers), leveling torque as the **single attitude authority** (ground normal + aim pitch target; aim strength applies to pitch axis only; `aimPitchDamping` adds aim-only settle damping independent of ride stiffness), two-path recovery (upright-stuck vs flipped; unstick arms only while vertically settled so belly scrapes don't get pulse trains). Ground contact tracked by timestamp, not exit callback. Public: `IsHoverGrounded`, `AverageGroundNormal`, `SetRecoveryEnabled(bool)`, `SetAimPitch(degrees, weight)` |
| `HoverController_Propulsion.cs` | v1.0 | Drive/drag mutually exclusive; top speed in `ApplyDrive` with exact-cap clamp (crossing tick lands exactly on the cap, no ripple); lateral damp excludes intended strafe velocity (`strafeTopSpeed` is the real lateral ceiling); lateral drive gated at the cap with a gentle over-speed bleed, so dodge bursts at the cap are **additive** (surge, then ~1s glide back down); strafe dodge burst; continuous boost requires >= 0.15 throttle deflection. Strafe pitch applies **no torque** -- it hands the accumulated target to `Foundation.SetAimPitch`. Public: `DriftLerp`, `StrafeModeBlend`, `StrafePitchLimit`, `OnJumpDenied` event |
| `HoverController_Energy.cs` | v1.0 | `TryConsume` fail also resets regen lockout (intentional anti-spam). EMP freeze additive. Public: `Energy`, `EnergyNormalized`, `IsEmpFrozen`, `IsRegenerating` |
| `HoverController_Weapons.cs` | v1.0 | Weapons **never spend energy**. EMP only gates fire via `IsEmpFrozen`. `WeaponDefinition` SO (shared); `WeaponSlot` (per-vehicle). Four `WeaponType`s: SingleShot, Automatic, Missile, Mine. Two `ProjectileMode`s: Instantiated, ParticleSystem. Public: `ActiveSlot`, `ActiveSlotIndex`, `CurrentLockState`, `LockProgress`, `LockTarget`, `RefillAmmo()`, `SetActiveSlot()` |
| `HoverController_Aim.cs` | v1.0 | **LateUpdate only; no `DefaultExecutionOrder` needed.** Reads actual vehicle pitch from Rigidbody (no independent accumulation). Only rotates active slot's `vfxMount`; Instantiated-mode weapons unaffected. Called by Weapons via `NotifySlotChanged(WeaponSlot)` |
| `HoverController_Shield.cs` | v1.0 | Fixed-duration invulnerability burst. Flat energy cost on activation; cannot be cancelled by player. Active shield absorbs an incoming EMP (shield deactivates, no freeze). New activation still blocked while already EMP-frozen. Public: `IsActive`, `TimeRemaining`, `OnShieldActivated`, `OnShieldDeactivated`, `TryActivate()`, `TryAbsorbEmp()` |
| `VehicleHealth.cs` | Active | HP pool, `OnDamaged`/`OnDeath` events, invulnerability (post-respawn grace and shield-driven), `Respawn()` |
| `VehicleHUD.cs` | v1.2 | Event-driven; `SyncAll()` in Start; trailing `_wasRegenerating` flag |
| `HoverCameraController.cs` | Active | Cinemachine 3.1.6; strafe cam uses `LockToTargetNoRoll` |
| `ParticleWeaponCollision.cs` | Active | Particle-based weapon hits; implements `IDamageable` |
| `PickupManager.cs` | Planned | Not yet implemented |
| AI FSM | Planned | Phase 2; chase/evade/shoot |

## Inter-Module Wiring

- **Input:** All HoverController scripts acquire input via `GetComponent<IHoverInputProvider>()` in Awake. No inspector wiring. Swap `PlayerHoverInput` for an AI component to change who drives the vehicle. Key input props: `ThrottleInput`, `StrafeX`, `TurnInput`, `Boost`, `Drift`, `Jump`, `StrafeHeld`, `CameraLookY`, `FirePressed`, `FireHeld`, `CycleWeaponNext/Prev`, `ShieldPressed`.
- **Foundation → Propulsion:** Propulsion reads `Foundation.IsHoverGrounded` each FixedUpdate to gate throttle, drag, and drift.
- **Energy → Propulsion/Weapons/Shield:** Propulsion calls `TryConsume` per frame for boost (continuous) and once for jump/dodge (instantaneous). Shield calls `TryConsume` once on activation. Weapons only reads `IsEmpFrozen` -- no energy is spent on firing.
- **Shield → Health:** `VehicleHealth.TakeDamage` short-circuits when `_shield.IsActive`. Shield owns its own invulnerability state; the existing `_isInvulnerable` flag remains for post-respawn grace.
- **Energy → Shield:** `EmpProjectile.Consume` calls `Shield.TryAbsorbEmp` on hit; if true the freeze is skipped. Shield also subscribes to `Energy.OnEmpFreezeApplied` as a defensive fallback. New activation is blocked while `IsEmpFrozen`.
- **Weapons → Aim:** Weapons calls `aim.NotifySlotChanged(slot)` on slot change. Aim caches the new `vfxMount`.
- **Propulsion → Foundation (aim pitch):** Propulsion calls `foundation.SetAimPitch(accumulatedDegrees, strafeBlend)` every FixedUpdate; `(0, 0)` on strafe exit and EMP freeze. Foundation's leveling torque drives toward the target (`aimPitchTrackingStrength`, pitch axis only); Propulsion never applies pitch torque itself. One attitude authority, no competing forces.
- **Propulsion → Aim:** Aim reads `transform.eulerAngles` directly (actual vehicle orientation); no reference to Propulsion needed.
- **`HoverDebugSettings` ScriptableObject:** Optional global gizmo toggle. All four scripts check it first, then fall back to their local `drawDebug` bool.

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
- Bullets eaten on flat ground during acceleration -- suspected particle collision layer mask issue (emitters hitting vehicle's own collider). Cyan debug ray confirmed stable and level; not a pitch problem.
- Raise `strafeTopSpeed` toward 45 to better match forward top speed of 60. Since lateral damp now excludes intended strafe velocity, this needs no `strafeAccel` compensation; the knob works directly.
- Playtest tune: `aimPitchTrackingStrength` (150, pitch axis only) with `aimPitchDamping` (8; aim-only settle damping, independent of `pitchRollDamping` ride stiffness); `unstickMaxVerticalSpeed` (1).
- Hover springs sag ~1m below `hoverHeight` at rest (weight accel ~39 m/s^2 vs spring accel 40/m at 4 points); pre-existing, documented in the liftStrength tooltip. Gravity feedforward is a possible future fix.
- Resolved: `driftLateralDamp` already 0 in VTP_Default; Fixed Timestep already 0.01 (normalized the rational count to exact).

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
| Weapon | Status |
|---|---|
| Machine Gun | Stubbed |
| Minigun | Stubbed |
| Shotgun | Stubbed |
| Rocket Launcher | Stubbed |
| Soft Homing Projectile | Stubbed |
| Hard Lock Projectile | Stubbed |
| Sniper / Lightning Bolt | Planned |
| Laser Cannon | Planned |
| Gravity Well / Repulsor | Planned |
| Bouncing Disc Blade | Planned |
| Floating Proximity Mine | Planned |
| Directional Remote Mine | Planned |

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