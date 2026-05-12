# Hover Combat Prototype — Claude Context

## Behavioral Rules
- Read existing files before writing. Do not re-read unless the file has changed.
- Thorough in reasoning, concise in output.
- Skip files over 100KB unless the task explicitly requires them.
- No sycophantic openers or closing fluff.
- No em-dashes.
- Do not guess APIs, versions, flags, commit SHAs, or package names. Verify by reading code or docs before asserting.

---

## Project Summary
Ground-level aerial dogfighting game. Hover vehicles behave like low-altitude jets. Inspired by Twisted Metal and Rocket League. Unity 6.3 URP. Single-player prototype; 1v1 vs AI, expanding to 4-5 opponents.

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
| `HoverController_Foundation.cs` | v1.0 | **Do not modify without explicit justification.** Spring-damper hover lift, leveling torque, two-path recovery (upright-stuck vs flipped). Public: `IsHoverGrounded`, `AverageGroundNormal`, `SetRecoveryEnabled(bool)` |
| `HoverController_Propulsion.cs` | v1.0 | Drive/drag mutually exclusive; top speed in `ApplyDrive`; strafe dodge burst; continuous boost requires >= 0.15 throttle deflection. Public: `DriftLerp`, `StrafeModeBlend`, `StrafePitchLimit`, `OnJumpDenied` event |
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
- **Propulsion → Aim:** Aim reads `transform.eulerAngles` directly (actual vehicle orientation); no reference to Propulsion needed.
- **`HoverDebugSettings` ScriptableObject:** Optional global gizmo toggle. All four scripts check it first, then fall back to their local `drawDebug` bool.

---

## Architecture Principles
- **Opposing forces cause jitter.** Prefer mutually exclusive force application over tuning competing forces against each other.
- **Timestep mismatch is a jitter source.** Default 50Hz Fixed Timestep against high-refresh displays produces visible jitter. Pending: change to 100Hz (0.01).
- **Update/FixedUpdate ordering is a real bug surface.** Use timestamps (`Time.unscaledTime`) over frame-scoped booleans for cross-boundary state.
- **ScriptableObjects for shared data.** Vehicle-specific scene references belong in slots, not definitions.
- **`DefaultExecutionOrder` is a last resort.** Only use when there is a clear, justified architectural reason.
- **`ForceMode.Force` and `ForceMode.Acceleration` are timestep-independent.** Confirmed.

---

## Known Issues & Pending Tuning
- Bullets eaten on flat ground during acceleration -- suspected particle collision layer mask issue (emitters hitting vehicle's own collider). Cyan debug ray confirmed stable and level; not a pitch problem.
- Raise `strafeTopSpeed` toward 45 to better match forward top speed of 60.
- Drop `driftLateralDamp` toward 0 (gap from `lateralDamp=1` is imperceptible).
- Change Fixed Timestep from 0.02 (50Hz) to 0.01 (100Hz).

---

## Design Pillars (Guardrails)
Reject or redesign any feature that violates these:
- Hover is persistent. Always airborne; not a modifier or special state.
- Movement is the primary skill expression. Momentum and positioning determine outcomes, not loadouts or stats.
- No asymmetric loadouts. All vehicles share the same weapons and abilities. Identity comes from handling profile + one unique special weapon only.
- Movement disruption over flat damage. Destabilization, knockback, and tempo denial are prioritized over raw DPS.
- Clarity at high speed. Visual readability must be maintained during fast, chaotic play.
- Feel first. Mechanics must be fun before they are beautiful.

---

## Systems Reference

### Energy System
Shared non-damaging ability resource. Governs mobility/utility only, not weapons. Regenerates over time when idle. Depleted by: Boost, Jump, Shield.

### Shared Abilities
| Ability | Contract (when it succeeds) |
|---|---|
| Boost | Repositioning -- not permanent speed increase |
| Jump | Verticality meaningfully disrupts targeting |
| Shield | Timed, not spammed |
| EMP/Freeze | Denies tempo, not just deletes player control |

### Weapons
- **Machine Gun** -- infinite ammo, low DPS, requires exposure commitment
- **Missile** -- dumbfire or hard-lock; one target, no deferred execution
- **Shotgun Burst** -- close range; only succeeds when lethal proximity is achieved
- **Rail/Lightning Gun** -- high precision; only succeeds when aim is maintained under chaos
- **Ricochet Disk / Mines / Hazard Field / Multi-target** -- planned secondaries; see GDD for contracts

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