# Hover Combat Project Context — Technical

> Extracted from PROJECT_CONTEXT.md

## 15. Technical Architecture

### Engine & Stack
- **Engine:** Unity 6.2 (URP template) — no HDRP
- **Physics:** Rigidbody-based hover vehicle controller
- **Input:** Unity Input System (Vector2 action for stick input; axes split in code)
- **Camera:** Cinemachine 3.1.6 third-person
- **Version Control:** GitHub + Fork + Rider

### Module Overview

| Module | Status | Notes |
|---|---|---|
| `HoverController_Foundation.cs` | Stable | Do not modify without clear justification |
| `HoverController_Propulsion.cs` | v5.1 | Drive/drag mutually exclusive; top speed enforced inside `ApplyDrive` |
| `HoverController_Energy.cs` | Fixed | Uses `lastConsumeTime` timestamp; no frame-scoped flags |
| `HoverController_Weapons.cs` | Active | `WeaponDefinition` as ScriptableObject; `WeaponSlot` holds vehicle-specific refs |
| `HoverController_Aim.cs` | Active | Tracks `CameraLookY` + yaw only; `[DefaultExecutionOrder(-10)]` required |
| `VehicleHealth.cs` | Active | HP pool, `OnDamaged`/`OnDeath` events, invulnerability, `Respawn()` |
| `VehicleHUD.cs` | v1.2 | Event-driven; `SyncAll()` in Start; trailing `_wasRegenerating` flag |
| `HoverCameraController.cs` | Active | Cinemachine 3.1.6; strafe cam uses `LockToTargetNoRoll` |
| `ParticleWeaponCollision.cs` | Active | Handles particle-based weapon hits; implements `IDamageable` |

### Planned Modules (Not Yet Implemented)
- `PickupManager.cs` — spawns and manages map pickups
- AI FSM — chase, evade, shoot behavior (Phase 2)

### Established Architecture Principles
- **Opposing forces cause jitter.** Prefer mutually exclusive force application over tuning competing forces against each other.
- **Timestep mismatch is a jitter source.** Default 50Hz Fixed Timestep against high-refresh displays produces visible jitter. Recommended: change to 100Hz (0.01).
- **Update/FixedUpdate ordering is a real bug surface.** Use timestamps (`Time.unscaledTime`) over frame-scoped booleans for cross-boundary state.
- **ScriptableObjects for shared data.** Vehicle-specific scene references belong in slots, not definitions.
- **`DefaultExecutionOrder` is a last resort.** Only use when there is a clear, justified architectural reason.
- **Foundation is stable.** Avoid touching `HoverController_Foundation.cs` without explicit justification.
- **`ForceMode.Force` and `ForceMode.Acceleration` are timestep-independent.** Confirmed.

### Active Known Issues
- Bullets being eaten on flat ground during acceleration — suspected particle collision layer mask issue (emitters hitting vehicle's own collider). Cyan debug ray confirmed stable and level; not a pitch problem.

### Pending Tuning (Not Yet Applied)
- Raise `strafeTopSpeed` toward 45 to better match forward top speed of 60
- Drop `driftLateralDamp` toward 0 (gap from `lateralDamp=1` is imperceptible)
- Change Fixed Timestep from 0.02 (50Hz) to 0.01 (100Hz)

---

## 16. Prototype Completion Checklist

- ? Project, repo, and IDE integration
- ?? Hovercraft movement & physics
- ?? Camera and input control
- ?? Energy + ability system
- ?? Basic combat loop (primary fire + pickups)
- ?? AI opponent

*Last consolidated: 2026-03-23*
*Sources: Game Design Document v1, Combat Design Bible, Market & Identity, Publishing & Creative Leadership Summary*
