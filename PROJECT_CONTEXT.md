# Hover Combat Project — Consolidated Design Context

> This file is the authoritative design reference for AI-assisted development on this project.
> It consolidates the Game Design Document, Combat Design Bible, Market & Identity document,
> and Publishing & Creative Leadership Summary.
> All implementation decisions should be evaluated against the pillars defined here.

---

## 1. Project Identity

**Elevator pitch:**
A ground-level aerial dogfighting game inspired by Twisted Metal and Rocket League.

**One-sentence differentiator:**
A fast sci-fi hover combat game where momentum, spatial awareness, and timing determine fight outcomes — built around low-altitude dogfighting rather than traction-based car combat.

**Core player fantasy:**
Players feel they outplayed opponents through control, timing, and spatial awareness. Not a power fantasy, hero shooter, or stat-driven build game. Accessible chaos with depth beneath the surface.

**Emotional target:** *I won because I was better, not because I built better.*

**Design motto:** *"The fun comes from control."*
Hover Combat is about mastering momentum and precision — being slightly overpowered, but always at risk of losing control.

---

## 2. Design Pillars

These are non-negotiable. Any feature, mechanic, or system that conflicts with these pillars should be rejected or redesigned.

- **Hover is persistent.** Vehicles are always technically airborne. Hover is the primary movement mode, not a modifier or special state.
- **Movement is the primary skill expression.** Mastery of momentum and spatial positioning determines outcomes — not loadouts, builds, or stat optimization.
- **No asymmetric loadouts.** All players share the same weapons and abilities. Vehicle identity comes from handling profile and one unique special weapon only.
- **Execution mastery over configuration.** Skill is expressed through motion and timing, not menu decisions.
- **Movement disruption over flat damage.** Destabilization, knockback, and tempo denial are prioritized over raw DPS.
- **Clarity at high speed.** Visual readability and readable win moments must be maintained even during fast, chaotic play.
- **Feel first.** Tune handling and responsiveness before visual polish. Mechanics must be fun before they are beautiful.

---

## 3. Movement Model

**Physics philosophy:**
Vehicles are omnidirectional, vector-controlled crafts that feel like low-altitude jets dogfighting above the ground. Hover uses downward raycasts to maintain target height with proportional lift. Stabilization torque keeps level pitch/roll. The vehicle is always technically airborne — "grounded" only means close enough to the surface to push off.

**Movement modes:**

| Mode | Description |
|---|---|
| Normal Movement | Forward/reverse thrust with yaw steering; hybrid between car-like torque and strafing thrust with slight drift inertia on yaw |
| Strafe Mode | Twin-stick omni-directional movement with decoupled aiming; allows fine aim correction without hard turns |
| Forced Movement | Knockback, spinouts, destabilization, physics reactions |

**Jump & Boost:**
Add instantaneous upward or forward force scaled by the energy meter. Recharge rate tuned to encourage rhythmic use rather than spam.

---

## 4. Core Gameplay Loop

**Spawn → Locate enemies → Pressure when strong → Evade and recover when weak → Re-engage.**

Primary mastery axis: **tempo control** — deciding when fights happen and when to disengage.

- Offense creates exposure
- Exposure creates punish windows
- Momentum shifts define outcomes

**Initial scope:** 1v1 combat vs. AI, expanding to 3–4 AI opponents.

---

## 5. Energy System

Energy is a shared non-damaging ability resource. It governs all mobility and utility abilities — not weapons.

- Regenerates over time when idle
- Encourages pacing and decision-making over ability spam
- Depleted by: Boost, Jump, Shield

---

## 6. Shared Abilities (All Vehicles)

All abilities have explicit contracts defining when they succeed. Abilities that violate their contract are poorly used, not overpowered.

### Boost
- Primary horizontal tempo tool
- Used for engagement, disengagement, and missile evasion
- **Contract:** Succeeds when used for repositioning, not as a permanent speed increase

### Jump
- Vertical repositioning tool
- Used to break locks, bypass hazards, and create shotgun angles
- **Contract:** Succeeds when verticality meaningfully disrupts targeting

### Shield
- Timed survivability burst; primary missile counter
- **Contract:** Succeeds when timed, not spammed

### EMP / Freeze
- Energy disruption and movement suppression; punishes overextension
- **Contract:** Succeeds when it denies tempo, not when it simply deletes player control

---

## 7. Weapon Arsenal

All vehicles carry the same weapon set. The **machine gun is the only infinite-ammo weapon**. All other weapons consume pickup-acquired ammo. Map control over pickups is therefore strategically meaningful.

### Machine Gun (Primary — Infinite Ammo)
- Low damage, accurate, sustained forward pressure with wind-up
- Requires exposure commitment
- **Contract:** Only succeeds when the shooter accepts risk

### Missile System — Two Modes (Secondary)
- **Dumbfire:** No guidance; rewards prediction
- **Hard Lock:** Requires exposed lock phase; strong pursuit after commit
- **Rule:** One target only; no deferred execution; guidance requires commitment

### Homing Missile (Secondary)
- Requires lock-on; high DPS
- Distinct from dumbfire in guidance behavior

### Shotgun Burst (Secondary)
- Close-range burst damage; rewards spacing and vertical timing
- **Contract:** Only succeeds when lethal proximity is achieved

### Rail / Lightning Gun (Secondary)
- High precision, high instability sensitivity
- Rewards aim discipline under forced movement
- **Contract:** Only succeeds when precision is maintained under chaos

### Ricochet Disk (Secondary)
- Bounces off environment for indirect threat; rewards map literacy
- **Contract:** Only succeeds when arena geometry is weaponized

### Mines (Secondary)
- Delayed predictive traps; punishes predictable routing
- **Contract:** Only succeeds when future routing is correctly predicted

### Hazard Field (Secondary)
- Temporary zone denial and map editing; forces rerouting
- **Contract:** Only succeeds when space is controlled, not chased

### Multi-Target / Paint-and-Execute (Secondary)
- Hold to mark targets; release to strike all marked simultaneously
- Player is fully vulnerable during painting
- **Contract:** Only succeeds when cognitive load is managed under threat

---

## 8. Pickups

Map-scattered items create strategic map control pressure.

| Pickup | Effect |
|---|---|
| Ammo Pickup | Restores secondary weapon ammo |
| Energy Cell | Rapidly recharges energy meter |
| Health Repair | Restores hit points |

---

## 9. Vehicle Identity Layer

- **Roster size:** ~5 vehicles (target for vertical slice)
- **Shared systems:** All vehicles use the same energy system and weapon arsenal
- **Differentiation:** Handling profile defines baseline playstyle
- **Unique special weapon:** One per vehicle; allowed to bend standard rules and provide spectacle

### Planned Archetypes
- Balanced all-rounder
- Heavy momentum bruiser
- Light agile interceptor
- High-risk specialist (optional / stretch)

---

## 10. Arena Philosophy

Levels are kinetic spaces that reward movement control — not wide open bowls, but structured arenas that create tension and near-miss opportunities.

- Verticality matters
- Momentum corridors and choke spaces
- Movement literacy is rewarded
- Geometry is a weapon (ricochet, cover, routing prediction)

**Prototype arena:** Small test arena with ramps and obstacles. Single level.


---

## 12. Visual Style

- Stylized semi-realistic tone — clean readability over realism
- Emphasis on glowing energy and hover effects
- Clear silhouettes and color-coded player/AI vehicles
- Visual polish deferred until mechanics are confirmed fun

---

## 13. Market Positioning

- **Scope:** Mid-scope, skill-driven, feel-first action game — not a live-service hero ecosystem or blockbuster
- **Accessibility target:** Easy to pick up, hard to master; immediate fun within 30 seconds
- **Differentiator:** Low-altitude dogfighting movement model vs. traction-based car combat
- **Not this:** Hero shooter, stat-driven build game, power fantasy, spreadsheet-style progression

### Strategic Guardrails
- Hover must remain the primary movement mode at all times
- Avoid exposing spreadsheet-style stats to players
- Prevent hero-shooter level asymmetry
- Maintain visual clarity at high speed
- Keep movement central to identity in all design decisions

---

## 14. Publishing Strategy & Creative Direction

### Target Path
Publisher partnership post-prototype. Build vertical slice → pitch → scale responsibly. Self-publishing is viable but carries heavy marketing burden.

### Creative Director Role (Meade)
- Accountable for: vision clarity, milestone delivery, risk communication, scope discipline
- Not responsible for: netcode, marketing execution, implementing every system personally
- Transition required: from builder of systems → curator and protector of the experience

### Publisher Role
- Milestone-based funding, production support, QA, localization, marketing & PR, multiplayer engineering

### Vertical Slice Target (Phase 1 — 3–6 months)
- One arena
- 3–5 vehicles
- Strong controller feel
- Cohesive UX
- No multiplayer complexity

### What Gets Evaluated in a Pitch
- Risk predictability and scope discipline
- Judgment under pressure and communication clarity
- Confidence signals
- **Not** feature volume

---

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

- ✅ Project, repo, and IDE integration
- 🚧 Hovercraft movement & physics
- 🚧 Camera and input control
- 🚧 Energy + ability system
- 🚧 Basic combat loop (primary fire + pickups)
- 🚧 AI opponent

---

*Last consolidated: 2026-03-23*
*Sources: Game Design Document v1, Combat Design Bible, Market & Identity, Publishing & Creative Leadership Summary*
