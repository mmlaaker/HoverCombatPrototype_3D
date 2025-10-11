# 🕹️ Hover Combat Prototype
*A ground-level aerial dogfighting game inspired by **Twisted Metal** and **Rocket League.***

---

## 1. Project Overview
This project is a small-scale, single-player prototype exploring hover-vehicle combat at ground level — where vehicles behave like aerial dogfighters but without the constant forward motion of aircraft.

The goal is to develop **fun, physical, skill-based movement and combat** in a confined arena environment.  
Initial scope: **1v1 combat**, expanding to 3–4 AI opponents.

---

## 2. Core Design Pillars
1. **Hybrid Controls:**  
   - A blend between *Twisted Metal*’s weighty vehicle steering and *Rocket League*’s responsive, air-like maneuvering.  
   - Hovercrafts can strafe and jump, offering omnidirectional control.
2. **Energy-Based Abilities:**  
   - Boost, jump, and shields consume energy — a shared non-damaging ability resource.  
   - Players manage energy to stay mobile and survive.
3. **Pickup-Based Weaponry:**  
   - Infinite low-damage machine gun.  
   - Secondary weapons (missiles, shotgun bursts, etc.) rely on limited ammo pickups.  
   - Map control becomes strategic.
4. **Arena Combat Loop:**  
   - Players chase, strafe, dodge, and time ability usage while managing energy and ammo.  
   - Focus on “feel first” — tuning handling and responsiveness before visual polish.

---

## 3. Gameplay Summary
| System | Description |
|--------|--------------|
| **Camera** | Third-person follow camera with mild positional lag and rotation offset controls. |
| **Vehicle Movement** | Hovercrafts use a Rigidbody-based controller. Ground proximity forces stabilize hover height. Steering blends drift and strafing. |
| **Combat** | Forward-facing weapons; player aims primarily via vehicle orientation. No soft lock or perfect tracking. |
| **Abilities** | Non-damaging utility powers that consume shared energy: <br>• **Boost** – temporary acceleration burst.<br>• **Jump** – instant hover lift.<br>• **Shield** – temporary damage resistance. |
| **Weapons** | • **Machine Gun** (infinite ammo, low DPS) <br>• **Missile** (forward shot, medium DPS) <br>• **Homing Missile** (requires lock, high DPS) <br>• **Shotgun Burst** (short range, spread) |
| **Pickups** | Map-scattered items for ammo, health, and energy recharge. |
| **Game Modes** | 1v1 versus AI opponent (initial). Later expansion: 3–4 combatants per match. |

---

## 4. Technical Overview

| Category | Implementation |
|-----------|----------------|
| **Engine** | Unity 6.2 (URP template) |
| **Physics Base** | Rigidbody-based hover vehicle controller |
| **Input** | Unity Input System (single-player bindings for keyboard/controller) |
| **Camera System** | Third-person follow camera with aim assist offset |
| **Core Resources** | • **Energy Meter:** shared pool for mobility & utility<br>• **Ammo:** per-weapon pickup pools |
| **Art & VFX** | Placeholder meshes and particle effects. Visual polish deferred until mechanics are fun. |
| **AI (Phase 2)** | Simple finite-state machine (FSM) for chasing, evading, and shooting behavior. |

---

## 5. Control Scheme (Prototype)
| Action | Key / Binding | Notes |
|--------|----------------|-------|
| Thrust Forward | `W` / Left Stick Up | Standard acceleration |
| Reverse / Brake | `S` / Left Stick Down | Reduces forward momentum |
| Turn Left / Right | `A` / `D` / Left Stick X | Yaw rotation |
| Strafe Left / Right | `Q` / `E` or Right Stick X | Lateral movement |
| Jump | `Spacebar` / Face Button | Consumes energy |
| Boost | `Left Shift` / Trigger | Short acceleration burst |
| Fire Primary | `Left Mouse` / RT | Infinite ammo |
| Fire Secondary | `Right Mouse` / LT | Requires pickup ammo |
| Activate Shield | `F` / Face Button | Energy cost, brief invulnerability |
| Camera Orbit | Mouse / Right Stick | Free rotation around vehicle |
| Reset Orientation | `R` | Upright correction if flipped |

---

## 6. Vehicle Behavior Breakdown

**Hover Physics**
- Uses downward raycasts to maintain a target hover height.  
- Proportional lift based on distance from the ground.  
- Stabilization torque to keep level pitch/roll.  

**Steering**
- Hybrid between car-like torque and strafing thrust.  
- Slight drift inertia on yaw.  
- Strafing allows fine aim correction without hard turns.

**Jump & Boost**
- Add instantaneous upward or forward force (scaled by energy meter).  
- Recharge rate tuned to encourage rhythmic use rather than spam.

---

## 7. Combat Systems

**Weapon Slots**
- **Primary:** Infinite machine gun (low damage, accurate).  
- **Secondary:** Limited ammo (missiles, shotgun, etc.).  

**Energy System**
- Governs *non-damaging* abilities (boost, jump, shield).  
- Regenerates over time when idle.  
- Encourages pacing and decision-making.

**Pickups**
- **Ammo Pickup:** Restores secondary ammo.  
- **Energy Cell:** Rapidly recharges energy.  
- **Health Repair:** Restores hit points.

---

## 8. Prototype Scope
- **Single level:** small test arena with ramps and obstacles.  
- **Two hovercrafts:** Player and 1 AI.  
- **Functional prototype goals:**
  - Responsive hover movement.
  - Working boost, jump, and shield.
  - Primary & secondary weapon test.
  - Energy and ammo systems integrated.
  - Simple health and damage feedback.

---

## 9. Future Expansion Ideas
- Arena hazards (explosive barrels, shock fields).  
- Environmental effects (dust trails, hover distortion).  
- AI improvements (dodge, strafe combat awareness).  
- Multiplayer or split-screen support.  
- Upgradeable hovercraft stats or cosmetics.  

---

## 10. Visual Style Goals
- Stylized semi-realistic tone (clean readability > realism).  
- Emphasis on glowing energy and hover effects.  
- Clear silhouettes and color-coded player/AI vehicles.

---

## 11. Development Notes
- Use **URP** for performance and easy post-processing.  
- **No HDRP** (unnecessary overhead).  
- Keep project modular:  
  - `HovercraftController.cs` handles movement and forces.  
  - `EnergySystem.cs` governs ability usage.  
  - `WeaponSystem.cs` handles primary/secondary weapons.  
  - `PickupManager.cs` spawns pickups.  

---

## 12. Diagrams (Planned)
- Control Flow (Vehicle Input → Movement → Abilities → Energy System).  
- Interaction Map (Player ↔ Pickups ↔ Weapons ↔ UI).  
*(Will add via `.png` or `.drawio` export in `/Docs/` later.)*

---

## 13. Implementation Priorities
1. ✅ Establish project, repo, and IDE integration (complete).  
2. 🚧 Implement **hovercraft movement & physics.**  
3. 🚧 Add **camera and input control**.  
4. 🚧 Add **energy + ability system.**  
5. 🚧 Add **basic combat loop** (primary fire + pickups).  
6. 🚧 Add **AI opponent.**

---

## 14. Design Philosophy
> *“The fun comes from control.”*  
> Hover Combat is about mastering momentum and precision — being slightly overpowered, but always at risk of losing control.

---

## 15. Credits
**Design & Programming:** Meade Laaker (with ChatGPT collaboration)  
**Engine:** Unity 6.2 URP  
**Version Control:** GitHub + Fork + Rider integration  
