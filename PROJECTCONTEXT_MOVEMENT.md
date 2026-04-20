# Hover Combat Project Context — Movement

> Extracted from PROJECT_CONTEXT.md

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

**Spawn ? Locate enemies ? Pressure when strong ? Evade and recover when weak ? Re-engage.**

Primary mastery axis: **tempo control** — deciding when fights happen and when to disengage.

- Offense creates exposure
- Exposure creates punish windows
- Momentum shifts define outcomes

**Initial scope:** 1v1 combat vs. AI, expanding to 3–4 AI opponents.
