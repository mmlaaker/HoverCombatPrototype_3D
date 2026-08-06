# Physics & Tuning Audit

**Date:** 2026-08-01
**Commit at audit:** `4a34f21`
**Purpose:** Establish what the current numbers actually do, ahead of a deep tuning pass.

> ## ⚠ Point-in-time snapshot. Values here are superseded.
>
> This documents the state at `4a34f21`, **before** the sedan tuning pass. The *derivations, formulas and method* are all still valid and worth reading. The *values* are not: ride height, damping, top speed, turn rate, jump, dodge and gravity have all moved since.
>
> - For current state and open work, read **`HANDOFF.md`**.
> - For current values, read **`Assets/Data/VTP_Default.asset`**. Never trust a number copied into a doc.
>
> Two things in this document have since been resolved and are corrected in `HANDOFF.md`:
> - The **spin-ceiling open question** is closed. The tooltip's claim was correct.
> - The **"bullets eaten" section** correctly records that the layer-mask theory was disproved, but the cause remains open.
>
> Superseded by this pass: the 1 m sag (eliminated by gravity feedforward), the strafe forward dead band (fixed), the underdamped hover (retuned), and slope compensation (removed as redundant).

## How to read this

Every number here is **derived from source code and serialized asset values**, not measured at runtime. The formulas were traced line by line through `HoverController_Foundation.cs` and `HoverController_Propulsion.cs`; the values come from `Assets/Data/VTP_Default.asset`, the `WD_*.asset` weapon definitions, the emitter prefabs, and `ProjectSettings/`.

Derivation is reliable for anything algebraic and less so for anything depending on the inertia tensor, which was not measured. Claims are marked where that matters.

**One hazard is worth stating up front:** the weapon emitters in `Prototype_Scene` carry prefab **overrides** on emission rate, burst count and collision mask, so the prefab assets and the live scene disagree. All weapon figures here were re-read from the running editor. An earlier draft of this audit trusted the prefabs and produced a wrong conclusion as a result; that correction is documented in the Fixed section below rather than quietly removed.

Three findings are worth confirming in play mode before you trust them fully: the **hover damping ratio**, the **47.2 m/s stacked-jump figure**, and the **strafe forward dead band**.

---

## Baseline constants

| Quantity | Value | Source |
|---|---|---|
| Chassis mass | 1000 kg | `HoverCar_PlayerController.prefab` |
| Hover points | 4 | prefab `hoverPoints` array |
| Fixed timestep | 0.01 (100 Hz) | `TimeManager.asset` |
| Unity gravity | 9.81 m/s² | `DynamicsManager.asset` |
| Total gravity (grounded) | **39.24 m/s²** | `extraGravityMultiplier` 3 is *additive* on top of Unity gravity |
| Total gravity (airborne) | **42.24 m/s²** | plus `extraAirGravity` 3 |
| Max angular speed | 50 rad/s | `DynamicsManager.asset:38` |
| Solver iterations | 12 | `DynamicsManager.asset` |

The gravity figure matters more than it looks. `extraGravityMultiplier: 3` does not mean "3x gravity", it means gravity plus three more of it, so the craft lives at **4x Earth gravity**. Almost every derivation below depends on that.

---

## Hover

`Foundation.cs:344` applies `a = liftStrength · compression − liftDamping · velocityAlongNormal` per point, as `ForceMode.Acceleration`, with **no division by point count**. Four points therefore multiply the authored numbers by four.

```
k = 4 × 10  = 40 s⁻²        (stiffness the chassis actually feels)
c = 4 × 1.5 =  6 s⁻¹        (damping)
```

### Resting height

The springs settle where lift balances weight:

```
sag = gravity / k = 39.24 / 40 = 0.98 m
```

With `hoverHeight: 5` the craft rests at **4.02 m**, not 5. This confirms the sag already noted in `CLAUDE.md` and gives it a closed form. A gravity feedforward of `39.24 / 4` per point would cancel it exactly and make `hoverHeight` mean what it says.

### Damping

```
ω = √40 = 6.32 rad/s  →  1.01 Hz
ζ = c / 2ω = 6 / 12.65 = 0.474
```

ζ 0.474 is **underdamped**:

- Overshoot `exp(−πζ/√(1−ζ²))` = **18.4%**
- 2% settling time `4/(ζω)` = **1.33 s**

Expect a soft float, and over a second of visible bob after landings and slope transitions. This is the single most likely source of "the hover feels vague" during the pass.

| Target | `liftDamping` |
|---|---|
| ζ 0.7 (firm, slight overshoot) | 2.21 |
| ζ 1.0 (no bounce at all) | 3.16 |

Retune this whenever `liftStrength` moves. The ratio is `(4 × liftDamping) / (2 √(4 × liftStrength))`.

### Slope compensation is effectively off

The ramp is `s = clamp01(1 − normal.y)`, which is cosine-linear rather than linear in angle:

| Slope | `s` | Multiplier at `slopeLiftMultiplier` 1.15 |
|---|---|---|
| 30° | 0.134 | **1.02x** |
| 45° | 0.293 | 1.04x |
| 90° | 1.000 | 1.15x |

At the authored 1.15, a 30° hill gets a 2% lift boost, which is nothing. The `[Range]` maximum is the value at a *vertical wall*, so any setting has to be far above what you want on real slopes.

Separately: compression is measured along the ray axis (`−point.up`) while the damping term and the applied force both use `hit.normal` (`Foundation.cs:342-344`). On a slope those differ by `1/cos θ`, so compression over-reads. Minor, but it means slope behaviour has two compounding quirks rather than one.

---

## Jump

`ForceMode.VelocityChange` on world up (`Propulsion.cs:572`), so the impulse **is** the launch speed in m/s and mass plays no part.

| | Value |
|---|---|
| Max-charge launch | 40 m/s |
| Apex height | **18.9 m** |
| Time to apex | 0.95 s |
| Return speed | **40 m/s** |
| Tap (min charge) apex | 2.7 m |

Because both jumps write velocity directly, stacking an air jump at the peak gives a landing speed that is **independent of gravity**:

```
√(40² + 25²) = 47.2 m/s
```

### The hard-landing threshold is crossed

`hardLandingMinSpeed` is **45**, so a max-charge jump followed by an air jump triggers the hard landing. Severity comes out at `(47.2 − 45) / (70 − 45)` = **0.087**, which means:

- Springs drop to 92% for 0.35 s (imperceptible)
- Camera impulse of 0.17 (barely there)
- A standard dust puff, since the heavy threshold is 0.6

So it is a **cosmetic false positive today, not a slam**. It matters for two reasons: the tooltip asserted 45 was safe when it is not, and severity climbs quickly if jump values go up during the pass. Setting `hardLandingMinSpeed` above 48 clears it.

The tooltip has been corrected to carry the formula rather than stale constants.

---

## Dodge

`Propulsion.cs:431-439` applies a linearly tapering acceleration, so the speed added is the area under the taper:

```
Δv = dodgeForce × dodgeDuration / 2 = 1000 × 0.1 / 2 = 50 m/s
```

**50 m/s is exactly full top speed, delivered sideways in a tenth of a second**, at a peak of 1000 m/s² (about 102 g). The class default would have given 15 m/s, so the authored value is more than three times the original design intent.

Since `strafeTopSpeed` is 30, a dodge puts you 20 m/s over cap. The over-speed bleed at `strafeLateralCapStrength` 3 has a 1/3 s time constant, so the excess decays about 95% in 1 second, matching what the tooltip claims.

This is the **largest readability risk** in the current tuning and sits directly against the "clarity at high speed" pillar. `dodgeDuration` 0.1 is also below its own tooltip's stated 0.15 to 0.35 range.

---

## Strafe: a forward dead band

Three separate systems govern forward speed, and in strafe mode they leave a gap.

- `ApplyDrive` clamps forward speed to `lerp(topSpeed, strafeTopSpeed, strafeBlend)` = **30** at full strafe
- `ApplyOverSpeedBleed` uses the **unblended** `effectiveTopSpeed` = **50** (`Propulsion.cs:789-807`)
- `ApplyDrag` forward only acts below 0.15 throttle

Hold the stick forward in strafe at any speed **between 30 and 50 m/s** and nothing acts on you: no drive, no drag, no bleed. The craft coasts there indefinitely.

The lateral cap works exactly as documented. It is only the forward axis that has the hole, and the `strafeTopSpeed` tooltip previously implied both bled off. Corrected.

## Drive and drag are not mutually exclusive

Drive fires at `|throttle| ≥ 0.001`. Forward drag fades out across `0 → 0.15`. **Both are active in the 0.001 to 0.15 band.**

The magnitude is small, but it contradicts the invariant stated in `CLAUDE.md` and the "opposing forces cause jitter" architecture principle. Worth knowing if low-throttle creep ever feels notchy.

---

## Rotation

| Axis | Rate | Notes |
|---|---|---|
| Yaw (ground) | `7/6` = **66.8°/s** | 42.9 m turn radius at top speed |
| Yaw (drifting) | 100.3°/s | 28.6 m radius |
| Yaw (airborne) | 33.4°/s | `airTurnMultiplier` 0.5 |
| Air pitch | 200.5°/s | full flip in 1.80 s |
| Air roll | **343.8°/s** | full roll in 1.05 s |
| Flip recovery | 429.7°/s | constant torque, not angle-proportional |

The craft **rotates 5.1x faster rolling in the air than yawing on the ground**. That may well be intentional for a game about air control, but it is a large asymmetry to have arrived at by accident, and the 42.9 m ground turn radius feeds directly into the arena guardrail about corridors accommodating turn radius at combat speed.

Nothing here saturates the 50 rad/s engine ceiling. Air roll runs at 6 rad/s, 12% of it.

---

## Weapons

> **Read emitter numbers from the scene, not the prefab.** Every weapon emitter in `Prototype_Scene` carries prefab **overrides** on emission rate, burst count and collision mask. The figures below are read live from the scene instances on `HoverCar_Prototype`, which is what actually plays. The prefab assets disagree in several places, and trusting them is how the first draft of this audit got a finding wrong.

### Knockback spans two orders of magnitude with nothing in between

`WeaponImpact.Apply` uses `ForceMode.Impulse`, so with mass 1000, `Δv = impulse / 1000`.

| Weapon | Impulse | Δv | Relative to 50 m/s top speed |
|---|---|---|---|
| Missile, direct hit | 100,000 | **100 m/s** | 2.0x top speed |
| Missile, splash | 50,000 | +50 m/s | 1.0x |
| Shotgun (30 pellets x 3000) | 90,000 | **90 m/s** | 1.8x |
| Chain Gun (200 x 20/s) | 4,000/s | 4.0 m/s per second | small |
| Machine Gun (200 x 16/s) | 3,200/s | **3.2 m/s per second** | negligible |

A single rocket imparts twice the craft's top speed. The sustained guns impart a few m/s per second of fire. There is no weapon occupying the middle of that range.

### Four of six weapons deal no damage

Emission is read live: Machine Gun runs **two** emitters at 8/sec each (16/sec combined), Chain Gun runs one at 20/sec, Shotgun fires a **30**-pellet burst. All muzzle velocities are 150 m/s; lifetimes of 0.5 s and 0.33 s give roughly 75 m and 50 m of effective range.

| Weapon | `damage` | Rate | DPS | Time to kill (100 HP) | `maxAmmo` |
|---|---|---|---|---|---|
| `WD_ChainGun` | 1 | 20/s | **20** | **5.0 s** | 0 (unlimited) |
| `WD_MachineGuns` | 0.5 | 16/s | **8** | **12.5 s** | 0 |
| `WD_Shotgun` | **0** | 30/burst | 0 | never | 0 |
| `WD_Missile` | **0** | 1.5/s | 0 | never | 0 |
| `WD_SoftHomingMissile` | **0** | 1.5/s | 0 | never | 0 |
| `WD_HardLockMissile` | **0** | 1.5/s | 0 | never | 0 |

The Chain Gun is the only weapon that kills in a reasonable time. The Machine Gun works but is slow enough that a fight decided by it would take 12.5 seconds of unbroken fire. Everything else does pure knockback and cannot reduce health at all.

**Every weapon has unlimited ammo.** The GDD specifies limited pickup ammo on everything except the Machine Gun.

Note also that the AI vehicle carries Machine Guns and a Shotgun but **no Chain Gun**, so the AI has no access to the one weapon that meaningfully damages.

### Soft Homing has not diverged

`WD_SoftHomingMissile` carries identical impact values to the heavy missile: `impactForce` 100000, `splashImpactForce` 50000, `destabilizeFraction` 0.15. `CLAUDE.md` records that the three were seeded identically during migration and are "now free to diverge, which the GDD wants". The GDD specifies low damage and force for the harassment tool. They have not diverged.

### Other weapon data notes

- **`WD_HardLockMissile`** runs `lockConeAngle` 45, the maximum the `[Range]` permits, at `lockRange` 115. Its own tooltip warns that wide cones "will happily grab the wrong target in a crowd", against a design contract of "slow to set up, but it will not miss".
- **`WD_MachineGuns.fireRate` is 0.01**, sitting exactly on the `[Min]` floor. Inert today because particle-mode fire rate only paces ammo drain and burst gaps, and ammo is unlimited. It becomes live the moment ammo is enabled.
- **`Missile.prefab` `splashRadius` is 10** against a tooltip recommending 5 to 8.

---

## Energy

| Ability | Cost | Notes |
|---|---|---|
| EMP | 70 | 70% of pool, matches the GDD's "70-80%" |
| Shield | 35 | |
| Jump | 25 | flat, regardless of charge |
| Dodge | 15 | |
| Boost | 20/sec | 5.0 s from full, exactly as documented |

Pool 100, regen 20/s after a 1 s lockout, so **empty to full is 6.0 s**. EMP plus Shield costs 105, so the two are mutually exclusive on one tank. That reads as intentional.

---

## Working as designed

Stated explicitly so you do not re-derive them mid-pass. All of these check out against the code:

- **Air control rates.** 14/4 = 200.5°/s pitch, 24/4 = 343.8°/s roll. The tooltips claim ~200 and ~344, and the full-rotation times they quote (1.8 s flip, just over 1 s roll) are correct.
- **Aim pitch damping.** ζ = `(8+8)/(2√150)` = 0.653, giving 6.7% overshoot. The tooltip's claim that damping 8 alone yields "~35% overshoot" computes to 33.8%. Accurate.
- **Boost budget.** 100 energy at 20/sec = 5.0 s, as stated.
- **Lateral cap bleed.** `strafeLateralCapStrength` 3 gives a 1/3 s time constant, ~95% decayed in 1 s, as stated.
- **Hard landing max speed.** 70 m/s corresponds to a 58 m drop, matching the tooltip's "roughly 60m".
- **Exact-cap clamp.** The `(cap − current) / Δt` term genuinely lands the crossing tick exactly on the cap with no overshoot ripple.
- **Lateral damp about a moving setpoint.** Excluding intended strafe velocity works as described; `strafeTopSpeed` really is the lateral ceiling.
- **Yaw torque.** Uses `ForceMode.Force` with a manual `× I_yy`, correctly reproducing inertia-independent Acceleration semantics.
- **Fixed timestep** is 0.01 (100 Hz) with interpolation, as documented.
- **Single attitude authority.** Foundation genuinely is the only thing applying pitch/roll torque. Propulsion hands it targets and never competes. The architecture holds.

---

## Needs polish, ranked by tuning impact

1. **Dodge Δv of 50 m/s.** Equal to full top speed. Largest readability risk.
2. **Hover ζ 0.474.** Underdamped; 1.33 s settle will read as vague.
3. **Weapon damage.** Four of six weapons at zero, and the AI carries none of the two that work.
4. **Knockback spread.** 100 m/s versus 3 m/s per second, nothing between.
5. **Strafe forward dead band**, 30 to 50 m/s.
6. **Slope compensation** effectively inert at 1.15.
7. **Unlimited ammo** on every weapon.
8. **Hard landing false positive** on stacked jumps (cosmetic today).
9. **Yaw-to-roll asymmetry** of 5.1x.
10. **Drive/drag overlap** in the 0.001 to 0.15 throttle band.

---

## Conflicts with the design pillars

| Pillar | Conflict |
|---|---|
| *Hit disruption over flat damage* | Over-satisfied. A rocket disrupts by 2x top speed and deals no damage at all, so the ordering is not "health second", it is "health never". Only the Chain Gun expresses both halves. |
| *Clarity at high speed* | A 50 m/s dodge in 0.1 s and a 100 m/s rocket punt are both hard to follow. |
| *No asymmetric loadouts* | Holds. All weapons are shared. |
| *Special weapons are exclamation points* | Untestable; no specials implemented yet. |
| *Pickup placement creates vulnerability* | Cannot function: unlimited ammo on every weapon removes the reason to contest pickups. |
| *Corridors accommodate turn radius* | 42.9 m radius at top speed is a hard constraint on arena geometry. |

---

## Fixed during this audit

**`VFX_MachineGuns.prefab` had a broken collision mask, but only for newly created instances.** Both emitters in the prefab asset carried `collidesWith` = `m_Bits: 55` (layers 0, 1, 2, 4, 5). `PlayerVehicle` is layer 6 and `AIVehicle` is layer 7, so neither vehicle layer was set: an instance dragged fresh from that prefab would fire bullets that pass through every vehicle. Both are now set to Everything, matching `VFX_Shotgun.prefab`, with `ParticleWeaponCollision.ConfigureCollisionMask` stripping the firer's own layer at Awake.

**This is *not* the cause of "bullets eaten on flat ground", and an earlier draft of this audit wrongly claimed it was.** Reading the live scene shows every emitter overrides the prefab mask with a correct per-vehicle value:

| Instance | Mask | Decoded |
|---|---|---|
| `HoverCar_Prototype` (all emitters) | 129 | Default + AIVehicle |
| `HoverCar_AI` (all emitters) | 65 | Default + PlayerVehicle |

Each vehicle's guns already target the opposing layer correctly, and the runtime own-layer strip is a no-op because the own bit was never set. **The machine gun does hit the AI in the current scene.** The prefab fix is authoring hygiene for future instances, nothing more.

So the "bullets eaten" cause remains **open**. Both candidate explanations are now ruled out: self-collision is impossible (own layer bit unset) and the mask is correct. What is left is that bullets legitimately collide with Default-layer terrain and die there, which is expected behaviour for a miss. The next step is unchanged from what `CLAUDE.md` already recommended: enable `drawDebug` on the emitter's `ParticleWeaponCollision` and read the contact colours. Sparse fire may also be a factor in the perception, since the two machine gun emitters together only produce 16 tracers per second.

**`FoundationTuning` class defaults would have destroyed any new vehicle profile.** `liftStrength` 50000 and `liftDamping` 5000 predate the move to `ForceMode.Acceleration`. Four points at that stiffness is 200,000 m/s² per metre of compression, so any profile created from the asset menu launched on frame one. Defaults now match the authored working values, and `unstickLiftForce` was raised from 25 to the authored 60, which its own tooltip's "try 40 to 80" already implied.

**Tooltip corrections** to `hardLandingMinSpeed` (stale jump-return constants), `minDriftSpeed` (claimed an alignment that does not hold), `dodgeForce` (no guidance on the highest-blast-radius knob), `strafeTopSpeed` (promised a forward bleed that does not exist), `slopeLiftMultiplier` (did not explain the cosine ramp), `liftStrength` and `liftDamping` (no explanation for why the numbers look small, no damping-ratio guidance), `flipRecoveryTorque` (slider six times wider than its own guidance), `splashRadius`, plus a jargon pass on three `AIHoverInput` fields and two `HoverCameraController` wiring fields.

No gameplay-feel value in `VTP_Default.asset` or any `WD_*.asset` was touched.

---

## Open question

`WeaponDefinition.destabilizeFraction`'s tooltip states that past about 0.3, off-centre hits "run into Unity's spin speed ceiling and start looking identical". `DynamicsManager.asset:38` sets `m_DefaultMaxAngularSpeed` to **50 rad/s**, not the historical default of 7 that this claim reads as though it was calibrated against.

Confirming or correcting it needs the vehicle's actual inertia tensor, which was not measured. Left in place rather than rewritten on a guess. Worth a live check with `WeaponDebugDraw`'s impulse-split visualisation during the pass.

---

## Stale claims in CLAUDE.md

- Cites a forward top speed of **60**; `VTP_Default` has **50**. The linked suggestion to raise `strafeTopSpeed` toward 45 was proportioned against 60, so the equivalent against 50 is about 37.
- Attributes "bullets eaten" to **self-collision**. Ruled out: the emitter's own layer bit is not set in any mask, so it cannot hit its own hull. Cause still open.
- States drive and drag are **mutually exclusive**; they overlap in the 0.001 to 0.15 throttle band.
- Describes the Shotgun as a **40-pellet** burst. The prefab asset says 40, but the scene instance overrides it to **30**. `WeaponDefinition.damage` and `destabilizeFraction` tooltips repeat the 40 figure and are worth reconciling.
