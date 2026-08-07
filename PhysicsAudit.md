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
>
> ### Update 2026-08-06: the weapons section was arithmetically right and physically wrong
>
> Every knockback figure in the Weapons section below was derived correctly from the source and the assets, and **not one of them was happening.** `RocketProjectile` detonated through `OnCollisionEnter`, which is speed-gated and never fired at the speed these projectiles actually fly. Missiles bounced off their target, drifted, and self-detonated on their lifetime timer six seconds later, splash-only. The direct-hit branch had never executed, so `impactForce` was inert and the derived "100 m/s direct hit" had never once been applied to anything.
>
> **This is a limitation of the method, not a slip in the arithmetic.** Deriving from source tells you what a code path computes. It cannot tell you whether that path ever runs. Any future audit written this way should pair each derivation with a single runtime confirmation that the code executes at all, which is cheap and would have caught this immediately.
>
> Now fixed (`ProjectileSweep`, `RocketProjectile` v1.6, `EmpProjectile` v1.1) and the figures re-derived **and measured**. See the Weapons section, which has been corrected in place.
>
> ### Update 2026-08-07: weapon tuning consolidated, and the emitter figures below were incomplete
>
> All weapon tuning now lives on the `WeaponDefinition` asset. Projectiles read it live; particle emitters have it written into them. **This changes how to audit weapons: read `Assets/Data/WD_*.asset` and nothing else.**
>
> That matters for this document because the emitter numbers in the Weapons section were gathered from the running scene, which was correct but not complete. The values were actually spread across four override layers (`VFX_*.prefab` → `HoverCar_PlayerController.prefab` → a prefab variant → the scene instance), and the player and AI craft had **silently drifted apart**: Machine Gun 8/sec on the player against 10/sec on the AI, muzzle velocity 150 against 100, Shotgun 150/100 against 100/80. None of that was intentional. Reconciled onto the player's values.
>
> Migration was verified by dumping every value before the restructure and diffing all six assets afterwards; all six matched. Behaviour was re-measured through the real fire path and is unchanged: Dumbfire 99.89 m/s, Soft Homing 54.89 m/s, flank-high 34.74 vs 7.61 rad/s over repeated trials.

## How to read this

Every number here is **derived from source code and serialized asset values**, not measured at runtime. The formulas were traced line by line through `HoverController_Foundation.cs` and `HoverController_Propulsion.cs`; the values come from `Assets/Data/VTP_Default.asset`, the `WD_*.asset` weapon definitions, the emitter prefabs, and `ProjectSettings/`.

Derivation is reliable for anything algebraic and less so for anything depending on the inertia tensor, which was not measured. Claims are marked where that matters.

> **The method's blind spot, learned the hard way.** Derivation tells you what a code path *computes*. It cannot tell you whether that path ever *runs*. The entire weapons knockback section below was derived correctly and described a branch that had never executed once in the build. Pair every derivation with one cheap runtime check that the code fires at all. A single `Debug.Log` in the branch would have done it.

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

**Corrected 2026-08-06.** Top speed is now **60 m/s**, not 50. Values below are current, and the missile figures are **measured in the running editor** rather than derived, through the real `FireAllMuzzles` path at the shipped projectile speed.

| Weapon | Impulse | Δv | Relative to 60 m/s top speed | Source |
|---|---|---|---|---|
| Rocket Launcher, direct hit | 100,000 | **100.0 m/s** | 1.67x top speed | measured (99.9 / 98.7 / 100.2 / 100.0 across four approaches) |
| Soft Homing, direct hit | 55,000 | **55.0 m/s** | 0.92x | measured (54.2 / 55.0) |
| Hard Lock, direct hit | 100,000 | 100 m/s | 1.67x | derived, untuned, still on the seeded value |
| Missile splash (Rocket / Hard Lock) | 50,000 | up to +50 m/s | 0.83x | derived. **Still unmeasured**, see below |
| Missile splash (Soft Homing) | 28,000 | up to +28 m/s | 0.47x | derived. Still unmeasured |
| Shotgun (30 pellets x 1800) | 54,000 | 54 m/s if all connect | 0.90x | derived. Particle path unmeasured |
| Chain Gun (200 x 20/s) | 4,000/s | 4.0 m/s per second | small | derived |
| Machine Gun (200 x 16/s) | 3,200/s | **3.2 m/s per second** | negligible | derived |

The original finding holds: there is still nothing occupying the middle of the range between a rocket and a sustained gun. Soft Homing at 0.92x top speed is now the closest thing to a middle entry, and it is the one that behaves the way the pillars ask for, since a hit at just under top speed changes your line without exceeding what you can drive out of.

**A direct hit does not also splash.** The direct-hit victim is added to `_splashedThisExplosion` and skipped by the splash loop, so a rocket delivers 100,000 *or* a falloff-scaled 50,000, never both. Splash still has not been measured, because reading it requires a detonation beside a craft rather than on it.

### Magnitude alone is the wrong frame: the flip threshold

Measured this session, and it changes how the table above should be read. A craft left resting past **80 degrees of tilt** is locked out of jump, steering and thrust for about **1.6 seconds**, and input cannot shorten it. So knockback is not a continuous scale, it is a scale with a cliff in it.

The chassis inertia is `(3293.6, 3756.1, 1091.0)`, so roll is roughly 3x easier to induce than pitch, and **where** a hit lands decides the outcome more than how hard it is. Identical flank hits 1.2m above the centre of mass, three reps each:

| Weapon | Peak roll rate | Peak tilt | Downed? |
|---|---|---|---|
| Rocket Launcher (100,000) | 34.7 rad/s | 167 deg | Yes, every rep |
| Soft Homing (55,000) | 7.6 rad/s | 21.6 deg | No, every rep |

A square hit from the same Rocket Launcher tilts the target only 8 degrees. Note the tilt outcome is **not** linear in force: initial roll rate scales with impulse, but past roughly 10 to 14 rad/s the craft commits and one-sided hover lift drives it the rest of the way over. That is the point of no return the `destabilizeFraction` tooltip describes, and it is why halving the force more than halves the tilt.

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

### Soft Homing has now diverged (resolved 2026-08-06)

At audit time all three missile definitions carried identical impact values. They no longer do:

| Definition | `impactForce` | `splashImpactForce` | `destabilizeFraction` |
|---|---|---|---|
| `WD_Missile` (Rocket Launcher) | 100000 | 50000 | 0.15 |
| `WD_SoftHomingMissile` | **55000** | **28000** | 0.15 |
| `WD_HardLockMissile` | 100000 | 50000 | 0.15 |

That puts the low force on the harassment tool, which is what the GDD asks for, and lands the two weapons cleanly on opposite sides of the flip threshold (see above). `destabilizeFraction` was deliberately left at 0.15 on all three: its proposed change to 0.3 was reasoning about angular-velocity saturation against `maxAngularVelocity` 50, which nothing here approaches, whereas the threshold that actually governs the outcome is the 80-degree tilt, which saturates far lower.

**`WD_HardLockMissile` remains untuned** and is now the outlier: the Rocket Launcher's full payload on the one weapon that cannot miss.

### Missile flight shape (new 2026-08-06, unaudited)

`RocketProjectile` v1.6 added per-prefab flight-shape dials that did not exist at audit time: `homingDelay`, `flareOffset`, `flareDuration`, `flareDirection`. No derivations exist for these yet. Two measured facts:

- **`turnRate` is the homing accuracy dial and 90 was too low to connect.** Against a target 15m off-axis at 30m range: 90 missed by 8.9m, 120 only clipped the blast edge, 160 was the first clean direct hit. `SoftHomingMissile` is now at 160; `HardLockMissile` is still at 120.
- **The flare costs no accuracy.** Flared and unflared shots landed the identical 55.0 m/s at the same 0.53s, because the missile homes on an offset aim point that decays onto the target rather than flying an open-loop heading.

**Unmeasured and worth a pass:** missile `speed` is 70 against a top speed of 60, so a target fleeing in a straight line is closed on at only 10 m/s. No steering value fixes that; it is a `speed` question.

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
4. **Knockback spread.** 100 m/s versus 3 m/s per second. Partly addressed: Soft Homing now sits at 55 m/s, giving the range a middle entry, but the Rocket Launcher is still at 1.67x top speed and Hard Lock is untuned at the same figure.
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
| *Hit disruption over flat damage* | Over-satisfied, and still is. A rocket disrupts by 1.67x top speed (measured) and deals no damage at all, so the ordering is not "health second", it is "health never". Only the Chain Gun expresses both halves. Worse than the original framing: at 1.67x top speed the victim is travelling faster than they can drive, so their momentum skill is irrelevant until they stop, which reads as removal rather than disruption. Soft Homing at 0.92x is the one weapon currently satisfying the pillar as written. |
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

## Open question, now answered (2026-08-06)

`WeaponDefinition.destabilizeFraction`'s tooltip states that past about 0.3, off-centre hits "run into Unity's spin speed ceiling and start looking identical". The audit flagged that `m_DefaultMaxAngularSpeed` is **50 rad/s**, not the historical 7 the claim reads as being calibrated against, and left the question open pending a measured inertia tensor.

**Both are now measured, and the tooltip is aimed at the wrong ceiling.** Inertia is `(3293.6, 3756.1, 1091.0)`. The hardest hit available in the build, a Rocket Launcher strike high on the flank at `destabilizeFraction` 0.15, peaks at **34.7 rad/s**, so the 50 rad/s spin ceiling is never reached and cannot be what flattens hits together.

The ceiling that actually matters is the **80-degree flip threshold**. Past roughly 10 to 14 rad/s of roll the craft commits and one-sided hover lift carries it the rest of the way over, so every hit above that point produces the same outcome (fully inverted, downed, ~1.6s lockout) regardless of how much harder it was hit. That is the real saturation, it sits far below 0.3, and it is a gameplay threshold rather than an engine limit.

**Consequence for tuning:** raising `destabilizeFraction` toward 0.3 buys nothing useful, and for any weapon meant to disrupt rather than punish it should probably go *down* from 0.15, not up. The tooltip's advice is worth rewriting against the flip threshold rather than the spin ceiling.

---

## Stale claims in CLAUDE.md

*Status as of 2026-08-06 marked per item.*

- ~~Cites a forward top speed of **60**; `VTP_Default` has **50**.~~ **Resolved by the tuning pass, in the other direction:** `VTP_Default` now genuinely has `topSpeed` 60 and `strafeTopSpeed` 40, so CLAUDE.md was correct and the audit's figure is the stale one. Every "relative to top speed" ratio in the original weapons section was therefore computed against 50 and reads ~20% high; the corrected table above uses 60.
- Attributes "bullets eaten" to **self-collision**. Ruled out: the emitter's own layer bit is not set in any mask, so it cannot hit its own hull. **Cause still open.**
- States drive and drag are **mutually exclusive**; they overlap in the 0.001 to 0.15 throttle band. **Still open.**
- ~~Describes the Shotgun as a **40-pellet** burst.~~ **Corrected in CLAUDE.md.** The scene instance is confirmed live at **30**. The `WeaponDefinition.destabilizeFraction` tooltip still repeats the 40 figure and is still worth reconciling.
