# TODO

**Everything unfinished, and nothing else.** Verification debt, blockers, known traps, pending
decisions, unimplemented features.

**When an item here is finished it LEAVES this file.** Its outcome goes to `CLAUDE.md` (what the
system now does and why), its measurements go to `TuningLog.md` (how the number was arrived at), and
its number is retired at the bottom of this file. Nothing closed is kept here, however good the
reasoning was. This file is read to find work; anything that is not work is cost.

## The documents

| Question | Document |
|---|---|
| **What is not done?** | **`TODO.md`** (this file) |
| What exists, how does it work, why is it built that way? | `CLAUDE.md` |
| Which unfinished things gate the next milestone, and in what order? | `ROADMAP.md` |
| How do I measure this without fooling myself? | `Measuring.md` |
| How was a shipped number arrived at, and what was rejected on the way? | `TuningLog.md` |
| What should this be? Why? | `GameDesignDocument.md` |
| Physics derivations and method, frozen at `4a34f21` | `PhysicsAudit.md` |

Two things are deliberately absent from this file and live in `CLAUDE.md`: **consciously accepted
limitations**, and **theories disproved by measurement**. Read that list before investigating any
symptom that sounds familiar; it exists specifically to stop a session being spent on a dead end.

## Rules that apply to everything below

- **Live values come from `Assets/Data/VTP_Default.asset` and `Assets/Data/WD_*.asset`, never from a
  doc.** Numbers quoted here are the ones an item was written against and may already be stale. Items
  quote a value only where the value IS the defect.
- **Pair every derivation with one cheap runtime check.** An entire audit section was once
  arithmetically perfect and described a branch that had never executed.
- **Never trust a zero you have not proven can be non-zero.**
- **Weight owner feel-reports above both the docs and the derived readouts, and check the instrument
  before checking the game.** The owner's instinct has beaten the record four times.

**Item numbers are permanent IDs and are never reused.** Gaps are retired or renumbered items, both
listed at the bottom. `ROADMAP.md` points at these numbers, so a number that moves tier gets a new
one rather than inheriting someone else's.

---

## How work is tiered

| Tier | What belongs in it |
|---|---|
| **0** | **Movement and camera. TOP PRIORITY, and specifically what gets worked next.** |
| **1** | Blocks the combat loop |
| **2** | Feel and feedback that is not movement, **or is movement blocked on art or VFX** |
| **3** | Traps and code quality. Not currently visible, will be misdiagnosed when they are |
| **4** | Verification debt and instrumentation. Nothing here blocks play or development |
| **5** | Design decisions with no engineering blocker. Needs an owner call, not code |
| **6** | Unimplemented features |

**The sorting rule, set by the owner 2026-08-16: anything movement or camera related belongs in
Tier 0, unless it is blocked by art or VFX, in which case it defers to Tier 2.** Weapons come after
movement, so weapon-subject work stays out of Tier 0 even when the symptom is a camera one.

| Tier | Open |
|---|---|
| **0** | 0.12 · 0.13 · 0.14 · 0.15 · 0.16 · 0.17 · 0.18 · 0.19 |
| **1** | 1.1 · 1.2 · 1.3 · 1.4 · 1.5 · 1.6 |
| **2** | 2.2 · 2.4 · 2.5 · 2.7 · 2.9 · 2.11 |
| **3** | 3.1 · 3.2 · 3.3 · 3.5 · 3.6 · 3.7 · 3.8 · 3.9 · 3.10 · 3.11 · 3.15 · 3.16 |
| **4** | 4.1 · 4.2 · 4.3 · 4.4 · 4.5 |
| **5** | 5.1 · 5.2 · 5.3 · 5.4 · 5.5 · 5.7 · 5.8 · 5.10 · 5.11 |
| **6** | 6.1 · 6.2 · 6.3 · 6.4 · 6.5 · 6.6 |

### Before the next playtest build

Cheap things that make a long session worth more, or that stop it producing an unreadable result:

- **4.4** press `M` once in the first thirty seconds and confirm the marker landed in `Player.log`
  before continuing. Eight sessions have already been lost to a marker key that silently never arrived.
- **0.15** three camera shake channels are unassigned, so **0.16 and 2.2 cannot be judged this session
  either** unless they are wired first. This is minutes of work.
- **0.17** boost is built and has never been judged. A feel question that needs a driver, not a
  measurement.
- **0.14** the strafe camera retune is the owner's own task and gates nothing else.
- **Watch:** the two-barrel-roll landing margin is 0.12s at the shipped fall gravity (`TuningLog.md`).
  If landing tricks feels inconsistent, that is the first number to suspect, not the trick scorer.

---

## Tier 0 — Movement and camera. TOP PRIORITY

**This is the working set.** Fifteen movement behaviours were judged good in the movement playtest and
are recorded in `CLAUDE.md` > Standing Decisions. **Read those before changing anything here:** several
items below are bounded by something already confirmed to feel right, and those confirmations are the
acceptance criteria.

### 0.12 The vehicle briefly leaves frame while the camera pitches up

**OPEN BUG**, and the only one in this tier. Classified a bug by the owner rather than a polish or
tuning item, because it violates a stated hard requirement: **vehicle fully onscreen at all times.**

Part of the craft clips offscreen mid-transition as the camera pitches up to look down on it. Both
endpoints are fine; the motion between them is not. Quantified on real terrain: hull margin ran +0.2086
of the viewport before the pitch, **-0.0404 during it (fully off screen)**, and +0.0806 settled at full
stick-up. Reproduced a second time at -0.1090. **The BOTTOM edge is the one breached.**

**Two theories are dead. Do not re-run them.**

1. **Orbit geometry.** The settled pose at full stick-up fits comfortably, so `pitchUpDistanceGain` was
   never the cause. It would have bought margin without touching the mechanism.
2. **Camera lag.** The rig really does trail its solved pose by up to 2.498m during a fast stick-up, so
   the guard genuinely is certifying a frame nobody is looking through. Feeding it the rig's REAL
   position was implemented, measured and backed out: it changed the worst margin by -0.0009 at 60 m/s,
   and at 45 m/s the guard performed slightly WORSE than being switched off entirely. Correcting the
   position while still assuming an ideal aim makes the model less self-consistent, not more.

**The cause is localised and the remaining work is a shape change in `Measure`, not tuning.** The clip
reproduces readily on real terrain and **does not reproduce at all on flat ground with the craft held
level, at any speed, with the guard on, off or modified.** So the missing variable is chassis attitude.
`Measure` treats the hull as a bounding SPHERE, which is attitude-independent, while the thing that
actually leaves frame is the rear-bottom corner of an oriented box. Two independent confirmations:

- **The sphere model is provably attitude-blind.** `EvaluateState` grades `Inverted` identically to
  `Resting` to three decimals: `guardScale` 1.000 both, `guardRemoved` 0.000 both, vertical margin 7.53
  both, horizontal 27.32 both. Turning the craft upside down changes nothing the framing check can see.
- **The real silhouette swings, and asymmetrically.** Sweeping the measured hull box through a full
  rotation against a fixed camera: **pitch overflows the BOTTOM edge by up to 24.3% of screen height**,
  clipping continuously from roughly 105 through 210 degrees, while **roll overflows by at most 7.7%**.
  Same camera, same craft, same box. The bottom edge matches the terrain measurements exactly.

*Caveat on those percentages: the sweep held the camera at the resting pose with the craft parked, so
the absolute figures are not the in-flight condition. The pitch-versus-roll comparison at a fixed
camera is the valid part, and it is a 3:1 difference the model cannot represent.*

**Constraint before starting:** `Measure` is deliberately shared by `EvaluateState` and the framing
guard so the inspector cannot promise a margin the runtime does not deliver. Any change lands on both
at once and the readout must be re-judged with it.

**Verification harness worth reusing:** the oriented-box viewport check written for this investigation.
Eight corners of the measured collider union through `Camera.WorldToViewportPoint`, reporting worst
margin and which edge broke. It is the only instrument here that measures the frame the player sees
rather than the frame the solver intended.

**Test discipline, and it cost three bad conclusions:** the first runs let the craft accelerate from
wherever the previous test left it, so no two were comparable and one apparent regression was pure
starting-state noise. **Pin position, rotation and velocity before any camera framing test.**

### 0.13 The camera has no answer while the craft is flipped and helpless

**NARROWED 2026-08-16 after the owner tested the original scope.** The craft flips, you have no agency
until it rights itself, and the camera simply settles to the side. Nothing useful happens in a window
where the player is doing nothing and wants to be looking at where they will end up.

**The trick half is CLOSED and must not be re-proposed.** The original item paired this with "right
stick moves the camera during heavy air, so a long trick sequence can set up the landing". The owner
built a session around it and rejected it: **with trick control already on one stick, camera control on
the other is too much to do at once.** That is a decision from play, not a guess. Air control keeps the
camera behaviour it has.

**What remains open is only the flipped case, and the answer is not necessarily control.** Three shapes,
and it should be a deliberate choice between them:

- **Hand the player the stick.** Cheapest, and the input really is free: Propulsion suppresses commanded
  yaw entirely while `IsDowned`, so the right stick has nothing to do and there is no mode conflict to
  resolve. Note this is a different situation from the rejected one, because the player is not also
  flying a trick.
- **A specific framing for the downed state.** Pull back, or swing to a readable angle, or frame the
  craft against where it needs to get back to. Costs no input and cannot be fumbled.
- **Something else entirely.** The requirement is that the window stops feeling dead, not that a
  particular mechanism is used.

**Do not conclude that camera panning during recovery is a bug.** A separate report of the camera
swinging out on a standing charge jump was root-caused and fixed (it was the travel-heading bound, see
`CLAUDE.md`). What is left from that report is a wart worth knowing: **air control makes the left stick
PITCH the moment the craft clears the clearance gate, and a charged jump always clears it**, so holding
any throttle through a charge jump reliably starts a tumble from standing. Propulsion documents that
hazard for hops and solves it with the height floor, which by design does not protect the charged case.
**That is currently the most reliable way to reach the state this item is about.**

**One latent issue, separate and not the cause of anything observed.** `UpdateHeadingProxy` scales by
`Time.deltaTime` with no clamp, so a 194ms GC frame permits ~35 degrees of heading movement in a single
frame and the converge term reaches 90% of chassis yaw at once. Both terms are designed to prevent
snaps at 3.7ms and both stop protecting at 194ms. A `Mathf.Min(Time.deltaTime, ~0.05f)` would make the
camera hitch-proof. **Not applied:** it was never demonstrated to cause a felt artifact, and the
editor's idle allocation means the hitch may not exist in a build at all.

### 0.14 Strafe camera view retune

**The owner's own task, not work to schedule.** The camera update changed the strafe view and it wants
retuning by hand.

One dependency: the reticle's resting height is set by `reticleProjectionDistance` (200 rests ~438px
above centre, 50 ~381px, 10 ~200px), and moving the strafe camera moves the crosshair with it.
**Retune the view first, then set that number to match.**

### 0.15 Three camera impulse channels are unwired

`HoverCameraImpulseRouter` warns on `Awake` that the **EMP**, **weapon recoil** and **denied jump**
sources are unassigned, so those three events currently produce **no camera shake at all**. Two of five
channels are hooked up. The router already reports it clearly; nobody had read the warnings.

**This invalidates the status of 0.16 and 2.2**, both of which describe themselves as wired but
unjudged. They cannot have been judged, because the shake half never fired. **Do this first in any
session that intends to judge either.**

### 0.16 A denied jump gives no usable feedback

**Wired, feel unjudged, and blocked on 0.15.** The player presses jump, nothing happens, and there is
no way to tell "out of energy" from "input didn't register" from "still in the post-land lockout". This
matters more than a normal missing cue because energy as tempo is a design pillar: a resource the
player is meant to manage as their primary tempo tool currently gives no feedback when it runs out.

Shipped mechanism: a camera impulse on its own channel, thrown down the chassis axis so a denial while
banked still reads as failing to rise. The air value is smaller deliberately, because the token is not
consumed and the failure is not final. Audio is out of scope, which is why this landed on the camera.

The owner's first pass was "just feels bad". Unlike the crash and EMP channels this was NOT the
screen-direction bug, so it is a genuine shape problem. Changed from a 0.12s recoil to a 0.18s bump:
0.12s is seven frames and reads as a frame hitch, and a bump returns to where it started, so it lands
as a lurch rather than a one-way shove. **Still unjudged.**

**If it still reads wrong, the next lever is direction rather than duration.** A denial is a failure to
LAUNCH, so a brief up-twitch that falls back may be more legible than a downward sag.

**Judge it by draining the meter and pressing jump, not with the test hotkey**, which fires without the
empty meter that gives it meaning. Note jump costs changed (tap is now much cheaper than a full charge),
so how often denial arrives has moved.

### 0.17 Boost is fully built and has never been judged

*Absorbs the old 0.9. Measured values are in `TuningLog.md` > Boost framing envelope.*

Owner note from a playtest: "the boost feels somewhat mid". The multiplier was never the problem; a 50%
increase is real, so this was always presentation. All four planned camera terms shipped: a sustained
pull-back, an FOV overshoot that settles rather than steps, extra lag at the moment of engaging, and a
return slower than the entry.

**Nothing about it has been judged by feel.** Two specific questions:

- **How boost reads at the MOMENT OF ENGAGING**, which is entirely camera.
- **Whether apparent longitudinal motion roughly tripling is the right amount.** That is the framing
  working as authored, not a defect, and the knobs are `zPullBack`, `zLagOnEngage` and `fovOvershoot`.

**Judge the DRIVE mode half now and leave the strafe half alone.** Strafe boost deliberately shows
nothing (2.11), so judging the two together contaminates the drive result.

**Two things to know before judging.** `boostBlendSeconds` is **0.35**, not the 0.15 several notes
claimed; that cut was set in the inspector and never persisted. The owner's decision was to leave it
and judge boost as-is, and **if boost still reads flat now that the speed pass has landed, 0.15 is the
first thing to try, ahead of any camera value.**

**Absolute sense of speed is a different question and is not fixable here.** Perceived speed is
structurally low on this chassis, so it reads about half as fast as its km/h suggests. See 6.2.

### 0.18 `flipRecoverySpeedThreshold` is an unconfirmed guess

Currently 2. The tooltip says so explicitly: 0.5 was too tight for this chassis and 2 is a guess.

**The failure mode is subtle:** a hull that comes to rest on a curved face and micro-rocks keeps
resetting the arming timer and never recovers, **which looks identical to recovery being broken.**

Worth settling in the same session as 0.13, since both are about the flipped state.

### 0.19 No camera preview state can reproduce a mid-flip pose

`CameraPreviewState.Inverted` sets the craft upside down but still travelling along its own nose, so it
cannot express the moment that mattered in the boost-gate bug: **nose swept past perpendicular to travel
while moving at speed.** Every one of the twelve states has velocity aligned with the chassis, which is
precisely the assumption that bug lived under.

That is part of why the defect survived so long: it was invisible in the inspector and only appeared
under a controlled playtest. **A state carrying a nose/travel disagreement would make this class of bug
judgeable without driving**, which is the argument for building it. Needs `FramingInputs.travelSpeed` to
differ from `forwardSpeed`, which `BuildPreviewInputs` currently pins equal on purpose.

Lowest priority in this tier: it is tooling that prevents a future bug rather than work you can feel.

---

## Tier 1 — Blocks the combat loop

The checklist item "Basic combat loop (primary fire + pickups)" is blocked by 1.1 through 1.4 together.
**None of them is independently sufficient.**

### 1.1 Death is terminal; respawn is a stub

`VehicleHealth.HandleDeath()` sets `IsAlive = false`, fires `OnDeath`, and calls
`gameObject.SetActive(false)`. Once disabled the vehicle's own MonoBehaviours stop, so it cannot revive
itself; something external must hold the reference and call in. **Nothing does.** There is no match
manager, round system or spawn controller.

`VehicleHealth.Respawn()` exists but has no callers, and is incomplete even if called. It sets health,
`IsAlive` and re-enables the object. It does NOT:

- reset position or velocity, so you respawn where you died at whatever velocity you had, possibly
  inside geometry
- fire `OnDamaged`, so the HUD health bar stays at 0 until the next damage event
- reset energy or ammo
- grant invulnerability (see 1.2)
- reset `AIHoverInput._state`, which `HandleDeath` latches to `Dead` and nothing clears. **A respawned
  AI would be inert forever**, its `Update` hitting `case AIState.Dead: ZeroAllOutputs()`.

`EnemyHealthBar` is the only thing already respawn-safe.

**Done looks like:** a decision on who owns respawn (match manager vs the vehicle itself), spawn point
selection, and a `Respawn` that resets the full vehicle state including AI FSM state and HUD.

**`PlaytestReset` does NOT advance this despite looking adjacent.** It is a held-button escape hatch that
restores pose and velocity and nothing else, on purpose, so runs after one stay readable. It does settle
one design question by demonstration: **the position-and-velocity half of a real respawn needs
`HoverCameraController.NotifyVehicleWarped` or the camera flies across the level**, and the same is true
for every future spawn point, checkpoint or teleport.

### 1.2 Post-respawn invulnerability does not exist

`VehicleHealth.SetInvulnerable(float)` has no callers. The machinery is entirely live: the flag, the
timer, the `Update` tick, the `TakeDamage` early-out and an `[INVULN]` gizmo label. **It simply can never
be switched on.** Both `CLAUDE.md` and `VehicleHealth`'s own header describe a post-respawn grace period
that has never run.

Separate clearly: **shield invulnerability works**, through a different path. Only the timed grace is dead.

Pairs with 1.1: respawning without it is spawn-camping by construction.

### 1.3 Four of six implemented weapons deal zero damage

`WD_Missile`, `WD_SoftHomingMissile`, `WD_HardLockMissile` and `WD_Shotgun` are all at `combat.damage: 0`.
Only `WD_ChainGun` (1) and `WD_MachineGuns` (0.5) do damage.

Not cosmetic: both delivery paths are live and both are being handed zero. The combat loop can therefore
neutralise a player completely but never finish them. **That is a coherent design if chosen deliberately
and a hole if not.** The inspector now warns on any definition sitting at 0.

**Watch out when picking numbers:** for ParticleSystem weapons `damage` counts PER PARTICLE, so the
Shotgun's 30-pellet burst applies it thirty times.

**Blocks:** 1.1 is pointless without this (nothing can die), 5.11 cannot be answered without it (no combat
pressure), and the self-damage suppression built into the rocket only matters once this is non-zero.

### 1.4 Every weapon has unlimited ammo

All six definitions have `maxAmmo: 0`, which `WeaponSlot.Initialize` turns into `currentAmmo = -1`. The
design specifies limited pickup ammo for everything except the Machine Gun.

Knock-on effects while this stands:

- `RefillAmmo` is a no-op (see 3.1)
- `OnAmmoDepletedForSlot` is **unreachable**, not merely unsubscribed: both invocations sit inside
  `if (slot.currentAmmo > 0)` and `currentAmmo` is permanently -1
- pickups have nothing to restore

### 1.5 `PickupManager` is unwritten

The only planned-and-entirely-absent gameplay module. Design calls for Ammo, Energy Cell and Health Repair
pickups, with **placement as the primary mechanism for forcing engagement** (powerful pickups in exposed
positions). Depends on 1.3 and 1.4 to have anything meaningful to give.

### 1.6 The AI cannot fire anything except slot 0

`AIHoverInput` never sets `FirePressed`, so it cannot fire any `SingleShot` or `Missile` weapon, including
the Shotgun it actually carries in slot 1. It also never sets `CycleWeaponNext/Prev`, so it never leaves
slot 0 regardless. **Two independent causes; fixing one alone changes nothing.**

Net effect: the AI's entire offense is the Machine Gun at 0.5 damage per pellet, 8 pellets/sec. It carries
no Chain Gun, which is the only weapon with a reasonable time to kill.

Also deferred inside `AIHoverInput`: strafe, drift, jump, shield, EMP, all hardcoded false or zero. The
missing EMP is what makes 3.15 unreachable in play.

---

## Tier 2 — Feel and feedback

Non-movement feel, plus movement feel that is **blocked on art or VFX** and therefore cannot be worked
in Tier 0.

### 2.2 EMP launch has no acknowledgement

**Wired, feel unjudged, and blocked on 0.15.** EMP is the most expensive ability in the game and empties
the meter you need in order to disengage. The projectile carries its own particle visual so the shot is
visible, but a commitment that large should confirm itself.

Shipped mechanism: its own impulse channel, a recoil thrown backward along the chassis so the launch reads
as a one-way shove.

The owner's first pass was "feels bad", and **most of that was the screen-space direction bug**
(`Measuring.md` trap 11): the craft's world heading was being applied as a screen direction, so the recoil
went sideways instead of backward. Now measured as a clean pull away from the craft, and the duration was
cut from a 0.35s wallow. **Needs re-judging after that fix.** Whether the weight is right for the game's
largest single energy commitment is still open; it currently sits at a little over half the landing punch.

*Filed here rather than Tier 0 because the subject is an ability, not movement. It becomes judgeable at
the same moment 0.16 does.*

### 2.4 Hard landings land with no dust

**BLOCKED ON ART.** `landingDustPrefab` and `landingDustHeavyPrefab` are both null on both vehicle
prefabs. They pointed into a VFX package that is no longer on disk, and `HoverVehicleVFX.HandleHardLanding`
null-checks and returns.

The whole hard-landing feature therefore currently delivers only the camera punch and the spring give. The
event, the severity split, the heavy/standard threshold and the ground raycast are all live and correct;
only the two prefab slots are empty.

### 2.5 Missile detonations have no explosion VFX

**BLOCKED ON ART.** `blast.explosionPrefab` is null on all three missile definitions, same removed package.
`RocketProjectile` null-checks before instantiating, so this fails silently rather than erroring.
Detonations currently produce no visual at all.

Note the dependency runs the opposite way from how it used to: the definition states `splashRadius` and the
VFX is authored to match it, not the reverse.

### 2.7 Automatic weapon fire rates are far too low

Owner note: both automatics "need to increase the firing speed a bunch to keep up with the reticle". Live
emission is **8/sec per Machine Gun barrel and 20/sec on the Chain Gun**.

Related: 3.6 records `WD_MachineGuns.combat.fireRate` at 0.01, which is a **separate value** from the
emitter rate. Both need to move together, and the definition is the single author since
`ApplyDefinitionToEmitter` pushes it into the emitter.

**Do not tune camera recoil before this item and 3.6 are fixed.** Recoil is wired, per weapon and opt-in
(`combat.recoilVelocity`, 0 on all six assets). It lands once per shot, so its right value is a function of
fire rate, and both automatics are currently misconfigured in opposite directions: the Chain Gun at 20/sec
would blur any kick into a tremor, and the Machine Guns at 0.01 would fire one kick every 100 seconds. Any
number picked now gets invalidated when the rates move. The single-shot weapons are tunable today and are
the sensible place to start.

#### Deferred: per-weapon recoil CHARACTER, not just strength

**Deliberately not built**, because the weapons are placeholder and shaping camera feel around them now is
tuning against a moving target. Recorded so the reasoning is not re-derived.

Only strength is per weapon today. Shape and duration live on a single shared impulse source, so every gun
that opts in shares one character. The owner found that insufficient immediately: **the shotgun wants a
short snap and a missile wants a longer envelope with a wind-up dip**, which is a difference in kind
rather than in size.

**Do not infer the split from `projectileMode`.** It was considered because it happens to separate the
current six correctly, and rejected: mode describes how a weapon spawns damage, not how it should feel. A
railgun would be Instantiated and want a sharp crack; a plasma stream would be ParticleSystem and want a
sustained push. Worse, the failure would be silent, and the only way to author around it would be to change
the projectile mode, which changes gameplay.

**And do NOT solve it by writing shape and duration onto the shared source before each shot.** That is the
obvious design and it is unsafe: an impulse reads its shape and duration off the live source component for
its whole life, so a still-playing impulse adopts whatever the next shot wrote. Full mechanism is
`Measuring.md` trap 14. The consequence is simply that **different characters need different sources.**

**The shape it should take when it is worth building:** a `recoilCharacter` enum next to `recoilVelocity`,
named for FEEL rather than for camera objects (`Snap`, `Swell`), with one source per character on the
camera and the router picking by name. The weapon says what kind of event it is; the camera owns how that
is expressed, so the weapon definition never learns a camera concept. Start with exactly the two that were
empirically found. `recoilVelocity` 0 stays the off switch, so the enum only ever picks character. Cost per
character is one GameObject, one serialized field and one enum value, so do not create them speculatively.

### 2.9 Taking a weapon hit produces no camera feedback at all

*The rest of this item is closed: crash shake, landing punch and bad-angle tumble were all judged good, and
the queued `levelingTorqueStrength` change is withdrawn. **Do not make that change without a fresh
complaint** -- it was the only queued reason to modify `HoverController_Foundation`.*

Being shot is currently as silent as the denied jump was. **The crash path does not and cannot cover it:**
crash shake hangs off `OnCollisionEnter`, which needs a physical contact, and weapon damage does not arrive
that way. Rockets detonate from `ProjectileSweep` before touching, and particle weapons never had a
collision to begin with.

The hook exists and has no camera subscriber: `VehicleHealth.OnDamaged(currentHealth, maxHealth)`. Note the
signature carries HP rather than damage dealt, so the router would need to difference it to get a
magnitude, or the event needs a third parameter.

**Undecided on purpose, and worth deciding before wiring:** a shake on every hit is a real design choice,
since it costs aim stability at exactly the moment the player most needs it, and some games deliberately
refuse it.

**Also still untested: vehicle-on-vehicle collision.** The AI craft is on its own layer with its own mesh
colliders and should route identically, but nobody has driven it. Easy path: park the AI and drive into it.

### 2.11 Boost presentation FX: vignette, speed lines, duration-based rumble

**BLOCKED ON VFX, deferred to PRE-ALPHA 2 by the owner**, to be picked up with the general FX quality
pass. Their reasoning: the whole FX layer is currently basic, and building one polished effect against a
bed of rough ones is the wrong order. **This is the only reason it is not in Tier 0.**

**The deferral does not withdraw the diagnosis, and the sequencing note still stands:** boost strength in
BOTH modes is being judged with half its presentation missing, so **do not retune `boostAccelMultiplier`
or `boostSpeedMultiplier` on feel until it lands.**

**This is the fix for a live complaint, not a polish item.** Owner: "my boost feels like it does next to
nothing while strafing." **The physics is not the problem and must not be touched:**

| | Speed | Time to 95% | FOV | Camera distance |
|---|---|---|---|---|
| Strafe | 53.0 -> 66.4 (**+25%**) | 0.33s | **55 -> 55** | **6.95 -> 6.95** |
| Drive | 80.1 -> ~99 (**+24%**) | 0.34s | 65 -> 69 | 9.84 -> 10.25 |

Identical relative gain, identical time to arrive. **Strafe boost delivers everything drive boost does
and shows the player nothing at all**, because the lens was removed from strafe on the owner's
no-modifiers rule. Boost's felt impact is mostly presentation, which is exactly why the same physics
reads as "next to nothing" in one mode and fine in the other.

**Do NOT resolve this by restoring the FOV kick to strafe.** It is a genuine aim modifier here: the
reticle is a projected world point rather than a centred crosshair, resting ~438px above centre, and a
projected point moves radially with the lens, so the shipped +4 degree kick drags it **35 pixels**
before the overshoot is counted. The owner's rule is correct and better founded than the comment that
originally justified it.

**So the cue must touch neither the rig nor the lens**, which is precisely what the three requested
effects do. Vignette and speed lines are screen-space overlays: no camera movement, no zoom change, no
reticle displacement, and they read as speed more strongly than an FOV kick does. That makes them the
only available answer rather than merely a nice one.

**The rumble is the one to be careful with.** It is the first camera effect requested that builds with
DURATION HELD rather than firing on an event, and the boost envelope already tracks exactly that. But
rumble moves the camera, so in strafe it would move the reticle for the same reason FOV does. Either gate
it to drive mode, or make it a screen-space shake of the rendered image rather than a camera move.

---

## Tier 3 — Known traps and code quality

**These are not currently causing visible problems.** Each is recorded because it will be misdiagnosed as
something else when it does.

### 3.1 `RefillAmmo` secretly resets cooldown and wind-up

Implemented as `slot.Initialize()`, which also resets `cooldownRemaining` and `windUpProgress` and rebuilds
`baseEmitterRates`. So picking up ammo will **cancel your cooldown and dump your Chain Gun spin-up**: a
three-second wind-up erased by driving over a pickup. Nothing in the method name suggests it.

Harmless today because there are no pickups and no ammo; a confusing "minigun bug" the day both exist.
**Fix:** split ammo restoration out of `Initialize`.

### 3.2 `ParticleWeaponCollision` walks the hierarchy per collision event

Both `GetComponentInParent<IDamageable>()` and `GetComponentInParent<Rigidbody>()` are called **inside** the
per-event loop, although `other` is invariant across the whole loop. A 30-pellet shotgun burst landing on
one target does 60 hierarchy walks instead of 2.

The file's own note defers this to "pooling debt", but the hoist is independent of pooling and is a two-line
change. Do the hoist now; keep the pooling note for later.

### 3.3 The AI steers away from its own target

`AIHoverInput.obstacleMask` defaults to `~0`, so the 5-ray avoidance fan treats the player vehicle as a
wall. At close range avoidance urgency fully overrides waypoint steering, meaning **the AI actively backs
away from the thing it is trying to shoot.**

Also casts from `transform.position + Vector3.up * 0.5f`, which is inside its own hull.

**Fix:** restrict `obstacleMask` to terrain layers.

### 3.5 `RocketProjectile.Explode` allocates

Uses the allocating `Physics.OverlapSphere` while the rest of the project is scrupulously non-alloc
(`RaycastNonAlloc` in the hover springs, `OverlapSphereNonAlloc` in the scans, static reusable buffers in
`ProjectileSweep`). **Detonations are bursty, so this is garbage generated at exactly the worst moment.**

### 3.6 `WD_MachineGuns.combat.fireRate` is 0.01

That is the `[Min]` attribute floor, giving a **100-second cooldown**. Almost certainly a slider dragged to
the bottom rather than an intended value. Compare `WD_ChainGun` at 20.

Currently invisible: for a ParticleSystem Automatic weapon the emitters play regardless of `IsReady`, and
`maxAmmo` is 0 so no ammo decrements. The only observable effect is that `OnWeaponFired` fires once per 100
seconds. **It breaks the moment the Machine Gun gets ammo (1.4).** Move it with 2.7.

### 3.7 `SetActiveSlot` does not skip empty slots

Unlike `TickCycleWeapon`, which walks past slots with a null definition, `SetActiveSlot(int)` accepts any
in-range index. Pointing it at an empty slot **leaves the vehicle silently weaponless**, because `Update`
early-returns on `definition == null`.

Both paths now share `SwitchToSlot`, so the empty-slot skip only needs writing once and belongs there.

### 3.8 `SetRecoveryEnabled` has a sharp edge for its first caller

`HoverController_Foundation.SetRecoveryEnabled(bool)` has no callers today and `recoveryEnabled` is
permanently true. EMP does not use it, since Foundation's `FixedUpdate` early-returns wholesale on an EMP
freeze and skips recovery anyway.

Before anything wires it up: `IsDowned = recoveryEnabled && (...)`. So **`SetRecoveryEnabled(false)` on a
downed craft immediately clears `IsDowned` and hands full control back to a vehicle lying on its back**,
which is the exact bypass that was closed in Propulsion v1.3. That is arguably correct (disabling recovery
should disable the lockout recovery exists to enforce) but it is a trap worth knowing.

Minor: it also does not clear `unstickForceTimer`, so an in-flight unstick push finishes.

### 3.9 Scene wiring noise

Cosmetic, no runtime effect, but misleading when reading the inspector:

- Weapon slots 2/3/4 (the three missiles, all `Instantiated` mode) each have a particle emitter assigned,
  which that mode ignores. **No longer purely cosmetic:** switching away from a slot now stops its emitters
  and rewrites their emission rate, so slot 2 will reach into the **Shotgun's** emitter, which is what it
  actually points at, on every switch. Harmless today only because the Shotgun is burst-only, but it is now
  a live coupling rather than dead data. Slots 3 and 4 additionally hold a null element in the list.
- Slot 5 (Chain Gun, `ParticleSystem` mode) has a muzzle point assigned, which that mode ignores.
- `blast.damageLayers` is `Everything` on `WD_ChainGun`, `WD_MachineGuns` and `WD_Shotgun`, none of which
  read the blast section.

### 3.10 Smooth ground normals depend on Read/Write Enabled, and fail SILENTLY without it

`ResolveSurfaceNormal` interpolates a mesh's vertex normals so the hover system reads the smooth surface the
player sees rather than the flat triangle underneath. It needs `Mesh.isReadable`, which means **Read/Write
Enabled ticked in the model's import settings.** All 24 collision meshes in `Prototype_Scene` currently have
it. Nothing enforces that.

**The failure mode is the problem, not the dependency.** A terrain asset imported without it does not error,
warn, or behave oddly in any visible way. It silently falls back to flat triangle normals **for that object
only**, and the craft goes back to being nudged a few degrees several times a second while crossing it. It
will read as "this one part of the arena feels bumpy", which is not a description that points anywhere near
an import setting.

**When this will bite:** arena blockout (6.2), when new geometry arrives from outside the RVP package. It is
filed here rather than Tier 0 because nothing is wrong today and there is no work to do until then.

Options if it becomes a nuisance rather than a one-off: log a one-time warning naming the offending mesh the
first time a non-readable one is hit, or add an editor validation pass over the scene's `MeshCollider`s.
**Deliberately not built yet**, because a warning that fires legitimately on primitive colliders would be
the fifth instrument in this project to cry wolf.

**If anyone is tempted to untick it to save memory:** Read/Write Enabled keeps a CPU-side copy of the mesh,
so it doubles that mesh's memory. This feature is why it must stay on.

### 3.11 The smooth-normal mesh cache is never cleared

`_meshNormalCache` is a `static Dictionary<Mesh, MeshNormalData>` filled lazily per mesh driven over. **The
cache itself is required, not an optimisation:** `Mesh.triangles` and `Mesh.normals` allocate a fresh array
on every access, so reading them per hover ray would generate garbage four times per `FixedUpdate` forever.

What is missing is eviction. It is never emptied and it holds strong references to `Mesh` objects.

- **In a build this is harmless.** The cache lives as long as the level does, roughly 80KB per large terrain
  mesh, and only for meshes actually driven on.
- **In the EDITOR it accumulates**, because statics survive exiting play mode, and the held references also
  stop those meshes being unloaded. Negligible with one scene; it grows once there are several.

**Fix when convenient:** clear it on play-mode state change in the editor, and consider
`RuntimeInitializeOnLoadMethod` for domain-reload-disabled setups. Left out deliberately for now, since
adding lifecycle code that cannot be exercised in a single-scene project is its own risk.

### 3.15 An EMP freeze silently eats a landed trick

`Energy.Grant` refuses to pay while EMP-frozen, matching `TryConsume` so a freeze cannot be worked around by
banking a trick during it. That is defensible and probably right, since EMP denies tempo. **It was never
DECIDED, and that decision is what remains open.**

The readout half is fixed: both the HUD and the gizmo used to report any shortfall as "pool full", because a
saturated pool was the only cause either knew about; a frozen pool produced the identical shortfall and got
the identical, wrong label. The two causes are now named separately.

**What is left is the design question, not the display.** A landed trick during a freeze still pays nothing,
and whether that is right has never been argued: the player did the work and the landing was clean.

**Not reachable in play today**, because the AI cannot fire an EMP (1.6), which is why this is not in Tier 0
despite being a trick item. **Done looks like:** a decision on whether a freeze should forfeit the trick,
cancel it, or merely defer the payout until the freeze lifts.

### 3.16 The trick system lives in scene overrides, not on the prefabs

`HoverController_Tricks` on the player craft, and `/Canvas/TrickTracker_TMP` plus its `trickText` wiring, are
all **instance overrides in `Prototype_Scene`** rather than changes to `HoverCar_Prototype.prefab` and the
Canvas prefab. It works, and it works in a build, because the build ships the scene.

**Why it was left this way:** applying prefab overrides is all-or-nothing per instance, and neither instance
has been audited for what else is sitting unapplied. Pushing blind would silently commit unrelated pending
changes into the prefabs.

**When it bites:** the moment a second scene exists, or a second player craft is instanced, or the prefab is
used as the base for the five-vehicle roster (5.7). **Done looks like:** audit the override list on both
instances, then apply deliberately.

Note the AI craft has no `HoverController_Tricks` at all, which is correct rather than an oversight:
`AIHoverInput` hardcodes drift to false so it can never arm air control and would bank nothing.

---

## Tier 4 — Verification debt and instrumentation

**Nothing in this tier blocks play, the playtest, or continued development.** It is what is believed but
not proven, plus instruments that are missing or lying. Work it when a question actually needs answering.

### 4.1 There are no tests, at all

No asmdef, no test assembly, no EditMode or PlayMode tests anywhere in `Assets/` outside the RVP
third-party folder. Every regression so far has been caught by hand-driven measurement, which is why
several bugs survived multiple sessions.

Not urgent, but the projectile helpers (`ProjectileSweep`, `WeaponImpact`, `TargetingScan`, `HoverMath`)
are pure static functions and are the cheapest possible place to start.

### 4.2 No instrument records the live caps or which force is acting

`FrameSpikeWatch` and `MotionTrace` between them cover frame death, allocation and whether the drawn
frame matched the physics. **Neither records the live forward and lateral speed CAPS, or which force
branch acted**, which is exactly what would let the boost ramp and the drift bleed be graphed against
each other.

Propulsion already captures both for its gizmo (`_dbgFwdCap`, `_dbgLatCap`, `_dbgDrive`, `_dbgDrag`,
`_dbgBleed`), so **closing this is mostly a matter of exposing those rather than computing anything
new.**

The jump arc no longer needs this; it is measured well enough by the impulse table in `TuningLog.md`.

### 4.3 Particle weapon delivery is unverified end to end

Missile, EMP and HardLock detonation and knockback are verified in play mode. **The Shotgun, Machine Gun
and Chain Gun particle paths are not**, and 1.3 means damage is zero on most of them anyway, so only
knockback could be exercised today.

Related and still unmeasured: **particle collision at scale.** Every multi-vehicle performance figure
was taken with AI driving and no combat, because the AI cannot fire (1.6).

### 4.4 Two things the first build did not test

The GC question itself is closed: the periodic hitch was the editor's own allocation and does not
survive a build. What that three-minute run did not cover:

**Allocation under sustained weapon fire and heavy projectile traffic.** Never measured. `AllocationBisect`
requires a parked craft and every run since has been driving, so these paths have no allocation figure at
all. **The build makes this cheap:** idle allocation there is 0.01-0.08 MB/s, so anything the weapons
allocate will stand out against a floor of essentially zero, which was never true in the editor's
4.47 MB/s. Fire continuously through a long run and read `alloc_mb_s` in the spike CSV.

**The marker key has never been confirmed in a build.** The first build run recorded zero markers because
nobody pressed `M`: not evidence of failure, but not evidence of success either, and this project has
already lost eight sessions to a marker key that silently never arrived. **Press `M` once in the first
thirty seconds of the next long run and confirm `[MotionTrace] MARKER 1` in `Player.log` before
continuing.** If it is missing, stop and fix it rather than spending the rest of the session producing an
unfalsifiable trace. Gamepad input is already proven in a build.

### 4.5 `FrameSpikeWatch`'s throttling verdict compares two samples of a noisy signal

The verdict compares the **first** benchmark sample against the **last** one. That is only valid if a
single sample is a good estimate of the machine's speed, and it is not: across 91 heartbeats the
benchmark has **sd 0.093ms on a mean of 0.532ms, 17.4%**, ranging 0.366 to 0.655. Two samples drawn from
that spread can differ by 79% with nothing happening at all.

**It has fired twice and been wrong both times, and cost time both times.**

- First firing printed `THROTTLING LIKELY ... +37.1%` for a session that is flat when binned into
  30-second windows (first half against second half is +1.7%). It was believed and written into another
  item before anyone checked it.
- Second firing printed `-16.8%` for a session whose binned benchmark is flat to within 8%, during a
  genuine performance investigation. It pointed at the CPU, which was one of the few things measurably
  fine.

**This is trap 15 wearing different clothes, and worse:** `FrameSpikeWatch` exists BECAUSE a fixed
threshold against a noisy quantity cries wolf, and its baseline is a smoothed running average for exactly
that reason. The benchmark got a naive endpoint comparison bolted onto the same instrument built to avoid
one.

**Done looks like:** compare the mean of the first N beats against the mean of the last N (N around 10,
roughly 20 seconds), and stay silent unless the difference clears the measured noise, something like 2
standard deviations rather than an arbitrary percentage. Printing the binned series instead of a verdict
would also be honest, and is what actually answered the question both times.

**Until then, ignore the verdict line and bin `bench_ms` from the CSV by hand.** A wrong confident answer
is worse than no answer, which is the premise of this whole file.

---

## Tier 5 — Design decisions with no engineering blocker

**These need an owner call, not code.** Listed with what is already measured so the decision is cheap.

### 5.1 Hard Lock is not designed through

**The mechanism works and is verified.** What follows is about what the weapon SHOULD be, not whether it
functions.

Stated intent: a volley of **small, low-damage, low-impact missiles**, lockable onto a single target or
distributed across several. Currently implemented as the degenerate case, one lock and one missile, and
tuned as though it were a Rocket Launcher.

Current values are the wrong shape for that intent: `impactForce` and `splashImpactForce` are identical to
the Dumbfire, on a weapon whose identity is that it cannot miss. `turnRate` is 120, below the 160 measured
as the threshold for landing a clean hit. `lockConeAngle` is 45 degrees at `lockRange` 115: wide for a
single lock, but plausibly right for sweeping across several targets in a cascade.

The lock now commits to one target and holds it, which is the primitive a multi-target version needs.
Extending it means promoting `LockTarget` to a list and looping `Scanning` back on itself with
already-committed targets excluded from the scan; the validity rule and the release-to-fire trigger carry
over unchanged.

**Do not tune this weapon until the design is settled.** Any number picked now is for a different weapon.

### 5.2 The knock-around pass is only partly applied

The missile force cut sits on Soft Homing, not Dumbfire, by owner decision. `WD_Missile` (Dumbfire) and
`WD_HardLockMissile` both keep the original values.

That is GDD-aligned: soft homing is specced as low damage and force, the Rocket Launcher as the primary
momentum-disruption tool. But it does mean **the Dumbfire keeps the over-scaled value the audit flagged**,
which measured a delta-v larger than the craft's own top speed, and where a flank hit fully inverted a
stationary craft and threw it about 80m.

**The open question is only whether the Dumbfire keeps that value.** The split itself is verified and working
as designed.

### 5.3 `destabilizeFraction` has never been tested against a landing hit

Held at 0.15 across all three missiles. It was set **before detonation worked**, so its rationale was never
verifiable: every apparent hit at the time was really the lifetime fuse firing splash-only. Detonation now
works, so this is measurable for the first time.

**New coupling:** it also governs self-inflicted spin from a rocket jump, and **self-flips become reachable
past roughly 0.36.** Recheck that before raising this value.

### 5.4 `selfImpactScale` default is a guess

Set at 0.5 on all six weapons, sized against existing mobility so a rocket jump lands between a tap jump and
a charged one. **Unmeasured in play.**

Note the self-boost **scales with launch speed**, because the rocket does not inherit vehicle velocity, so at
speed you close on your own blast. That reads as skill (shooting a wall you are charging kicks harder) but it
means the mechanic is strongest exactly when you are already fastest. If you would rather it were flat,
measure falloff from the launch position instead of the detonation point.

### 5.5 Homing cannot catch a runner, and the speed pass turned that into a defect

**Read from the assets 2026-08-16: all three missiles are at `speed: 70`, against a `topSpeed` of 80 and a
boosted 100.**

So a craft running flat out in a straight line is **not being closed on at all; it is pulling away**, by 10
m/s unboosted and 30 boosted. Previously missiles at 70 against a top speed of 60 closed at 10 m/s, which was
slow but finite, and that is the situation the original framing was written for. **No `turnRate` can fix a
negative closing speed.**

The original conclusion still holds as intent (homing weapons punish players who are turning, fighting or
cornered rather than running) but it is no longer a choice, it is the only thing the numbers permit.

**Decide which:** raise missile `speed` above the new boosted ceiling so homing can threaten a runner, or
accept the escape and keep homing as a close-quarters harassment tool. Either way **the value to change is
`speed`, not `turnRate`.** Nothing flagged this when top speed moved, which is the same
neighbouring-system-invalidated-by-a-shared-quantity shape as the air jump.

### 5.7 Vehicle roster is one profile

`VTP_Default` is the only `VehicleTuningProfile` in the project. The GDD proposes a five-vehicle framework on
two independent axes (momentum character, stability vs agility): Bruiser, Interceptor, Chaos Vehicle,
Precision Drone, Transition Specialist. None of the other four exists.

The architecture supports it already, one asset per archetype with scene refs staying on the MonoBehaviours,
so **this is authoring work, not engineering.**

### 5.8 Vehicle scale: deferred, with two gates

The craft is a ~1.6x sedan and disproportionately wide, which is what makes it read stubby rather than simply
large. Measured dimensions and the full reasoning are in `CLAUDE.md` > Vehicle Scale.

**Deferred deliberately, not forgotten.** The handling is good and there is nothing built for the craft to
look wrong against. Two points where the decision should be re-opened, because both produce work that
INHERITS the scale and is expensive to redo:

1. **Before arena blockout.** Geometry authored at one scale and rebuilt at another is far more costly than
   rescaling a single vehicle. Note the alternative is equally valid and cheaper: **build the arena TO the
   craft** (corridors ~20m, entrances ~10m) and nothing needs retuning at all.
2. **Before the five-vehicle roster** (5.7). Authoring five handling profiles against an unsettled scale means
   tuning five vehicles twice.

If it is ever revisited, the cheapest single improvement is **narrowing the hull.** Width is the least coupled
dimension to the tuning, since the attitude torques ignore the inertia tensor.

### 5.10 Wall jump / wall riding, speculative

Owner idea: count as grounded when deliberately oriented against a wall with sensors in range.

**Partly pre-built already.** The air jump now fires along the craft's LOCAL up, so tilting toward a wall
propels you away from it, and the measured 90-degree case is a clean pure-horizontal shove
(`TuningLog.md` > Air jump). That is most of a wall jump reached from a different direction and for a
different reason. **Re-read this item against that table; the remaining gap may be small enough that this
stops being speculative.**

**Test before building.** It may already partly work: hover points cast along `-point.up`, which rotates with
the craft, so orienting against a wall does put rays on it and the springs will push off. `IsDowned` is the
thing that would block it, and only on CONTACT past the flip threshold. A craft hovering the wall without
touching it keeps full control by design.

*Filed here rather than Tier 0 because it is speculative new movement, not polish on what exists. Promote it
if it becomes a decision to build rather than a question to test.*

### 5.11 Boosted strafe reaches a large fraction of drive top speed

**DEFERRED to PRE-ALPHA 2, and the deferral is principled rather than a punt:** the open question is whether
players CAMP in strafe under combat pressure, and there is no combat to apply pressure until weapons deal
damage (1.3). It cannot be answered before then, which is why it is not in Tier 0.

**Improved for free by the speed pass, from 100% to 83%.** Boosted omnidirectional strafe now sits at 66.3 m/s
against a drive top speed of 80, verified from the live caps. It is no longer true that strafe matches
drive-mode top speed, which was the sharpest form of the complaint.

The owner's stated goal is that drive mode is for fast forward and backward travel and strafe is for aiming at
a reduced, consistent omnidirectional speed, and specifically that players should not "play the whole game in
strafe mode".

**One mechanic that materially limits the camping case and was not accounted for when this was written:
continuous boost requires forward-dominant throttle**, so pressing boost with pure lateral stick fires a DODGE
instead. **A player cannot boost straight sideways at all.**

**The lever if camping does happen:** stop boost scaling the strafe ceiling. Note that alone would recreate the
paying-for-nothing bug already fixed for reverse boost (energy drained for acceleration into an unchanged
cap), so it would need boost in strafe to become dodge-only, which is a larger behaviour change than it first
appears.

---

## Tier 6 — Unimplemented features

### 6.1 Seven of thirteen weapons

Implemented: Machine Gun, Minigun (Chain Gun), Shotgun, Rocket Launcher, Soft Homing, Hard Lock.

Planned, no asset and no code: Sniper / Lightning Bolt, Laser Cannon, Gravity Well / Repulsor, Bouncing Disc
Blade, Floating Proximity Mine, Directional Remote Mine, and the per-vehicle Special.

Note `WeaponType.Mine` and `ProjectileMode` already exist and `TickMine` is implemented (fires at the muzzle
with no launch velocity), so **the two mine weapons are closer than the rest.**

### 6.2 Arena

The GDD has a full arena philosophy (track with rooms, three vertical layers, parallel routes at every major
node, corridors as momentum builders, falling as a tactic) and prototype scope calls for a small test arena
with ramps and obstacles. `Prototype_Scene.unity` is the only project scene.

**Measured constraint to design against: tunnel and doorway entrances want about 10m of clearance.** Interiors
are safe because the ceiling duck squats the craft; entrances are not.

**The arena is deliberately SPARSE.** Owner: "I hadn't really imagined an arena to be dense with props, maybe
just some environmental hazards and exploding barrels or something, but by no means full of objects on the
ground."

**So props are NOT coming to fix speed readability, and this item is not the blocker for it.** Ride height has
since been judged good independently, so only the speed half remains open, and what is left to read speed
against is ground texture density, road markings, the height and spacing of the large structures, and
screen-space effects such as 2.11's speed lines.

Two constraints any blockout must respect: the owner wants **platforms you can bump from underneath, not
spaces to duck under**, so the ceiling duck is not a design dependency; and the existing figure-eight loop is
the reference for drift curvature.

Note the environment was doubled to match the 2x vehicle mesh, which halved how fast the world goes past you
at a given speed.

### 6.3 No match flow

No menu, no match start or end, no scoring, no round structure. Combined with 1.1, there is currently no way
for a match to conclude.

### 6.4 Campaign layer

Per-vehicle pilot stories with intro and outro cinematics, Twisted Metal style. Explicitly a full product
layer rather than a test harness. Nothing exists.

### 6.5 Multiplayer

Explicitly a future phase. The solo game is specced to stand alone.

### 6.6 GDD diagrams

Section 15 lists two planned diagrams, Control Flow and Interaction Map, to be exported to `/Docs/`. Neither
exists, and neither does the folder.

---

## Renumbered 2026-08-16

**Tier M was dissolved** and the tiers re-sorted so that Tier 0 is movement and camera, which is what gets
worked next. **Old numbers are dead and are not reused.** `ROADMAP.md` was updated in the same pass.

| Was | Now | Why it moved |
|---|---|---|
| M.3 | **0.12** | Camera bug. Top tier |
| M.11 | **0.13** | Camera. **Narrowed**: the trick-camera half is closed, see the item |
| M.13 | **0.14** | Camera. Top tier |
| 3.13 | **0.15** | Camera, unblocked, and it gates two feel items |
| 2.1 | **0.16** | Jump is movement |
| 2.6 | **0.17** | Movement and camera feel (had already absorbed 0.9) |
| 5.6 | **0.18** | Movement. Already gated PRE-ALPHA 1 |
| 3.14 | **0.19** | Camera tooling |
| M.12 | **2.11** | Movement and camera, but **blocked on VFX**, so it defers |
| 0.3 | **4.1** | Verification debt. Blocks nothing |
| 0.5 | **4.2** | Instrumentation gap |
| 0.6 | **4.3** | Verification debt |
| 0.7 | **4.4** | Build checks |
| 0.11 | **4.5** | A lying instrument |

## Retired numbers

Closed and removed. Listed so an old note or commit message still leads somewhere, and so numbers are never
reused.

| Numbers | Where the outcome lives |
|---|---|
| M.1, M.2, M.4, M.14, 2.3, 2.8, 5.9, and all of the old Tier 4 | `CLAUDE.md` (standing decisions, module constraints) |
| M.5, M.6, M.7, M.8, M.9, M.10, 0.8, 0.10, 2.10 | `TuningLog.md` (measured tables and rejected attempts) |
| 0.1, 0.2, 3.4 | Done, nothing worth keeping |
| 0.9 | Merged into 0.17, which owns the whole boost judgement |

**The old Tier 4 was a deletion list and is fully executed**, which is why the number was free to reuse for
verification debt. Three dead APIs were removed. If a lock tone or a HUD pitch indicator is ever wanted, add
it deliberately rather than restoring them: each was dead because the consumer that justified it never
existed.
