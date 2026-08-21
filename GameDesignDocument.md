# 🕹️ Hover Combat Prototype
*A ground-level aerial dogfighting game inspired by **Twisted Metal** and **Rocket League.***

> **This document is design intent.** It describes what the game should be, and stays valid whether
> a thing is built yet or not. It deliberately carries no implementation status and no open task
> list, so that it never has to be corrected when code changes.
>
> | Question | Document |
> |---|---|
> | What should this be? Why? | **`GameDesignDocument.md`** (this file) |
> | What exists, how does it work, why is it built that way? | `CLAUDE.md` |
> | What is not done? | `TODO.md` |
> | How were the physics numbers derived? | `PhysicsAudit.md` (frozen at `4a34f21`; method current, values not) |
>
> Live tuning values come from `Assets/Data/*.asset`, never from any document.

---

## 1. Project Overview

This project is a hover vehicle combat game built around a single design conviction: **momentum management is the primary skill expression.**

Vehicles behave like low-altitude jets dogfighting above the ground. Combat and traversal are the same action: players who control momentum win fights. Players who react to momentum lose them.

The solo campaign pits the player against AI opponents across a roster of distinct vehicles, each with a unique pilot and reason to compete. This is a full product layer, not a test harness. Multiplayer is a future phase; the solo game stands alone.

Initial prototype scope: 1v1 combat against AI, expanding to 3-5 opponents per match.

---

## 2. Core Design Pillars

1. **Momentum Is the Resource**  
   Boost, jump, drift, and strafe are not just movement tools. They are the primary weapons. A player who enters a fight with speed and a planned approach vector beats a player who aims better but moves reactively. Every design decision is evaluated against this principle first.

2. **Combat and Traversal Are the Same Skill**  
   There is no mode switch between moving and fighting. The same inputs, the same momentum management, the same map reading: all of it applies simultaneously. A fight is two momentum vectors intersecting. The player who constructed the better approach angle three seconds earlier wins.

3. **Hit Disruption Over Flat Damage**  
   Being hit disrupts momentum first, health second. A hit that spins you out or kills your speed is more punishing than one that reduces HP, because it costs you the resource you are actually managing. Weapons that manipulate momentum vectors are prioritized over weapons that simply deal damage.

4. **Shared Arsenal, Distinct Profiles**  
   All vehicles share the full weapon roster and ability set. Vehicle identity comes from handling profile and one unique special weapon only. Vehicles change *how* actions are executed, never *whether* they are available. No vehicle is locked out of any situation.

5. **Energy as Tempo**  
   The energy meter governs mobility and utility, not weapons. Managing it is managing tempo: the ability to dictate when and where fights happen. A player who burns energy poorly loses the ability to disengage, reposition, or respond. Energy discipline is fight discipline.

6. **Feel First**  
   Mechanics must be fun before they are beautiful. Tuning handling and responsiveness takes priority over visual polish at every stage of development.

---

## 3. World and Narrative Framing

This is not a narrative game. The world exists to make the combat coherent and give each vehicle a reason to be in the arena. No deep lore is required or desired.

**The Premise**

An intergalactic combat sport (part sanctioned league, part blood sport) that draws pilots across species, factions, and civilizations. Nobody fully agrees on who started it or whether it is legitimate. It does not matter. The arena is the equalizer.

**The Pilots**

Each vehicle has a pilot with an origin and a reason to compete. Pilots may be human, alien, AI, cyborg, or biological symbiote. The roster spans the breadth of the universe that hosts the sport. Their vehicles reflect their philosophy, their faction, and their resources. A corporate human with a sponsor-plastered machine. An alien whose civilization has been doing this longer than humans have had writing. A scrappy mechanic who built their rig from salvage. A military AI drone entered by an institution with unclear motives. A biological symbiote craft where the line between pilot and vehicle is genuinely ambiguous.

**Campaign Structure**

Twisted Metal's model: per-vehicle pilot stories told through brief intro and outro cinematics. Players complete the campaign with one vehicle to see that pilot's outcome, then replay with another. Low production cost, high replayability, distinct perspective per vehicle.

**Weapon Faction Aesthetic**

Human-origin vehicles tend toward physical weapons: bullets, explosions, kinetic force. Alien and synthetic vehicles tend toward energy weapons: plasma, lasers, gravitational manipulation. Both coexist in every match. The distinction is aesthetic and identity-driven, not a balance lever. No weapon type is exclusive to any faction.

---

## 4. Gameplay Summary
| System | Description |
|--------|--------------|
| **Camera** | Third-person follow camera with mild positional lag and rotation offset controls. |
| **Vehicle Movement** | Hovercrafts use a Rigidbody-based controller. Ground proximity forces stabilize hover height. Steering blends drift and strafing. |
| **Combat** | Forward-facing weapons; player aims primarily via vehicle orientation. No soft lock or perfect tracking. |
| **Abilities** | Non-damaging utility powers that consume shared energy: <br>• **Boost** – temporary acceleration burst.<br>• **Jump** – instant hover lift.<br>• **Shield** – temporary damage resistance.<br>• **EMP** – soft-homing electric projectile; freezes energy use on unshielded hit. |
| **Weapons** | Shared 12-weapon roster (Machine Gun is infinite ammo; all others pickup-limited). See Section 9 for the full table. |
| **Pickups** | Map-scattered items for ammo, health, and energy recharge. |
| **Game Modes** | 1v1 versus AI opponent (initial). Later expansion: 3–4 combatants per match. |

---

## 5. Technical Overview

| Category | Implementation |
|-----------|----------------|
| **Engine** | Unity 6.3 URP |
| **Physics Base** | Rigidbody-based hover vehicle controller |
| **Input** | Unity Input System (single-player bindings for keyboard/controller) |
| **Camera System** | Third-person follow camera with aim assist offset |
| **Core Resources** | • **Energy Meter:** shared pool for mobility & utility<br>• **Ammo:** per-weapon pickup pools |
| **Art & VFX** | Placeholder meshes and particle effects. Visual polish deferred until mechanics are fun. |
| **AI (Phase 2)** | Simple finite-state machine (FSM) for chasing, evading, and shooting behavior. |

---

## 6. Control Scheme (Prototype)
**`Assets/Scripts/HoverControls.inputactions` is the source of truth for this table, exactly as
`Assets/Data/*.asset` is for tuning values.** Corrected against it 2026-08-18, when the two had
drifted: the table had claimed yaw was on the left stick. **The prototype is gamepad-only. The asset
binds no keyboard controls at all**, and the keyboard column this table used to carry described
bindings that do not exist.

| Action | Binding | Notes |
|--------|----------------|-------|
| Thrust / Reverse | Left Stick Y | `Hover/Throttle` is read as a full Vector2 (Y = throttle, X = strafe). Reverse doubles as the brake |
| Strafe Left / Right | Left Stick X, while Strafe Mode is held | Lateral movement, on the same stick as throttle |
| **Turn Left / Right** | **Right Stick X** | Yaw rotation. **Deliberately not the left stick**, whose X axis is already strafe |
| Camera Pitch | Right Stick Y | Drive Mode: camera pitch. Strafe Mode: vehicle nose pitch |
| Strafe / Aim Mode | `Hover/Strafe` — L2 / Left Trigger | Held, not toggled. Blends in lateral authority and free-aim pitch |
| Jump | `Hover/Jump` — Cross | Hold to charge, fires on release. One air jump per landing. Consumes energy |
| Boost | `Hover/Boost` — L3 | Continuous drain while held. In Strafe Mode without forward throttle it fires a dodge burst instead |
| Drift / Air Control | `Hover/Drift` — **L1 or R1** | Either bumper. Grounded: drift. Airborne: 3-axis air control (left stick = pitch/roll) |
| Fire | `Hover/Fire` — R2 | One fire action for all weapons. Weapon behaviour is per-slot, not per-button |
| Cycle Weapon | `Hover/CycleWeaponNext` / `Prev` — D-Pad Right / Left | There is no separate primary/secondary fire; the roster is a cycled slot list |
| Activate Shield | `Hover/Shield` — D-Pad Down | Energy cost, brief invulnerability |
| Fire EMP | `Hover/EMP` — D-Pad Up | Tap; ~70% energy cost. The action is required — `PlayerHoverInput` disables itself if it is missing from the asset |

There is no reset-orientation input. Righting a flipped craft is automatic and deliberately not player-cancellable — see Flip Recovery.

Two debug controls exist outside the asset, read straight off the device so they never require editing it: **hold Share** for the playtest reset, and **Options** for the session splash and controls overlay. See `CLAUDE.md` > Instrumentation.

---

## 7. Vehicle Behavior Breakdown

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

## 8. Vehicle Identity Philosophy

**Two Axes, Not One Spectrum**

Vehicle profiles are defined by two independent axes:

- **Mass and momentum character:** How the vehicle builds, holds, and loses speed. Not just top speed. The personality of momentum.
- **Stability vs agility:** How the vehicle handles directional change and how it responds to being hit.

These axes are independent. A vehicle can be high mass and low stability simultaneously. This creates a two-dimensional design space where vehicles occupy genuinely distinct positions that do not reduce to a simple fast/slow hierarchy.

**The Distinction That Prevents Hero Shooter Drift**

Vehicles change *how* actions are executed, never *whether* they are available. Every vehicle can pursue, escape, fight in corridors, fight in arena rooms, and use every weapon in the shared arsenal. The tuning changes the expression of those actions, not the availability of them.

The moment a player looks at a situation and thinks "I need the heavy vehicle for this" rather than "I prefer the heavy vehicle", the design has crossed into role selection territory. That is the line. Every vehicle must be capable of winning every situation.

**The Special Weapon Test**

The special weapon must pass this test before it is considered valid: would a player choose this vehicle *because of* the special, or *despite not needing* the special?

If the answer is because of it, the special is doing too much work and is functioning as a loadout tool. The vehicle identity must come from the tuning. The special is a stylistic exclamation point that expresses that identity in a spectacular way, not the reason the identity exists. Specials are allowed to bend the shared rules and provide spectacle. They are not allowed to be the answer to a specific tactical problem.

**Proposed Five-Vehicle Roster Framework**

| Profile | Momentum Character | Stability | Pilot Archetype |
|---|---|---|---|
| Bruiser | High mass, holds momentum relentlessly | High stability | Ancient alien civilization: unstoppable by tradition |
| Interceptor | Low mass, loses momentum fast | High agility | Biological symbiote: organic, reactive, fragile |
| Chaos Vehicle | High mass, unpredictable on impact | Low stability | Scrappy human mechanic: powerful and held together by stubbornness |
| Precision Drone | Low mass, operating at tolerance limits | Low stability | Military AI: calculating but unforgiving |
| Transition Specialist | Balanced mass | Specialized: fastest state transitions | Corporate human: tournament-engineered for adaptability |

No vehicle is strictly better than another in all situations. The bruiser beats the interceptor in a corridor but loses in an arena room. The chaos vehicle is terrifying when it works and a liability when it does not. The transition specialist does not win any specific matchup but never loses one badly either.

---

## 9. Combat Systems

**Weapon Slots**

All vehicles share the full weapon roster. Vehicle identity comes from handling profile and one unique special weapon. Weapons use limited ammo sourced from map pickups, except the Machine Gun which has infinite ammo.

### Energy System

Governs non-damaging abilities only. Weapons never spend energy. Regenerates over time when not actively consumed. Depleted by: Boost, Jump, Shield, EMP.

### Shared Abilities

| Ability | Input | Cost | Contract |
|---------|-------|------|----------|
| Boost | Hold | Continuous drain | Succeeds when used for repositioning, not permanent speed |
| Jump | Tap / Hold to charge | Flat cost | Succeeds when verticality meaningfully disrupts targeting |
| Shield | Tap | Flat cost | Succeeds when timed, not spammed. Absorbs one incoming EMP hit; shield deactivates, no freeze applied |
| EMP | Tap | ~70-80% of meter | Soft-homing electric projectile. Direct hit only, no AOE. On unshielded hit: applies energy freeze. On shielded hit: destroys shield, no freeze. Succeeds when it denies tempo, not when it deletes control |

### Knockback and the Flip Threshold

Measured in-engine, and it changes how weapon force should be authored.

**Force and concussive are the two words this document uses, and each maps to exactly one authored value.** Owner vocabulary, 2026-08-21. **Force** is the linear push and is `impact.impactForce`. **Concussive** is how much of that push becomes rotation -- the flip component -- and is `impact.destabilizeFraction`. Every weapon in the roster is therefore specifiable as two numbers plus a delivery mechanism, and the numbers are directly comparable across weapons.

**One coupling to know before authoring them: `destabilizeFraction` also governs self-inflicted spin, so past roughly 0.36 a rocket jump starts flipping the player who fired it** (`TODO.md` 5.3).

A vehicle that ends up resting past **80 degrees of tilt** is *downed*: jump, steering and thrust are all locked out for roughly **1.6 seconds**, and no amount of player input shortens it. So every knockback value implicitly picks one of two outcomes, and the gap between them is enormous:

- **Below the threshold**, a hit is momentum disruption. It shoves you off your line and you drive out of it. This is what the pillars ask for.
- **Above it**, a hit is a hard punish with a fixed timer, and the victim's skill stops mattering entirely.

Two rules fall out of this:

1. **Express knockback as a fraction of top speed, not as a raw number.** A hit that imparts more than top speed is not disruption, it is removal: the target is travelling faster than it can drive and its own momentum skill is irrelevant until it stops.

   **This rule was authored against a top speed of 60 and the speed pass moved it to 105, which silently changed what every weapon means.** The Rocket Launcher was measured imparting ~100 m/s, which was 1.67x top speed -- removal, comfortably. Against 105 the same force is 0.95x, which by this rule is now *disruption*. Nothing rechecked the roster when the ceiling moved. **Re-measure against 105 and author the whole roster from that baseline** (`TODO.md` 5.16); the numbers quoted above are the old baseline and are kept only to show which way the error runs.
2. **Where you hit matters more than how hard.** The chassis rolls about three times more easily than it pitches, so a hit that catches a flank high tips the craft far more readily than a square hit of the same force. A centred hit at full Rocket Launcher force barely tilts the target 8 degrees; the same force landing high on the flank rolls it completely over, every time.

This split is now the intended difference between the two homing weapons, verified over repeated identical shots:

| Weapon | Flank hit outcome |
|---|---|
| Rocket Launcher | Rolls fully over, 167 degrees, downed. Hard punish |
| Soft Homing | Rocks about 22 degrees and recovers. Pure disruption |

That is the design goal reached deliberately rather than by accident, and it is the model for tuning the rest of the roster: **spectacle weapons flip, pressure weapons do not.**

### Projectile Flight as Identity

A missile's *path* is a design surface, not just a delivery mechanism. Three dials shape it, and they are per-weapon:

- **Straight-flight delay.** How long the missile flies dumb before it starts chasing. At zero, homing shots feel cheap and unavoidable. A short delay lets the target see it coming and gives the missile a sense of weight.
- **Flare.** How far the missile swings wide before curling onto its target, expressed as a share of the distance to the target so it looks right at every range. Direction can alternate or randomise per shot, which is what makes a volley look like a volley instead of the same animation replayed.
- **Turn rate.** The turning circle. This is the accuracy dial, and **since 2026-08-21 it is also the evasion dial**: with projectiles faster than vehicles, turn rate is what decides whether a dodge beats the shot. Easy to set too low chasing a graceful arc, and now easy to set too high and remove the counter-play.

Crucially, the flare costs no accuracy: the missile aims at a point *beside* its target and that point slides onto the target as it closes, so it always converges. Spectacle and reliability are not in tension here.

**Speed catches, turn rate is what skill evades. Owner decision, 2026-08-21, and it reverses what this section used to say.** Projectiles are faster than vehicles: a weapon should generally be able to catch a straight runner, and **clever movement -- dodges, jumps, changes of line -- is what evades it.** Running in a straight line is not meant to be an escape.

That makes **turn rate the evasion dial**, not merely the accuracy one. A fast projectile with a wide turning circle catches the runner and loses to the dodge, which is exactly the intended split. Both dials are now design surfaces per weapon.

**The number to clear is the boosted ceiling, not top speed.** `topSpeed` is 105 and `boostSpeedMultiplier` is 1.25, so a boosted craft reaches **131 m/s**. All three missiles currently sit at `speed: 70` -- **53% of a boosted runner**, so they are not closing slowly, they are being lapped. Clearing 131 with real margin is the requirement (`TODO.md` 5.5).

*What this replaced, because the reversal matters: this section previously read missiles at 70 against a top speed of 60, concluded that homing weapons were tools for punishing players who are turning or cornered rather than for chasing runners, and advised that speed was the value to change and turn rate was not. The first half is no longer the design and the second half is now only half right.*

### Weapons

| # | Weapon | Input | Ammo | Notes |
|---|--------|-------|------|-------|
| 1 | Machine Gun | Hold | Infinite | Sustained fire. Delivers force that pushes but is **not concussive** and does not flip. Low DPS, always available, requires exposure commitment |
| 2 | Chain Gun | Hold | Limited | Sustained fire that must wind up and wind down. **The better Machine Gun that is not free.** Delivers force, not concussive. Fire rate scales via AnimationCurve |
| 3 | Shotgun | Tap | Limited | Short-range burst damage in a tight cone. Force **and** concussive. **May flip at exactly the right angle and distance, but generally will not** -- the pellets spread across the hull and their torques largely cancel, so the spread pattern is the physics as much as the look. Succeeds only when lethal proximity is achieved |
| 4 | Rocket Launcher | Tap | Limited | High force with a concussive blast radius. Knocks around vehicles and props. **Straight line, no homing.** The primary momentum disruption tool |
| 5 | Lob Bomb | Tap | Limited | Variant of the Rocket delivered over a launched arc instead of a straight line. **Identical to the Rocket for now except in trajectory**, and likely to grow bigger to pay for the slower delivery |
| 6 | Soft Homing Missile | Tap | Limited | Smaller force, small blast radius, concussive. **Soft homing means it can be out-manoeuvred and miss.** Harassment and pressure tool |
| 7 | Hard Lock Missile | Hold to scan and lock, **release to fire** | Limited | A volley of missiles locking onto one or more targets. Smaller force and blast radius, concussive. The lock commits when acquisition starts and holds, which is the primitive the multi-target version needs |
| 8 | Hard Lock Chain Lightning | Hold to scan and lock, **release to fire** | Limited | Variant of the Hard Lock Missile that delivers **near-instant damage** instead of missiles that have to find their targets. No blast radius, not concussive; possibly an impact force |
| 9 | Laser Cannon | Hold to charge, release | Limited | Releases a sustained beam that **carries force but no blast radius**. Pierces multiple targets. Short charge = weak shot, full charge = peak damage window that tapers off |
| 10 | Gravity Well / Repulsor Bomb | Tap to deploy | Limited | Lobbed deployable that pushes or pulls everything caught in it. Drains energy of caught vehicles. Duration-limited with visible decay VFX. One active at a time. Does not affect the deploying vehicle |
| 11 | Bouncing Disc Blade | Tap | Limited | Deployable that ricochets off walls, floors and ceilings in 3D space and deals damage on impact. **Impact rather than blast, but it may destabilise.** Rewards map literacy and spatial prediction |
| 12 | Mine | Tap to deploy | Limited | Force and concussive damage in a blast radius. **Consolidates what were previously two mines** (floating proximity and directional remote) into one weapon |

**Held, and deliberately undecided.** Both are gated on play rather than on argument:

| Weapon | The question | What answers it |
|---|---|---|
| **Sniper** | Instant hit, high damage, low fire rate, zoom that blinds outside the scope | **Its strongest claim was being the only weapon that could touch a runner, and the speed rule above just gave that to the whole roster.** What remains is long-range precision, which is a weaker case. Build Chain Lightning (8) first and see whether a long-range hole still exists |
| **Flamethrower** | Short-range sustained cone delivering a continuous shove rather than a burst | **The cheapest weapon on this list** -- it is the Machine Gun's particle machinery through a wide cone, no new systems. It overlaps the Laser Cannon (9), which is the most expensive. Build the Flamethrower first: it answers whether sustained force is fun before the Laser spends a beam system finding out |

**The Special is a concept, not a slot.** Every vehicle gets a unique special weapon and what those are is TBD. It was removed from this table on 2026-08-21 because reserving a numbered slot for it implied a design that does not exist yet: there is one vehicle profile (`TODO.md` 5.7), so "one per vehicle" has nothing to vary against. **The expected route is promotion** -- build a weapon for general use, and designate it a Special if it turns out to want that.

### Implementation status

Deliberately not recorded here. This document is design intent; it should stay readable as the
specification whether a weapon is built or not.

- **What is built, with which asset and delivery mechanism:** `CLAUDE.md` > Weapon Implementation Status.
- **What is not built, and the open design decisions attached to it** (the damage pass, ammo,
  Hard Lock's multi-target shape, the knock-around split): `TODO.md`.

### Pickups
- **Ammo Pickup:** Restores secondary ammo.  
- **Energy Cell:** Rapidly recharges energy.  
- **Health Repair:** Restores hit points.

---

## 10. Arena Design Philosophy

**The Core Model: Track With Rooms**

Arenas are not open bowls or pure mazes. They are tracks with rooms. The track provides directionality and flow: corridors where momentum builds and pursuits develop. The rooms are arena spaces where the flow opens up, a pickup lives, a hazard exists, or a chokepoint forces interaction. Players move through the track to reach the rooms. The rooms are where fights happen.

**Three Vertical Layers**

Every arena operates on three levels:

- **Ground level:** Primary combat surface. Wide enough for strafe and boost play, broken by obstacles and cover that reward spatial awareness.
- **Mid level:** Ramps, platforms, elevated routes. Where pursuits and escapes live. A vehicle with momentum advantage should be able to gain or lose an opponent by going vertical.
- **High level:** Dramatic verticality. Launch ramps, high platforms, deliberate fall lines. High risk, high reward positioning. Falling from height is a tactical choice with a physics outcome, not a punishment.

**Pickup Placement**

Powerful pickups live in vulnerable positions. Collecting them costs positional exposure. This is the primary mechanism for forcing engagement: players cannot ignore a power weapon spawn without conceding map control. The player who reaches it must accept the risk of being caught in the approach or the collection. This logic is borrowed directly from Quake and Unreal Tournament arena design.

**Parallel Routes and Intersection Design**

Every major node in the arena must be reachable by at least two routes of roughly equal travel time. This enables the head-off play: a pursuing player taking a different cut to intercept rather than chase. If one route is always faster, the meta collapses to a single path and the head-off play disappears.

**Corridor Function**

Corridors are not transitions. They are where momentum builds. A vehicle entering a room with full speed has a different set of options than one who had to brake to make a corner. Corridor width must accommodate vehicle turning radius at speed: tight enough to create pressure, wide enough that drift through a corner feels intentional rather than forced.

**Design Reference**

Quake and Unreal Tournament map logic applied to vehicle scale. Hydro Thunder route hierarchy. Jak 2 Erol race hybrid track/arena structure. F-Zero GX momentum corridor pacing. The emotional target is flow state: the feeling of reading the map fast enough that decisions happen slightly ahead of conscious thought.

---

## 11. Prototype Scope
- **Single level:** small test arena with ramps and obstacles.  
- **Two hovercrafts:** Player and 1 AI.  
- **Functional prototype goals:**
  - Responsive hover movement.
  - Working boost, jump, and shield.
  - Primary & secondary weapon test.
  - Energy and ammo systems integrated.
  - Simple health and damage feedback.

---

## 12. Future Expansion Ideas
- Arena hazards (explosive barrels, shock fields, environmental momentum traps).
- Environmental effects (dust trails, hover distortion, impact debris).
- AI improvements (momentum-aware pursuit, dodge timing, pickup prioritization).
- Multiplayer and split-screen support.
- Vehicle cosmetics.
- Additional vehicles expanding the two-axis roster.
- Per-vehicle campaign missions and cinematics.

*Note: Upgradeable vehicle stats are explicitly out of scope and in conflict with the no-loadout design pillar. Cosmetic customization is acceptable. Stat customization is not.*

---

## 13. Visual Style Goals
- **Tone:** High-energy, colorful, confident, and kinetic. Not grimdark, not sterile, not comedic. The aesthetic is a sci-fi blood sport: dangerous and spectacular without being grim.
- **Vehicles:** Engineered and lived-in. They have been used, modified, and fought in. Not clinical showroom machines. Bold silhouettes that read instantly at high speed.
- **Weapons:** Mixed human physical weapons (bullets, explosions) and alien/energy weapons (plasma, lasers) coexist in the same match. Faction identity is communicated through weapon aesthetic the way Halo communicates it: you know what hit you and who fired it without a UI prompt.
- **Readability:** Visual clarity is a gameplay requirement, not just an aesthetic preference. At high speed, with multiple vehicles in a chaotic fight, every visual element must communicate something useful. Spectacle that reduces readability is a design failure.
- **Reference touchstones:** Wipeout, F-Zero GX, Jak and Daxter hover vehicles, Ratchet and Clank Deadlocked, Unreal Tournament, Jet Set Radio.

---

## 14. Development Notes
- Use **URP** for performance and easy post-processing.  
- **No HDRP** (unnecessary overhead).  
- Keep project modular:  
  - `HoverController_Foundation.cs` and `HoverController_Propulsion.cs` handle hover physics and movement forces.  
  - `HoverController_Energy.cs` governs ability resource usage.  
  - `HoverController_Weapons.cs`, `HoverController_Aim.cs`, and `HoverController_Shield.cs` handle weapons, aiming, and shield activation.  
  - `PickupManager.cs` spawns pickups.  

---

## 15. Diagrams (Planned)
- Control Flow (Vehicle Input → Movement → Abilities → Energy System).  
- Interaction Map (Player ↔ Pickups ↔ Weapons ↔ UI).  
*(Will add via `.png` or `.drawio` export in `/Docs/` later.)*

---

## 16. Implementation Priorities

The intended order of construction, which has not changed:

1. Project, repo, and IDE integration
2. Hovercraft movement & physics
3. Camera and input control
4. Energy + ability system
5. Basic combat loop (primary fire + pickups)
6. AI opponent

**Current progress against this list is not tracked here.** See the Prototype Checklist in
`CLAUDE.md` for where things stand, and `TODO.md` for what remains on each.

---

## 17. Design Philosophy
> *"The fun comes from control."*  
> Hover Combat is about mastering momentum and precision: being slightly overpowered, but always at risk of losing control.

---

## 18. Credits
**Design & Programming:** Meade Laaker (with Claude collaboration)  
**Engine:** Unity 6.3 URP  
**Version Control:** GitHub + Fork + VSCode integration  
