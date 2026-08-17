# Hover Combat Prototype: Roadmap

Owner's milestone definitions, recorded 2026-08-14. **This file owns the definition of "done" at
each stage and the order the work happens in. It owns no facts of its own:** every item below is a
pointer into `TODO.md` by number, so nothing here can go stale independently of the item it names.

The distinction against the other documents: `GameDesignDocument.md` says what the game *is*,
`TODO.md` says what is *unfinished*, and this file says **which unfinished things stand between here
and the next stage.** `TODO.md` has no stage gating and is ordered by tier, not by milestone; that
is what this exists to supply.

**Art vocabulary, owner's definition:** *greybox* is untextured blocking, *placeholder* is
stand-in art that reads correctly, *non-final* is more refined than placeholder but explicitly not
ready to ship. Nothing on this roadmap reaches final art.

---

## PRE-ALPHA 1 — movement, one gun, nothing to shoot at

> Movement physics and machine gun. Greybox art. Test environment. No enemies. Bare minimum VFX.

**Status: essentially reached.** Movement was judged good on 2026-08-14 (see `CLAUDE.md`, the
movement pass is accepted); the test environment, greybox art and the machine gun are all in place;
the first build runs at 6.07ms with a 0.39ms spread.

Remaining, and the owner's own framing is "a few minor things":

The owner's own estimate 2026-08-14 is **movement at ~88%, wanting ~95% before calling this done.**
What stands in the way:

**`TODO.md` Tier 0 IS this milestone's work list**, since the 2026-08-16 re-tiering put movement and
camera in the top tier. Every row below is a Tier 0 item.

| | |
|---|---|
| **0.12** | a sliver of bumper still clips on the pitch-up. **Largely resolved 2026-08-17** by `minFrameMargin` 5; owner accepted the residual |
| **0.13** | the camera has no answer while the craft is flipped. **Narrowed 2026-08-16:** the trick-camera half was tested and rejected |
| **0.14** | strafe camera view retune. Owner's own task; retune the view first, then match `reticleProjectionDistance`. **Next up** |
| **0.16** | a denied jump gives no usable feedback. **Not judgeable in this build**, its channel is deliberately off |
| **0.18** | `flipRecoverySpeedThreshold` 2 is an unconfirmed guess. Settle alongside `0.13` |
| **0.19** | no camera preview state can reproduce a mid-flip pose. Tooling, lowest priority here |
| **0.20** | boost engaged from a standstill reads as a jolt. `ForwardGate` delivers the whole package in ~15ms |
| **0.21** | fall gravity wants another increase, bounded by the hard-landing rules and the trick landing margin |
| **0.23** | the drift hop has no cue of any kind. `OnDriftHop` has zero subscribers |
| **0.24** | thruster VFX and the boost camera fall out of step on release. Depends on `0.20` |

**Closed since this list was written:** `0.15` (never a defect, a deliberate playtest scoping
decision), `0.17` (boost drive mode judged good; residual became `0.20`), and `0.22` (aiming moved
the craft, opened and closed 2026-08-17). See `TODO.md` > Retired numbers.

Also worth doing during the next long run, though it gates nothing: **4.4**, confirm the marker key
works in a build and measure allocation under sustained fire.

Closed on 2026-08-15: the trick energy economy, built and judged good in one session. That was the
last open item that changes how movement PLAYS rather than how it behaves.

Deferred out of this milestone by the owner: **2.11** (boost presentation FX, blocked on the general
VFX pass) and **5.11** (strafe camping) both move to PRE-ALPHA 2. The second is principled: it asks
whether players camp under combat pressure, and there is no combat until `1.3`.

Note the AI already exceeds this milestone's bar: it drives, roams, flees and shoots, where
PRE-ALPHA 1 asks for no enemies at all.

---

## PRE-ALPHA 2 — weapons become real, and something takes the hits

> Movement and weapon physics. Grey-box art. Test environment. Enemy punching bag. Placeholder VFX.

The step is **weapon physics** and **a target worth hitting**. The AI punching bag largely exists
(`1.6` records it as deliberately passive), so most of the work is on the weapons themselves.

| | |
|---|---|
| **1.3** | four of six working weapons deal zero damage. **The root item** -- nothing can die until it moves |
| **1.4** | every weapon has unlimited ammo |
| **2.7** | automatic fire rates are far too low |
| **2.2** | EMP launch has no acknowledgement. Wired, unjudged, and **not judgeable in this build**: its channel is deliberately off |
| **2.4**, **2.5** | hard landings have no dust, missile detonations have no explosion. This milestone's "placeholder VFX" |
| **2.11** | boost presentation FX: vignette, speed lines, duration-based rumble. Deferred here 2026-08-14, and **confirmed as the remaining boost gap** by the owner 2026-08-16 |
| **2.12** | drift has no VFX. Blocked on VFX, so it joins the same pass. The camera half is `0.23` and is not blocked |
| **5.11** | whether players camp in strafe. Unanswerable until `1.3` gives combat something to pressure with |
| **3.1**, **3.6**, **3.7** | weapon-side traps worth clearing while the code is open |
| **5.2**, **5.3**, **5.4** | the knock-around pass, only partly applied |

---

## ALPHA — a loop you can lose

> Core loop and AI Enemy. Placeholder art. Placeholder arena. Placeholder VFX.

The step is **consequence**: death, respawn, pickups, and an AI that can actually threaten.

| | |
|---|---|
| **1.1**, **1.2** | respawn is a stub and post-respawn invulnerability does not exist. Both blocked by `1.3` |
| **1.5** | `PickupManager` is unwritten. The only planned-and-entirely-absent module |
| **1.6** | the AI cannot fire anything except slot 0 |
| **3.3** | the AI steers away from its own target |
| **6.3** | no match flow |
| **5.1** | Hard Lock is not designed through |

Placeholder arena work begins here (`6.2`), replacing the RVP demo city.

---

## BETA — five opponents, and art that means it

> Core loop vs 5 AI enemies. Non-final art. Non-final arena. Non-final VFX.

The step is **scale and presentation**. Five simultaneous AI is a performance question as much as a
design one, and nothing has ever been measured with more than two craft in the scene.

| | |
|---|---|
| **6.2** | the arena, at non-final quality |
| **6.1** | seven of thirteen weapons still unimplemented |
| **5.7** | vehicle roster is one profile |
| **5.8** | vehicle scale, deferred behind two gates |
| — | a performance pass against five AI. The instruments exist; the measurement does not |

---

## What this roadmap does not cover

Final art, campaign (`6.4`), multiplayer (`6.5`) and the GDD diagrams (`6.6`) all sit beyond BETA
and are not sequenced here.
