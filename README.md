# Hover Combat Prototype 3D

This repository contains the hover combat prototype (Unity 6.2 URP).

## Project documentation reference
All design and architecture details now live in dedicated project-context Markdown files:

- `PROJECTCONTEXT_OVERVIEW.md` - file map and navigation
- `PROJECTCONTEXT_IDENTITY.md` - identity, vision, design pillars
- `PROJECTCONTEXT_MOVEMENT.md` - movement model and core gameplay loop
- `PROJECTCONTEXT_ENERGY_ABILITIES.md` - energy system + ability contracts
- `PROJECTCONTEXT_COMBAT.md` - weapons, pickups, vehicle/audience flow, arena philosophy
- `PROJECTCONTEXT_STYLE_MARKET.md` - visual style, market position, publishing strategy
- `PROJECTCONTEXT_TECHNICAL.md` - engine stack, module status, principles, checklist

## Quick start
1. Open in Unity (2023.x+ with URP).
2. Open `Assets/Scenes/Prototype.unity` (or equivalent main scene).
3. Press Play in Editor.

## Workspace conventions
- Core game logic lives in `Assets/Scripts/*.cs` (hover controllers, weapons, AI, HUD).
- Data driven values use ScriptableObjects in `Assets/Data/`.
- Input uses Unity Input System actions.
- Camera uses Cinemachine.

## Notes for the assistant
Whenever you work on new features or bug fixes:
- read `PROJECTCONTEXT_OVERVIEW.md` first
- align changes with design pillars in `PROJECTCONTEXT_IDENTITY.md`
- track technical debt in `PROJECTCONTEXT_TECHNICAL.md`

---

## Choosable quick tasks
- [ ] Fix hover drift jitter (see tech section)
- [ ] Balance energy jump/boost pacing
- [ ] Add pickup spawn pattern test

---

## Contacts
- Author/owner: Meade Laaker
- Review requests: via GitHub PR, include context section pointing to any `PROJECTCONTEXT_*` reference
