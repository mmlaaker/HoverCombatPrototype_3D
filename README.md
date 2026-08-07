# Hover Combat Prototype 3D

This repository contains the hover combat prototype (Unity 6.3 URP).

## Project documentation
- `CLAUDE.md` - architecture, module status, design pillars, known issues (auto-loaded by Claude Code)
- `GameDesignDocument.md` - full design reference
- `TODO.md` - all open work: verification debt, blockers, known traps, design decisions, unimplemented features, and the list of things consciously closed
- `PhysicsAudit.md` - physics derivations and method. Point-in-time snapshot of `4a34f21`; the method is current, the values are not

There is no handoff document. `CLAUDE.md` records what the systems are and why; `TODO.md` records what is not done.

## Quick start
1. Open in Unity 6000.3.8f1 (Unity 6.3) with URP.
2. Open `Assets/Scenes/Prototype_Scene.unity` (the only scene).
3. Press Play in Editor.

## Workspace conventions
- Core game logic lives in `Assets/Scripts/*.cs` (hover controllers, weapons, AI, HUD).
- Data driven values use ScriptableObjects in `Assets/Data/`.
- Input uses Unity Input System actions.
- Camera uses Cinemachine.

---

## Contacts
- Author/owner: Meade Laaker
- Review requests: via GitHub PR, include a context section pointing at the relevant `CLAUDE.md` module entry
