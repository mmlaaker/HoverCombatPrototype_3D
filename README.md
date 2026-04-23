# Hover Combat Prototype 3D

This repository contains the hover combat prototype (Unity 6.2 URP).

## Project documentation
- `CLAUDE.md` - architecture, module status, design pillars, known issues (auto-loaded by Claude Code)
- `GameDesignDocument.md` - full design reference

## Quick start
1. Open in Unity (2023.x+ with URP).
2. Open `Assets/Scenes/Prototype.unity` (or equivalent main scene).
3. Press Play in Editor.

## Workspace conventions
- Core game logic lives in `Assets/Scripts/*.cs` (hover controllers, weapons, AI, HUD).
- Data driven values use ScriptableObjects in `Assets/Data/`.
- Input uses Unity Input System actions.
- Camera uses Cinemachine.

## Quick tasks
- [ ] Fix hover drift jitter (see tech section)
- [ ] Balance energy jump/boost pacing
- [ ] Add pickup spawn pattern test

---

## Contacts
- Author/owner: Meade Laaker
- Review requests: via GitHub PR, include context section pointing to any `PROJECTCONTEXT_*` reference
