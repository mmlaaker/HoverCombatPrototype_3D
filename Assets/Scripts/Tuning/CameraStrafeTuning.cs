using System;
using UnityEngine;

/// <summary>
/// Strafe-mode camera behaviour. Pure designer-facing values. No scene refs,
/// no runtime state.
/// </summary>
[Serializable]
public class CameraStrafeTuning
{
    // -------------------------------------------------------------------------
    // 📐 Resting Framing
    // -------------------------------------------------------------------------
    [Header("📐 Resting Framing")]
    [Tooltip("Where the strafe camera sits, relative to the craft: the aiming pull-in. THIS is " +
             "the authoring surface; VCam_Strafe's own Follow Offset is an OUTPUT, overwritten " +
             "every frame. Unlike drive there is no orbit, so the vector is used exactly as " +
             "written. Safe to drag during play.")]
    public Vector3 followOffset = new Vector3(0f, 3.5f, -6f);

    // -------------------------------------------------------------------------
    // 🔍 Zoom
    // -------------------------------------------------------------------------
    [Header("🔍 Zoom")]
    [Tooltip("FOV reduction (degrees) against the drive camera's base FOV, for a subtle zoom " +
             "in strafe mode. Recomputed every frame, so this and Base Fov can both be dragged " +
             "during play. Pull-in DISTANCE is separate and lives in Follow Offset above.")]
    [Range(0f, 20f)]
    public float fovReduction = 10f;

    // -------------------------------------------------------------------------
    // 🪂 Damping
    // -------------------------------------------------------------------------
    [Header("🪂 Damping")]
    [Tooltip("Vertical position damping in strafe mode. Strafe is otherwise bolted to the " +
             "chassis with zero damping, which is what makes jumps read as rigid. Damping the " +
             "VERTICAL axis only softens the jump without touching aim, because the crosshair " +
             "is yaw and pitch and damping those would corrupt it. Keep small: 0.2 to 0.4. " +
             "0 restores the old fully locked behaviour.")]
    [Range(0f, 1f)]
    public float verticalDamping = 0.25f;
}
