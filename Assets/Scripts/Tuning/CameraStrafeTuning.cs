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
    [Tooltip("Vertical position damping in strafe mode, applied only while the craft is OFF its " +
             "hover springs. Strafe is otherwise bolted to the chassis with zero damping, which is " +
             "what makes jumps read as rigid. Damping the VERTICAL axis only softens the jump " +
             "without touching aim, because the crosshair is yaw and pitch and damping those would " +
             "corrupt it.\n\n" +
             "It fades in with hover support, so it is fully absent while grounded and fully " +
             "present in free air. That gating is not optional decoration: applied flat, this " +
             "value also lags the small lift the craft gets from throttling forward, which leaves " +
             "the camera low and tips the view up about 8.7 degrees, dragging the crosshair with " +
             "it. Gating on vertical SPEED instead was tried and is much worse, because ordinary " +
             "slopes produce jump-sized vertical speeds and the damping then flickers on and off " +
             "several times a second, which reads as bumpiness on every hill.\n\n" +
             "Keep small: 0.2 to 0.4. 0 restores fully locked behaviour and costs you soft jumps.")]
    [Range(0f, 1f)]
    public float verticalDamping = 0.25f;

    [Tooltip("How fast Vertical Damping is allowed to change, in units per second. Landing is what " +
             "this is for.\n\n" +
             "Hover support goes from 0 to 1 in about 20ms when a jump touches down, so without a " +
             "limit the damping collapses in two frames and the rig snaps out the ~2m of lag it " +
             "built up during the fall. That snap IS the landing stutter; the craft itself lands " +
             "smoothly.\n\n" +
             "Lower is a softer landing and a camera that stays slightly loose for longer after " +
             "you touch down. Higher approaches the old snap. Below about 0.3 the looseness starts " +
             "being noticeable while driving away from a landing.\n\n" +
             "0 freezes the damping wherever it happens to be, which is not what you want; use a " +
             "large value, not zero, to disable the limit.")]
    [Range(0f, 20f)]
    public float verticalDampingSlew = 0.6f;
}
