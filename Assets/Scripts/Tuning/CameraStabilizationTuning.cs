using System;
using UnityEngine;

/// <summary>
/// Heading stabilization for the drive camera. Pure designer-facing values.
/// No scene refs, no runtime state.
///
/// These drive the CameraHeadingProxy introduced in HoverCameraController v1.2.
/// The behaviour they control is subtle and was hard-won; see that class for
/// why the drive camera follows a yaw-only proxy rather than the vehicle.
/// </summary>
[Serializable]
public class CameraStabilizationTuning
{
    // -------------------------------------------------------------------------
    // 🧭 Heading Proxy
    // -------------------------------------------------------------------------
    [Header("🧭 Heading Proxy")]
    [Tooltip("Vehicle tilt (degrees from upright) beyond which the drive camera stops reading " +
             "yaw from the transform and integrates the Rigidbody's yaw rate instead. Prevents " +
             "the 180-degree camera swing when the nose pitches through vertical during a flip. " +
             "Try 45 to 75.")]
    [Range(30f, 85f)]
    public float maxStableTilt = 60f;

    [Tooltip("How fast the stabilized heading re-converges to the vehicle's true yaw while " +
             "upright (per second). High enough to be imperceptible in normal driving, low " +
             "enough to avoid a snap after a flip lands on a changed heading. Try 8 to 15.")]
    [Range(1f, 30f)]
    public float headingSyncSpeed = 12f;
}
