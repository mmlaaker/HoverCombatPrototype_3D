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

    [Tooltip("Ceiling on how fast the stabilized heading may swing, in degrees per second.\n" +
             "The converge above is proportional to error with nothing capping it, so the longer " +
             "a stunt holds air control the harder the camera whips when you let go: measured " +
             "across a session, releasing at 2 degrees of drift cost 31 deg/s while releasing at " +
             "107 degrees cost 217, against about 70 for ordinary hard cornering. This caps that " +
             "one number, so a long stunt costs a slightly longer recentre instead of a snap.\n" +
             "Raise it if the camera now feels like it trails you home after a flip; lower it if " +
             "the release still snaps. Under about 100 it starts biting during ordinary corners, " +
             "which reads as the camera lagging the turn.")]
    [Range(60f, 720f)]
    public float headingCatchUpMaxRate = 180f;

    [Tooltip("How far the stabilized heading may drift from the craft's DIRECTION OF TRAVEL while " +
             "air control is held, in degrees.\n" +
             "Air control pins the heading so the stunt gets a still frame, but nothing bounded how " +
             "far it could drift, and a flip sweeps the nose through vertical so it fills fast: " +
             "measured up to 126 degrees, which then costs 0.6s of camera pan to unwind however the " +
             "rate is tuned. Bounding against travel rather than the nose is what makes this safe, " +
             "since a craft on a ballistic arc keeps its heading while the chassis tumbles freely.\n" +
             "Lower it if landing from a flip still swings the camera; raise it if the camera now " +
             "creeps around during a long stunt instead of holding still. At 360 the bound is off " +
             "and you get the old freeze back.")]
    [Range(15f, 360f)]
    public float headingMaxTravelDivergence = 40f;

    [Tooltip("Horizontal speed below which the travel heading stops being REFRESHED, in m/s. The " +
             "bound keeps applying against the last good heading rather than switching off.\n" +
             "That distinction is the whole point, and it is why this number matters far less than " +
             "it looks. When this same 8 meant 'stop APPLYING', it disabled the bound exactly where " +
             "it was needed: a flip bleeds off horizontal speed, so the craft finishes one falling " +
             "almost straight down (measured 5 m/s horizontal against 60 of descent) and the camera " +
             "whipped on landing. Latching fixed that, not the threshold. A falling craft keeps the " +
             "heading it had, so the latched value stays correct however low the speed goes.\n" +
             "Left at 8 because that is the value the fix was verified and play-tested at. Lower it " +
             "only if the camera holds a stale heading through a slow, curving flight; raise it only " +
             "if it picks one up from near-stationary drift.")]
    [Range(0.5f, 30f)]
    public float travelHeadingMinSpeed = 8f;
}
