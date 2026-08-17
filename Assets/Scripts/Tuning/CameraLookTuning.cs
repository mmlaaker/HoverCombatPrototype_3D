using System;
using UnityEngine;

/// <summary>
/// Where the drive camera LOOKS, as distinct from where it sits. Pure
/// designer-facing values. No scene refs, no runtime state.
///
/// This section exists because camera height and look direction used to be the
/// same knob. Raising the orbit angled the camera DOWN at the roof, which shows
/// more ground beside the craft and less horizon, the opposite of what a player
/// climbing a slope is asking for. Splitting them is what lets the stick mean
/// "show me further ahead" instead of "get higher".
///
/// Automatic slope compensation was considered and declined by the owner: the
/// camera never reacts to terrain on its own, so every term here is driven by
/// speed or by deliberate stick input.
/// </summary>
[Serializable]
public class CameraLookTuning
{
    // -------------------------------------------------------------------------
    // 🔭 Speed Look-Ahead
    // -------------------------------------------------------------------------
    [Header("🔭 Speed Look-Ahead")]
    [Tooltip("Metres the look point is pushed ahead of the craft at the reference speed. " +
             "Partly compensates for perceived speed being structurally low on this chassis: " +
             "60 m/s over a 6.88m hull is 8.7 body-lengths per second, against 15-16 for a " +
             "fast road car.")]
    [Min(0f)]
    public float speedLookAheadMax = 6f;

    [Tooltip("Forward speed (m/s) at which Speed Look Ahead Max is fully applied. " +
             "Set to the vehicle's top speed so the term saturates exactly at the cap.")]
    [Min(1f)]
    public float speedLookAheadReference = 60f;

    [Tooltip("Ceiling on how fast the look point may travel, in metres per second. This limits " +
             "the FRAMING, not the craft: the look point is what the camera aims at, so this " +
             "caps how fast the aim can slide forward off the nose.\n\n" +
             "It exists because boosting from a standstill was the only case that got the whole " +
             "look-ahead swing AND the boost lens-and-pull-back inside the same third of a " +
             "second. Each of those alone was already judged good; together they were not.\n\n" +
             "Measured 2026-08-17 over 50ms windows: flooring it from rest peaks at 7.7 m/s of " +
             "look-point travel and reads fine, boosting from rest peaks at 11.5 and reads as a " +
             "jolt. So 8 sits deliberately just above the case already accepted, which makes it " +
             "a no-op there and a clip only on the boost case. At cruise it does nothing at all, " +
             "because the term is saturated and the look point is not moving.\n\n" +
             "It is symmetric, so it also stops the look point snapping back 6m in one frame when " +
             "a collision takes the craft from top speed to zero.\n\n" +
             "Raise toward 12 to hand the jolt back. Set to 0 to remove the limit entirely and " +
             "restore the pre-fix behaviour.")]
    [Min(0f)]
    public float speedLookAheadSlew = 8f;

    // -------------------------------------------------------------------------
    // 🕹 Stick Coupling
    // -------------------------------------------------------------------------
    [Header("🕹 Stick Coupling")]
    [Tooltip("Metres the look point rises when the stick is pushed to full up-elevation. " +
             "This is the term that tips the camera toward the horizon instead of the roof, " +
             "and with slope compensation declined it is the main answer to 'let me see " +
             "further ahead on a climb'.")]
    [Min(0f)]
    public float pitchLookLift = 2.5f;

    [Tooltip("Metres the look point is pushed forward at full up-elevation. Works with Pitch " +
             "Look Lift; forward moves the aim past the nose, lift gets it over the crest.")]
    [Min(0f)]
    public float pitchLookAhead = 6f;

    // -------------------------------------------------------------------------
    // 🎯 Authority
    // -------------------------------------------------------------------------
    [Header("🎯 Authority")]
    [Tooltip("Look-ahead offsets are expressed in the LookAt target's local space, so they " +
             "rotate with the chassis. Left unchecked they would swing the aim point wildly " +
             "through a flip or a tumble. Enabled, they fade out with air control and with " +
             "how far from upright the craft is, the same way heading tracking already does. " +
             "Disable only to diagnose.")]
    public bool fadeLookAheadWhenInverted = true;
}
