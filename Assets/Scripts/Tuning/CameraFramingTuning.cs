using System;
using UnityEngine;

/// <summary>
/// Where the drive camera SITS: orbit elevation and shoulder shift. Pure
/// designer-facing values. No scene refs, no runtime state.
///
/// Elevation is expressed as a RANGE AROUND the neutral, not as an absolute
/// angle. The neutral is derived from Drive Follow Offset below, so that one
/// vector defines resting framing and the two range numbers only say how far
/// the stick may travel from it.
/// </summary>
[Serializable]
public class CameraFramingTuning
{
    // -------------------------------------------------------------------------
    // 📐 Resting Framing
    // -------------------------------------------------------------------------
    [Header("📐 Resting Framing")]
    [Tooltip("Where the drive camera sits at rest, relative to the craft. THIS is the authoring " +
             "surface: VCam_Drive's own Follow Offset field is an OUTPUT, overwritten every frame " +
             "by the solver, so editing it during play does nothing. Length in the YZ plane is the " +
             "orbit radius and its angle is the neutral elevation, both honoured exactly. " +
             "(0, 4.5, -8.5) = 9.62m at 27.9 degrees. Safe to drag during play.")]
    public Vector3 driveFollowOffset = new Vector3(0f, 4.5f, -8.5f);

    [Tooltip("Base look point, in the LookAt target's local space. Same story as Drive Follow " +
             "Offset: this wins over the composer's Tracked Object Offset, which is an output. " +
             "Raising it tips the camera toward the horizon at rest. Safe to drag during play.")]
    public Vector3 driveTargetOffset = new Vector3(0f, 1.5f, 0f);

    [Tooltip("Drive-mode field of view. Strafe derives from this minus the strafe section's FOV " +
             "Reduction, and boost adds on top of whichever is in force, so this one number moves " +
             "all three. Both vcam Lens fields are outputs. Safe to drag during play.")]
    [Range(30f, 100f)]
    public float baseFov = 65f;

    // -------------------------------------------------------------------------
    // 🎥 Pitch Orbit
    // -------------------------------------------------------------------------
    [Header("🎥 Pitch Orbit")]
    [Tooltip("Degrees per second of orbit elevation at full right-stick Y deflection. " +
             "0.8 gives roughly 48 deg/sec, which is the shipped feel.")]
    [Range(0.1f, 5f)]
    public float pitchSensitivity = 0.8f;

    [Tooltip("How far BELOW the authored neutral elevation the camera may drop, in degrees. " +
             "The authored offset (0, 4.5, -8.5) sits at 27.9 deg, so 18 here reproduces the " +
             "old absolute floor of 10 deg.")]
    [Range(0f, 45f)]
    public float pitchDownRange = 18f;

    [Tooltip("How far ABOVE the authored neutral elevation the camera may rise, in degrees. " +
             "The old absolute ceiling of 45 deg corresponds to 17 here; the default is " +
             "deliberately higher because the playtest complaint was not being able to see " +
             "far enough ahead on a climb.")]
    [Range(0f, 60f)]
    public float pitchUpRange = 25f;

    [Tooltip("How fast elevation springs back to the authored neutral once the stick is released.")]
    [Range(0.5f, 10f)]
    public float pitchRecenterSpeed = 2f;

    [Tooltip("Stick deflection below which camera pitch input is ignored. Was hardcoded at 0.05.")]
    [Range(0f, 0.4f)]
    public float lookDeadzone = 0.05f;

    [Tooltip("How much a hard turn suppresses camera pitch input. 0 is the old behaviour, 1 locks " +
             "pitch out completely at full steering lock.\n\n" +
             "This is thumb geometry, not a tuning preference. Right stick X is your yaw and right " +
             "stick Y is camera pitch, so holding a drift line means holding X hard over, and no " +
             "thumb pushes a stick perfectly horizontally. The vertical bleed comes along for the " +
             "ride and swings the camera while you are trying to hold a line.\n\n" +
             "Scaled on TURN magnitude rather than on drift, deliberately: the same bleed happens " +
             "in ordinary hard cornering, and scoping it to drift would leave it there.")]
    [Range(0f, 1f)]
    public float turnBleedSuppression = 0.8f;

    // -------------------------------------------------------------------------
    // 🖼 Framing Guard
    // -------------------------------------------------------------------------
    [Header("🖼 Framing Guard")]
    [Tooltip("Degrees of frame the craft must keep between itself and the edge. The guard runs " +
             "last and scales look-ahead back until this is satisfied. 0 disables it.\n\n" +
             "Why this cannot be solved by tuning: pitch look-ahead and speed look-ahead both tip " +
             "the aim, independently, and neither knows the other exists. Any pair of values that " +
             "fits at rest can be overrun by adding the other term, which is exactly what happens " +
             "when you throttle at full stick-up. Tuning buys 'usually in frame'; only a limit " +
             "buys 'always'.\n\n" +
             "The trade: Pitch Look Ahead and Speed Look Ahead Max become MAXIMA rather than " +
             "fixed amounts, since you get as much as the frame can afford. The inspector readout " +
             "reports when the guard is active and by how much, so it never silently overrides " +
             "your values.\n\n" +
             "It only ever scales the look-ahead. Resting framing, orbit radius and FOV are never " +
             "touched, so if the base framing itself does not fit, the guard will not rescue it " +
             "and the readout will say so.")]
    [Range(0f, 15f)]
    public float minFrameMargin = 2f;

    [Tooltip("Metres added to the orbit radius as the stick raises the camera, reaching this much " +
             "at full up-deflection. Zero is a pure arc: rising also swings the camera CLOSER and " +
             "more nearly overhead, which is what pushes the craft off the bottom of the frame.\n\n" +
             "Why this is a third lever and not a luxury: the craft sits at the centre of the orbit, " +
             "so the angle from camera down to craft is always exactly the elevation angle, and " +
             "distance cannot change it. The only budget is 'elevation minus how far the look axis " +
             "tips up must stay under half the vertical FOV'. Pitch ceiling and look-ahead compete " +
             "for it directly. Booming out buys frame back without surrendering either.\n\n" +
             "It is a weak lever, so expect to combine it with the other two rather than solve the " +
             "problem here alone. Applies to UP deflection only; looking down keeps the base radius.")]
    [Min(0f)]
    public float pitchUpDistanceGain = 0f;

    // -------------------------------------------------------------------------
    // 🏎 Shoulder Shift
    // -------------------------------------------------------------------------
    [Header("🏎 Shoulder Shift")]
    [Tooltip("Lateral camera offset at full drift, toward the outside shoulder. " +
             "Positive = right shoulder offset when turning left.")]
    public float shoulderShiftAmount = 1.8f;

    [Tooltip("Forward look-ahead added at full drift, on top of the speed-driven look-ahead. " +
             "Drift is an aiming tool, so this is framing the thing the player is buying.")]
    public float shoulderLookAheadAmount = 2.5f;

    [Tooltip("How fast the shoulder offset eases in and out. Matched to Propulsion's " +
             "drift blend for consistency.")]
    [Range(1f, 15f)]
    public float shoulderShiftLerpSpeed = 8f;
}
