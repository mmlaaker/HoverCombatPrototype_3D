using System;
using UnityEngine;

/// <summary>
/// What the camera does under boost. Pure designer-facing values. No scene
/// refs, no runtime state.
///
/// Every term here is scaled by Propulsion.BoostLerp, so all of it inherits
/// boostBlendSeconds from the vehicle tuning profile and arrives on the same
/// curve as the thrust. Shortening that ramp sharpens the camera with it, and
/// the two can never disagree about when boost started.
///
/// Why this is not a third virtual camera: a vcam switch routes through the
/// CinemachineBrain, which would stack a brain blend on top of
/// boostBlendSeconds and reinstate exactly the lag the ramp was shortened to
/// remove. Boost is also orthogonal to drive/strafe (it applies in either) and
/// continuous rather than discrete, so it is a value to multiply, not a camera
/// to cut to.
/// </summary>
[Serializable]
public class CameraBoostTuning
{
    // -------------------------------------------------------------------------
    // 🚀 Field of View
    // -------------------------------------------------------------------------
    [Header("🚀 Field of View")]
    [Tooltip("Degrees of FOV added at full boost, on top of whichever mode's base FOV is in " +
             "force. Composed against the strafe base as well as the drive base, so boosting " +
             "in strafe widens from the strafe zoom rather than fighting it.")]
    [Range(0f, 25f)]
    public float fovIncrease = 5f;

    [Tooltip("Forward speed (m/s) at which the FOV kick reaches full strength. Below it the kick " +
             "fades out, and in REVERSE it is zero.\n\n" +
             "This exists because boost genuinely engages in reverse (Propulsion v1.2 made that " +
             "work on purpose), and the lens was widening for it. Widening FOV is a forward-rush " +
             "cue; there is no rush to sell when you are backing up, so it just read as the camera " +
             "doing something arbitrary.\n\n" +
             "Keep this LOW. It gates on direction, not on how fast you are going: scaling the " +
             "kick by speed would delay it at boost onset, and the kick works precisely because " +
             "it is a transient arriving on the same curve as the thrust. At the shipped " +
             "acceleration, 2 m/s is reached in about a thirtieth of a second.")]
    [Range(0.5f, 15f)]
    public float forwardGateSpeed = 2f;
}
