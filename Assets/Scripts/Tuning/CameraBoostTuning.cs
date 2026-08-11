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

    [Tooltip("Extra degrees of FOV at the PEAK of the engage transient, on top of FOV Increase. " +
             "This is the part that spikes and settles rather than the part that stays.\n\n" +
             "Why a transient is worth more than a bigger sustained value: a wider lens is a " +
             "sustained cue, and the eye adapts to those within about a second, after which you " +
             "are simply driving with an odd lens. What reads as speed is CHANGE. So the spike " +
             "sells the moment you commit, and FOV Increase is only what remains afterwards.")]
    [Range(0f, 25f)]
    public float fovOvershoot = 6f;

    // -------------------------------------------------------------------------
    // 🏃 Pull-back
    // -------------------------------------------------------------------------
    [Header("🏃 Pull-back")]
    [Tooltip("Metres the camera falls back while boost is held. Applied along Z, straight behind " +
             "the craft, rather than along the orbit radius: that makes the view both further out " +
             "AND slightly flatter, which shows more horizon and more ground streaming past. " +
             "Booming along the radius would hold the angle and only add distance.\n\n" +
             "DRIVE ONLY. Strafe gets the lens change and nothing else, because the strafe " +
             "crosshair is yaw and pitch and moving the rig moves the player's aim.")]
    [Min(0f)]
    public float zPullBack = 1f;

    [Tooltip("Extra metres of pull-back at the PEAK of the engage transient, on top of Z Pull-back. " +
             "This is the camera being left behind at the moment you commit, and it is the most " +
             "direct 'something just happened' signal available, because it is the craft visibly " +
             "getting away from you rather than a lens trick.")]
    [Min(0f)]
    public float zLagOnEngage = 1.5f;

    // -------------------------------------------------------------------------
    // ⏱ Envelope
    // -------------------------------------------------------------------------
    [Header("⏱ Envelope")]
    [Tooltip("How fast the sustained terms fall away once boost is released. Lerp speed, so 2.5 is " +
             "roughly a 0.4s settle.\n\n" +
             "The ENTRY rate is deliberately not a field here: it comes from the vehicle profile's " +
             "Boost Blend Seconds, so the camera and the thrust can never disagree about when boost " +
             "started. This number only controls the way out, and it should be SLOWER than the way " +
             "in. Snapping back is a second event competing with the first, and the release is not " +
             "the moment worth dramatising.")]
    [Range(0.5f, 12f)]
    public float releaseSpeed = 2.5f;

    [Tooltip("How fast the engage transient decays while boost is still held. Lerp speed, so 3.5 is " +
             "roughly a 0.3s settle.\n\n" +
             "The transient is measured as the gap between the boost level and a slower copy of " +
             "itself, which means it costs no timers and cancels itself: once the slow copy catches " +
             "up, the gap is zero and the spike is over. It also scales with how SNAPPY the thrust " +
             "ramp is, since a faster ramp opens a wider gap. Sharpen Boost Blend Seconds and the " +
             "camera kick sharpens with it, for free.")]
    [Range(0.5f, 12f)]
    public float settleSpeed = 3.5f;
}
