/// <summary>
/// A definitive camera state: what the framing settles to once a given input is
/// fully blended in. Selected on HoverCameraController to preview that state
/// with the game stopped.
///
/// These are DERIVED, not authored. Each case is computed from the camera
/// tuning already on the component, so there is nothing to keep in sync and no
/// way to preview a frame the game never produces. Change a tuning value and
/// every state that reads it moves with it.
///
/// The list is deliberately short. Blends and modifier combinations are not
/// worth enumerating, because the composition model already guarantees that a
/// combination is the sum of its parts: get each terminal state right and the
/// combinations follow. Anything not listed here is reachable by driving.
/// </summary>
public enum CameraPreviewState
{
    /// <summary>Parked, stick centred. The resting pose, and the regression bar.</summary>
    Resting,

    /// <summary>Stick held full up. The state where the craft currently leaves the frame.</summary>
    StickFullUp,

    /// <summary>Stick held full down.</summary>
    StickFullDown,

    /// <summary>Flat out, no boost. Speed look-ahead fully applied.</summary>
    FullThrottle,

    /// <summary>
    /// Flat out with boost engaged and fully SETTLED: the engage transient has
    /// decayed and only the sustained pull-back and lens change remain. This is
    /// what a long boost looks like, and the state to judge for comfort.
    ///
    /// Note the speed term is identical to FullThrottle: speedLookAheadReference
    /// equals top speed and the term clamps at 1, so boost differs only in the
    /// terms it owns.
    /// </summary>
    Boost,

    /// <summary>
    /// The PEAK of the engage transient, roughly a tenth of a second after you
    /// commit. The widest lens and the furthest pull-back the camera ever
    /// reaches.
    ///
    /// A real frame rather than an average of one, which is the reason it earns
    /// a state: it exists for a fraction of a second while driving, so it is the
    /// single hardest thing in the boost to judge by eye, and it is also the
    /// frame doing most of the work. Judge Boost for comfort and this one for
    /// impact. If the craft only leaves frame here and not in Boost, the
    /// overshoot is too large rather than the framing being wrong.
    /// </summary>
    BoostPeak,

    /// <summary>
    /// Climbing at speed: stick full up AND flat out. This is the original
    /// "let me see further ahead on a slope" case, and the worst case for
    /// keeping the craft in frame.
    /// </summary>
    ClimbAtSpeed,

    /// <summary>Fully committed drift to the left, shoulder shift settled.</summary>
    DriftLeft,

    /// <summary>Fully committed drift to the right, shoulder shift settled.</summary>
    DriftRight,

    /// <summary>Strafe mode, aiming.</summary>
    Strafe,

    /// <summary>Strafe mode with boost engaged.</summary>
    StrafeBoost,

    /// <summary>
    /// Inverted and under full air control. Exercises the look-ahead authority
    /// fade, which should collapse both look-ahead terms to zero. If the look
    /// point moves at all in this state, the fade is broken.
    /// </summary>
    Inverted,

    // ── States where the nose and the travel direction DISAGREE ───────────
    //
    // Everything above this line has the craft travelling along its own nose.
    // That was an unstated assumption rather than a decision, and it is the
    // assumption two shipped defects lived under: neither was reproducible in
    // this dropdown, and both cost a driven playtest to find instead.
    //
    // These exist so that class of bug is judgeable with the game stopped.

    /// <summary>
    /// Boosting in REVERSE. Nose forward, travel backward, boost fully held.
    ///
    /// The forward gate should read zero here and kill every boost term, so
    /// this state should look like plain Resting apart from the lens. If it
    /// shows any pull-back or any widening, the gate is broken -- and a gate
    /// that fails open is invisible in every other state, because everywhere
    /// else the gate is meant to be open anyway.
    ///
    /// The speed magnitude is the camera's own reference rather than the
    /// vehicle's reverse cap, which is lower. Nothing here depends on it: the
    /// gate saturates on the SIGN long before the magnitude matters.
    /// </summary>
    ReverseBoost,

    /// <summary>
    /// Travelling at full speed with the nose swept fully across the direction
    /// of travel -- broadside, mid-spin, or the far side of a hard drift.
    ///
    /// This is the pose no previous state could express, and it is the exact
    /// shape of the boost-gate defect: forward speed reads zero while the craft
    /// is genuinely moving at speed. Speed look-ahead should therefore collapse
    /// to nothing while the boost terms stay fully live, because the two read
    /// DIFFERENT speeds. That asymmetry is the thing to judge.
    /// </summary>
    NoseAcrossTravel,

    /// <summary>
    /// Flipped, helpless, and the player has swung the camera to the limit of
    /// its downed range.
    ///
    /// The worst case for the downed orbit: the camera is as far off-axis as it
    /// can get. Judge whether the craft is still readable from there, and
    /// whether the horizon is still level -- the orbit rotates about world up
    /// precisely so that it is, and this is the state that shows it.
    /// </summary>
    Downed
}
