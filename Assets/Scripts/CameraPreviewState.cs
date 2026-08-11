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
    Inverted
}
