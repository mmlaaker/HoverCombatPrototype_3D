using UnityEngine;

/// <summary>
/// HoverController_Tricks v1.1
///
/// Pays energy for rotations landed in the air. The other half of the energy
/// economy: the pool refills by waiting for regen OR by landing tricks, and the
/// second one is a decision rather than a payout, because going for more rotation
/// costs the airtime you wanted for setting up the landing and blowing the landing
/// forfeits everything banked.
///
/// Owns no physics and writes nothing to the Rigidbody. It reads state that
/// Foundation and Propulsion already publish and pays the Energy pool. That is why
/// this exists as its own component rather than as a block inside Foundation:
/// Foundation is marked do-not-modify-without-justification, and this needed
/// nothing from it that was not already public.
///
/// WHAT COUNTS
///
///   Armed on the rising edge of air control, once per flight. After that the
///   whole flight counts, including rotation that continues after the button is
///   released. A tap jump therefore banks nothing, because it never clears
///   airControlMinClearance and never earns authority -- which is the confirmed
///   design, not a side effect. An EMP tumble or a rocket hit pays nothing either,
///   because neither arms air control.
///
///   PITCH and ROLL only. Yaw is excluded: a flat spin is neither trick, and
///   paying for it would make sitting on the stick an income.
///
///   Rotation rate is the MAGNITUDE of the pitch/roll pair, not their sum. This
///   is the whole reason a corkscrew needs no special case, and getting it wrong
///   is not a rounding error: summing components pays a diagonal rotation about
///   1.41x for the same actual rotation, purely because two components of one
///   vector were added as though they were separate rotations. Compare
///   measurement trap 19, which is the same mistake in the linear axes.
///
///   Body frame is correct HERE, and that is worth stating because trap 19 warns
///   against exactly this substitution. A flip and a roll are defined by the
///   craft's own axes; asking which world axis it turned about would give a
///   different answer for the same trick depending on which way the player
///   happened to be facing.
///
/// WHAT IT PAYS
///
///   Two independent ledgers, rolls and flips, each priced by naming the event
///   rather than by a curve: the first one pays one price and every one after it
///   pays another. Flips are priced higher because a flip costs roughly twice the
///   airtime of a roll, so pricing them equally would make rolls strictly the
///   better income and turn flips into decoration.
///
///   Both ledgers are continuous. A partial first rotation pays pro rata, so a
///   trick that lands at 1.8 rolls is not silently worth the same as one that
///   lands at 1.0, and nothing has to be rounded or snapped.
///
///   Every tick's rotation is split between the two ledgers by how much of it is
///   nose-over-tail, so a corkscrew feeds both. On top of that split it is scaled
///   by trickCorkscrewMultiplier, on a SLOPE rather than a cliff: a nearly clean
///   roll is barely touched and the full effect lands only on a rotation split
///   evenly between the axes. A threshold would have produced a visible edge where
///   a slightly untidy roll suddenly paid a different price.
///
/// LANDING IT (v1.1)
///
///   Two gates, and the first one is the reason this version exists.
///
///   ARRIVAL. The craft's attitude is sampled the moment it reaches the ground,
///   decomposed into bank and nose-over-tail against the SURFACE NORMAL rather
///   than against world up. Either angle past its limit forfeits immediately, no
///   matter how tidily the craft settles afterwards. Measuring against the surface
///   is what makes a clean landing on a hillside read as clean; world up would
///   charge the slope angle against the player on exactly the terrain where the
///   long airtime is.
///
///   SETTLE. Having arrived cleanly, the craft must still be upright and out of
///   recovery when trickSettleSeconds expires. A tumble usually latches IsDowned a
///   fraction of a second AFTER first contact, so paying on the contact frame
///   would pay for crashes. Compare measurement trap 2: measure the state flag,
///   not "it looked level for a moment".
///
///   ARRIVAL NEEDS TWO SIGNALS, and this is the non-obvious part. Hover support
///   and physical contact are each blind in exactly the case the other one
///   catches:
///
///     A clean landing produces NO collision at all. Measured on a 70 m/s drop:
///     the hard landing event fires and there are zero collision callbacks,
///     because the springs catch the chassis before the collider reaches the
///     ground.
///
///     A craft on its flank produces NO hover support. The rays fire along the
///     craft's own down axis, so on its side they point sideways and find
///     nothing; it reads as airborne while physically lying on the floor.
///
///   So arrival is whichever lands first, and the surface normal comes from the
///   contact when there was one and from the hover rays otherwise. Watching only
///   support, as v1.0 did, meant a side landing never resolved AT ALL: it stayed
///   pending until flip recovery righted the craft and then paid out for a clean
///   landing it never made.
///
///   Wall scrapes are filtered out by contact normal, so clipping a wall mid-trick
///   does not count as arriving. Hard landings are explicitly fine: impact speed is
///   never consulted, because the rule is that you may land as hard as you like as
///   long as you land on your belly.
///
///   Bouncing back into the air cancels the settle window instead of forfeiting,
///   and clears the arrival sample once the craft is properly clear of the ground,
///   so a bounce into another rotation is the same trick continuing and the
///   landing that gets judged is the one it finishes on.
///
/// DELIBERATELY ABSENT: an OnTrickBanked event. Nothing consumes one yet, and
/// three events were deleted from this project for being raised with no
/// subscriber. Add it when a HUD or VFX consumer actually exists.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class HoverController_Tricks : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // 📦 Tuning
    // -------------------------------------------------------------------------
    [Header("📦 Tuning")]
    [Tooltip("Vehicle tuning profile (shared SO). All numeric tuning lives here. Required.")]
    [SerializeField] private VehicleTuningProfile profile;

    /// <summary>Shorthand for profile.energy. Used at every read site below.</summary>
    private EnergyTuning E => profile.energy;

    // -------------------------------------------------------------------------
    // 🧭 Debug
    // -------------------------------------------------------------------------
    [Header("🧭 Debug")]
    [SerializeField] private bool drawDebug = false;

    [Tooltip("Optional global debug toggle. When assigned, overrides Draw Debug.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.IsEnabled(HoverDebugCategory.Resources) : drawDebug;

    // -------------------------------------------------------------------------
    // Thresholds that are not tuning
    // -------------------------------------------------------------------------

    /// <summary>
    /// HoverSupport at or below which the craft counts as airborne, and at or above
    /// which it counts as landed. Two values rather than one because a single
    /// threshold chatters: support dips on every bump crest, and a trick that
    /// banked and re-armed several times crossing rough ground would pay several
    /// times for one flight.
    ///
    /// Not exposed as tuning. They are not a feel decision, they are the width of
    /// a deadband around a signal Foundation already defines, and a knob here would
    /// invite tuning the accounting instead of the game.
    /// </summary>
    private const float AirborneSupport = 0.01f;
    private const float LandedSupport   = 0.5f;

    /// <summary>
    /// Minimum upward component of an averaged contact normal for that contact to
    /// count as reaching the GROUND rather than scraping a wall. Roughly 60 degrees
    /// off vertical, matching the spirit of Foundation's own floor filter.
    /// </summary>
    private const float FloorContactNormalY = 0.5f;

    /// <summary>
    /// How long the craft must be clear of its springs before a new arrival will be
    /// sampled. Without it the sample would clear on the same frame it was taken,
    /// since arrival happens while support is still far below the landed threshold.
    /// </summary>
    private const float ArrivalResetSeconds = 0.2f;

    // -------------------------------------------------------------------------
    // Runtime
    // -------------------------------------------------------------------------
    private HoverController_Foundation foundation;
    private HoverController_Propulsion propulsion;
    private HoverController_Energy     energy;
    private Rigidbody                  rb;

    /// <summary>Air control has engaged during the current flight, so rotation counts.</summary>
    private bool _armed;

    /// <summary>Barrel roll rotations accumulated this flight.</summary>
    private float _rollRotations;

    /// <summary>Flip rotations accumulated this flight.</summary>
    private float _flipRotations;

    /// <summary>Everything turned this flight, for the minimum-trick test and the readout.</summary>
    private float TotalRotations => _rollRotations + _flipRotations;

    /// <summary>Hysteresis state for the airborne/landed test. See AirborneSupport.</summary>
    private bool _grounded = true;

    /// <summary>Attitude has been sampled for this touchdown.</summary>
    private bool _arrived;

    /// <summary>Bank and nose-over-tail angle at the sampled arrival, against the surface.</summary>
    private float _arrivalRoll;
    private float _arrivalFlip;

    /// <summary>Seconds the craft has been clear of its springs, for clearing the arrival sample.</summary>
    private float _airborneClearTimer;

    /// <summary>A floor contact arrived since the last tick, with its averaged normal.</summary>
    private bool    _contactPending;
    private Vector3 _contactNormal;

    /// <summary>True while a landed bank is waiting out trickSettleSeconds before it pays.</summary>
    private bool _settling;

    /// <summary>Seconds left on the settle window.</summary>
    private float _settleTimer;

    // Debug-only record of the last resolved trick, so the gizmo can report an
    // outcome that has already happened. Never read by gameplay.
    private float  _dbgLastPayout;
    private float  _dbgLastGranted;
    private float  _dbgLastRotations;
    private bool   _dbgLastWasBanked;
    private string _dbgLastReason = "";
    private float  _dbgLastResolveTime = float.NegativeInfinity;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------
    private void Awake()
    {
        rb         = GetComponent<Rigidbody>();
        foundation = GetComponent<HoverController_Foundation>();
        propulsion = GetComponent<HoverController_Propulsion>();
        energy     = GetComponent<HoverController_Energy>();

        if (profile == null)
        {
            Debug.LogError(
                $"[Tricks] '{name}': VehicleTuningProfile is not assigned. " +
                $"Assign one in the inspector. Trick payouts disabled.",
                this
            );
            enabled = false;
            return;
        }

        if (foundation == null || propulsion == null || energy == null)
        {
            Debug.LogError(
                $"[Tricks] '{name}': needs Foundation, Propulsion and Energy on the same GameObject. " +
                $"Trick payouts disabled.",
                this
            );
            enabled = false;
        }
    }

    /// <summary>
    /// The physical half of arrival detection. This component sits on the vehicle
    /// root deliberately: Unity delivers collision callbacks only to the GameObject
    /// owning the Rigidbody, never to children, so a component that needs impacts
    /// has to live here. Foundation's OnCollisionStay and VehicleCollisionRelay's
    /// own Enter on this same object are independent of this one.
    /// </summary>
    private void OnCollisionEnter(Collision collision)
    {
        if (!_armed || _arrived)
            return;

        int count = collision.contactCount;
        if (count == 0)
            return;

        Vector3 sum = Vector3.zero;
        for (int i = 0; i < count; i++)
            sum += collision.GetContact(i).normal;

        Vector3 averaged = sum / count;

        // A wall clipped mid-trick is not a landing. Averaging first is deliberate:
        // a single glancing contact in a mostly-floor impact should not disqualify it.
        if (averaged.y < FloorContactNormalY)
            return;

        _contactPending = true;
        _contactNormal  = averaged;
    }

    // -------------------------------------------------------------------------
    // Tick
    // -------------------------------------------------------------------------

    /// <summary>
    /// FixedUpdate rather than Update because this integrates a Rigidbody angular
    /// velocity. At frame rate the integral would depend on frame rate, which is
    /// the same defect EmpProjectile v1.2 and RocketProjectile v1.6 were fixed for.
    /// </summary>
    private void FixedUpdate()
    {
        float dt      = Time.fixedDeltaTime;
        float support = foundation.HoverSupport;

        // Going down forfeits everything, at any point: mid-flight, on contact, or
        // during the settle window. Checked first so nothing below can pay out a
        // craft that is on its back.
        if (foundation.IsDowned)
        {
            if (_armed || _settling)
                Resolve(false, "downed");

            _grounded       = support >= LandedSupport;
            _contactPending = false;
            return;
        }

        bool grounded = _grounded
            ? support > AirborneSupport
            : support >= LandedSupport;

        // Clear the arrival sample only once the craft is properly off its springs,
        // so a bounce is judged on the landing it finishes with rather than the
        // graze it started with.
        if (support <= AirborneSupport)
        {
            _airborneClearTimer += dt;
            if (_airborneClearTimer >= ArrivalResetSeconds)
                _arrived = false;
        }
        else
        {
            _airborneClearTimer = 0f;
        }

        // Arrival: whichever signal lands first. See the class summary for why one
        // signal is never enough.
        if (_armed && !_arrived && (_contactPending || support > AirborneSupport))
        {
            SampleArrival(_contactPending ? _contactNormal : foundation.AverageGroundNormal);
        }

        _contactPending = false;

        if (!grounded)
            TickAirborne(dt);
        else
            TickGrounded(dt);

        _grounded = grounded;
    }

    /// <summary>
    /// Arms on the first frame of air control this flight, then accumulates. Also
    /// cancels a settle window, because being airborne again means the landing that
    /// opened it did not stick: the trick is still in progress.
    /// </summary>
    private void TickAirborne(float dt)
    {
        _settling = false;

        if (propulsion.AirControlWeight > 0f)
            _armed = true;

        if (!_armed)
            return;

        // Body frame is the correct frame here; see the class summary.
        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);

        float pitchRate = Mathf.Abs(localAngVel.x);
        float rollRate  = Mathf.Abs(localAngVel.z);

        // MAGNITUDE, not the sum. Summing would pay a diagonal rotation about 1.41x
        // for the same actual rotation.
        float rate = Mathf.Sqrt(pitchRate * pitchRate + rollRate * rollRate);
        if (rate <= 0f)
            return;

        // rate > 0 guarantees pitchRate + rollRate > 0, so this cannot divide by zero.
        // 0 is a clean barrel roll, 1 is a clean flip, 0.5 is an even corkscrew.
        float pitchShare = pitchRate / (pitchRate + rollRate);

        // Peaks at 1 when the rotation is split evenly and falls to 0 at either
        // clean axis, so the corkscrew multiplier arrives as a slope.
        float mix = 1f - Mathf.Abs(2f * pitchShare - 1f);

        float rotations = rate * dt / (2f * Mathf.PI)
                        * Mathf.Lerp(1f, E.trickCorkscrewMultiplier, mix);

        _flipRotations += rotations * pitchShare;
        _rollRotations += rotations * (1f - pitchShare);
    }

    /// <summary>
    /// Opens the settle window once the springs are properly carrying the craft,
    /// then pays when it expires. Nothing is paid on contact itself.
    /// </summary>
    private void TickGrounded(float dt)
    {
        if (_settling)
        {
            _settleTimer -= dt;
            if (_settleTimer <= 0f)
                Resolve(true, "landed");
            return;
        }

        if (!_armed)
            return;

        if (TotalRotations < E.trickMinRotations)
        {
            // Landed, but nothing that reads as a trick. Cleared rather than banked,
            // so a long straight jump cannot quietly accumulate wobble into a payout.
            Resolve(false, "under minimum");
            return;
        }

        _settling    = true;
        _settleTimer = E.trickSettleSeconds;
    }

    // -------------------------------------------------------------------------
    // Arrival
    // -------------------------------------------------------------------------

    /// <summary>
    /// Records how square the craft was to the ground at the moment it reached it,
    /// and forfeits immediately if either axis is outside its limit.
    /// </summary>
    private void SampleArrival(Vector3 surfaceNormal)
    {
        _arrived = true;

        Vector3 normal = surfaceNormal.sqrMagnitude > 0.0001f
            ? surfaceNormal.normalized
            : Vector3.up;

        _arrivalRoll = TiltAbout(transform.up, normal, transform.forward);
        _arrivalFlip = TiltAbout(transform.up, normal, transform.right);

        bool tooBanked = Mathf.Abs(_arrivalRoll) > E.trickMaxLandingRollAngle;
        bool tooPitched = Mathf.Abs(_arrivalFlip) > E.trickMaxLandingFlipAngle;

        if (tooBanked || tooPitched)
            Resolve(false, tooBanked ? "landed banked" : "landed nose-first");
    }

    /// <summary>
    /// How far the craft's up axis leans away from the surface normal, measured
    /// about one body axis. Signed, and correct all the way through inversion:
    /// a craft upside down reads 180 rather than folding back to 0, which an
    /// arcsine of a single component would do.
    ///
    /// Returns 0 when the decomposition is degenerate, which happens only when the
    /// OTHER axis is already at 90 degrees and has therefore failed its own gate.
    /// </summary>
    private static float TiltAbout(Vector3 craftUp, Vector3 normal, Vector3 axis)
    {
        Vector3 projected = Vector3.ProjectOnPlane(normal, axis);

        if (projected.sqrMagnitude < 0.0001f)
            return 0f;

        return Vector3.SignedAngle(craftUp, projected, axis);
    }

    // -------------------------------------------------------------------------
    // Payout
    // -------------------------------------------------------------------------

    /// <summary>
    /// One ledger's worth: the first rotation at its own price, everything after
    /// it at the repeat price, with a partial first rotation paying pro rata.
    /// </summary>
    private static float Ledger(float rotations, float first, float repeat)
    {
        if (rotations <= 0f)
            return 0f;

        if (rotations <= 1f)
            return first * rotations;

        return first + repeat * (rotations - 1f);
    }

    /// <summary>
    /// Energy for a given pair of rotation counts. Public and static so an
    /// inspector readout and the runtime cannot disagree about what a trick is
    /// worth: one formula, both callers.
    /// </summary>
    public static float PayoutFor(float rollRotations, float flipRotations, EnergyTuning tuning)
    {
        return Ledger(rollRotations, tuning.trickBarrelRollEnergy, tuning.trickBarrelRollRepeatEnergy)
             + Ledger(flipRotations, tuning.trickFlipEnergy,       tuning.trickFlipRepeatEnergy);
    }

    /// <summary>Payout the current flight would earn if it landed cleanly right now.</summary>
    private float PendingPayout()
    {
        if (TotalRotations < E.trickMinRotations)
            return 0f;

        return PayoutFor(_rollRotations, _flipRotations, E);
    }

    /// <summary>
    /// Ends the current flight, paying out or forfeiting, and resets everything so
    /// the next flight starts clean. Every exit path goes through here so there is
    /// exactly one place that can leave state behind.
    /// </summary>
    private void Resolve(bool banked, string reason)
    {
        float payout = banked ? PendingPayout() : 0f;

        // What it EARNED and what FITTED are different numbers, and the readout must
        // not confuse them: landing a trick on a full pool grants nothing, and
        // reporting that as a lost trick would blame the player for a good landing.
        float granted = payout > 0f ? energy.Grant(payout) : 0f;

        _dbgLastPayout      = payout;
        _dbgLastGranted     = granted;
        _dbgLastRotations   = TotalRotations;
        _dbgLastWasBanked   = banked;
        _dbgLastReason      = reason;
        _dbgLastResolveTime = Time.unscaledTime;

        _armed         = false;
        _rollRotations = 0f;
        _flipRotations = 0f;
        _settling      = false;
        _settleTimer   = 0f;
    }

    // -------------------------------------------------------------------------
    // 🎨 Debug Gizmos
    // -------------------------------------------------------------------------
#if UNITY_EDITOR
    /// <summary>
    /// Sits above the Energy label so the payout can be read against the pool it is
    /// paying into. Without this the whole feature is unjudgeable without a CSV,
    /// which is how it would end up tuned by guess.
    /// </summary>
    private void OnDrawGizmos()
    {
        if (!Application.isPlaying)
            return;

        if (!ShouldDrawDebug)
            return;

        if (profile == null)
            return;

        string label;
        Color  color;

        // Roll and flip are shown separately because they are priced separately:
        // a single combined number cannot tell you why a trick paid what it did.
        string split = $"roll {_rollRotations:F2}  flip {_flipRotations:F2}";

        if (_settling)
        {
            label = $"TRICK LANDING {split} -> {PendingPayout():F0}  hold {_settleTimer:F2}s\n"
                  + $"  arrived bank {_arrivalRoll:F0}deg  nose {_arrivalFlip:F0}deg";
            color = Color.yellow;
        }
        else if (_armed)
        {
            float pending = PendingPayout();
            label = pending > 0f
                ? $"TRICK {split} -> {pending:F0}"
                : $"TRICK {split} (under minimum)";
            color = Color.cyan;
        }
        else if (Time.unscaledTime - _dbgLastResolveTime < 2f)
        {
            if (_dbgLastWasBanked)
            {
                // Naming the shortfall matters: "earned 25, got 4" reads as a full
                // pool, where a bare "+4" reads as the payout being wrong.
                label = _dbgLastGranted < _dbgLastPayout - 0.01f
                    ? $"TRICK BANKED +{_dbgLastGranted:F0} of {_dbgLastPayout:F0}  (pool full)"
                    : $"TRICK BANKED +{_dbgLastGranted:F0}  ({_dbgLastRotations:F2} rot)";
                color = Color.green;
            }
            else
            {
                label = $"TRICK LOST: {_dbgLastReason}  ({_dbgLastRotations:F2} rot)";
                color = Color.red;
            }
        }
        else
        {
            return;
        }

        UnityEditor.Handles.color = color;
        UnityEditor.Handles.Label(transform.position + Vector3.up * 3.5f, label);
    }
#endif
}
