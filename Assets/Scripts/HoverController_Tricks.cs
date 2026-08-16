using UnityEngine;

/// <summary>
/// HoverController_Tricks v1.6
///
/// v1.6: OnTrickResolved reports the payout AND what reached the pool, rather than
/// what reached the pool alone. The readout is how a player learns the price list,
/// and the old signature could only ever show the leftover headroom in the pool --
/// an arbitrary number, because regen leaves the pool at an arbitrary level, so a
/// flip worth 25 landed at 82 energy read as "+18" and the rounding to fives looked
/// broken from the outside. It was not; the number was never the payout.
///
/// v1.5: trickPayoutRounding. A corkscrew-discounted payout is rounded to the
/// nearest increment, and only a discounted one, since a clean trick already lands
/// on a round number and rounding it could only move an authored value. Nearest
/// rather than upward because rounding up erases any discount smaller than one
/// increment, which is most of them: two rolls at a mild penalty are 21.88, and the
/// next increment of five is 25, the undiscounted price exactly.
///
/// v1.4: trickCleanAxisTolerance. The corkscrew discount used to ramp from the
/// instant a rotation was less than perfect, so a flip six degrees off the stick's
/// axis quietly lost a tenth of its payout. See the corkscrew note below.
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
///   Body frame is correct HERE, and that is worth stating because measurement
///   trap 19 warns against exactly this substitution. A flip and a roll are
///   defined by the craft's own axes; asking which world axis it turned about
///   would give a different answer for the same trick depending on which way the
///   player happened to be facing.
///
/// COMPLETED REVOLUTIONS, NOT TOTAL TRAVEL (v1.3)
///
///   A revolution is banked the moment the axis comes all the way round, and
///   nothing afterwards can take it back. Progress toward the next one is SIGNED,
///   so turning back unwinds it.
///
///   v1.2 accumulated the ABSOLUTE rate, which measured how far the craft turned
///   in total and never where it ended up. Measured: half a barrel roll out and
///   straight back again scored 0.989 rotations and paid 9.9 energy, with the
///   craft finishing three degrees from where it started. Wobbling the stick was
///   an income. Direction has to be part of the sum.
///
///   Banking on the CROSSING rather than on net displacement is what makes this
///   safe for real play. Net displacement would have been the obvious fix and it
///   is wrong: over-rotating a flip and pitching back to straighten up for the
///   landing would have eaten the flip you just earned, which charges the player
///   for the technique that makes a landing clean. Once round is once round.
///
///   It also makes the payout and the HUD the same number by construction. v1.2
///   could pay 9.9 while the tracker displayed nothing, because one was continuous
///   and the other floored to whole turns.
///
/// WHAT IT PAYS
///
///   Two independent ledgers, rolls and flips, each priced by naming the event:
///   the first one pays one price and every one after it pays another. Flips are
///   priced higher because a flip costs roughly twice the airtime of a roll, so
///   pricing them equally would make rolls strictly the better income and turn
///   flips into decoration.
///
///   The corkscrew multiplier scales the PAYOUT, not the accrual. It is applied as
///   a flight-level average of how diagonal the rotation was, weighted by how much
///   rotation happened at each moment, so a flight flown clean pays full and one
///   flown entirely on the diagonal pays the multiplier. Scaling the accrual
///   instead (as v1.2 did) meant a diagonal turn fed each axis at half rate and
///   completed neither, so corkscrews would have needed four revolutions to bank
///   anything and were effectively worthless.
///
///   That discount only starts outside trickCleanAxisTolerance. v1.3 ramped it from
///   the instant a rotation was less than perfect, which shorted a flip that read as
///   clean: a stick six degrees off axis lost a tenth of the payout for a trick the
///   player would call textbook. Nobody can hold a thumbstick to a degree, so the
///   band is what makes the discount describe a genuine corkscrew rather than
///   ordinary human aim.
///
/// LANDING IT
///
///   Two gates.
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
///   contact when there was one and from the hover rays otherwise.
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
    // 📢 Public state and events
    // -------------------------------------------------------------------------

    /// <summary>Completed barrel rolls banked on the current flight.</summary>
    public int BarrelRollCount => _rollCount;

    /// <summary>Completed flips banked on the current flight.</summary>
    public int FlipCount => _flipCount;

    /// <summary>True while a flight is being scored, whether or not it has earned anything yet.</summary>
    public bool IsTracking => _armed;

    /// <summary>Energy the current flight would pay if it landed cleanly right now.</summary>
    public float EscrowEnergy => PendingPayout();

    /// <summary>
    /// Raised once when a flight resolves. Arguments are whether it was landed, what
    /// the flight was WORTH, and what actually reached the pool. The two numbers
    /// differ whenever the pool could not take the whole payout -- because it was
    /// already full, or because an EMP freeze refused the grant outright -- and a
    /// readout needs both: the first teaches the price list, the second is the truth
    /// about what was gained. Collapsing them to one number means lying about one or
    /// the other.
    ///
    /// Added in v1.2 rather than v1.0 on purpose: this project has deleted three
    /// events that were raised with no subscriber. VehicleHUD's trick tracker is
    /// the subscriber that justifies it.
    /// </summary>
    public event System.Action<bool, float, float> OnTrickResolved;

    // -------------------------------------------------------------------------
    // Runtime
    // -------------------------------------------------------------------------
    private HoverController_Foundation foundation;
    private HoverController_Propulsion propulsion;
    private HoverController_Energy     energy;
    private Rigidbody                  rb;

    /// <summary>Air control has engaged during the current flight, so rotation counts.</summary>
    private bool _armed;

    /// <summary>Completed revolutions banked this flight. Never decremented.</summary>
    private int _rollCount;
    private int _flipCount;

    /// <summary>
    /// SIGNED progress toward the next revolution on each axis, in turns. Turning
    /// back unwinds it, which is what stops a wobble from paying.
    /// </summary>
    private float _rollProgress;
    private float _flipProgress;

    /// <summary>
    /// Rotation-weighted sum of how diagonal the turning was, and the total rotation
    /// it is weighted against. Their ratio is the flight's average diagonal-ness,
    /// which is what the corkscrew multiplier scales the payout by.
    /// </summary>
    private float _mixSum;
    private float _mixWeight;

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
    private int    _dbgLastRolls;
    private int    _dbgLastFlips;
    private bool   _dbgLastWasBanked;
    private bool   _dbgLastEmpBlocked;
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

        // SIGNED, so turning back unwinds progress instead of adding to it.
        float pitchRate = localAngVel.x;
        float rollRate  = localAngVel.z;

        float absPitch = Mathf.Abs(pitchRate);
        float absRoll  = Mathf.Abs(rollRate);

        // Magnitude, not the sum, so a diagonal rotation is measured honestly.
        float rate = Mathf.Sqrt(absPitch * absPitch + absRoll * absRoll);
        if (rate <= 0f)
            return;

        // Diagonal-ness: 0 on a clean axis, 1 on an evenly split corkscrew. Averaged
        // over the flight and applied to the PAYOUT rather than to the accrual, so a
        // corkscrew still completes revolutions at the normal rate.
        float pitchShare = absPitch / (absPitch + absRoll);

        // Purity is how much of the turning is on its dominant axis: 1 is perfectly
        // clean, 0.5 is an even corkscrew. Working from the dominant axis rather than
        // from pitchShare directly is what makes this symmetric between rolls and flips.
        float purity = Mathf.Max(pitchShare, 1f - pitchShare);
        float tol    = E.trickCleanAxisTolerance;

        // A tolerance BAND rather than a point. Without it the discount begins the
        // instant a rotation is not perfect, so a flip a few degrees off the stick's
        // axis was quietly shorted despite reading as clean. Inside the band it is
        // free; outside, the discount ramps smoothly to full at an even corkscrew.
        float mix = (tol <= 0.5f || purity >= tol)
            ? 0f
            : (tol - purity) / (tol - 0.5f);

        float travelled = rate * dt / (2f * Mathf.PI);

        _mixSum    += mix * travelled;
        _mixWeight += travelled;

        _rollProgress += rollRate  * dt / (2f * Mathf.PI);
        _flipProgress += pitchRate * dt / (2f * Mathf.PI);

        float threshold = E.trickRevolutionThreshold;
        BankRevolutions(ref _rollProgress, ref _rollCount, threshold);
        BankRevolutions(ref _flipProgress, ref _flipCount, threshold);
    }

    /// <summary>
    /// Converts signed progress into banked revolutions once it reaches the
    /// threshold, keeping the remainder.
    ///
    /// Direction-agnostic on purpose: a turn to the left and a turn to the right
    /// are both a completed revolution, and banking on the CROSSING is what makes a
    /// counter-rotation to set up the landing free rather than costly.
    ///
    /// It subtracts the THRESHOLD rather than a whole turn, which is the part worth
    /// understanding. Subtracting a whole turn would make the credit a one-time head
    /// start, so the measurement's ~3% shortfall would accumulate and the fourth
    /// revolution of a long spin would fail to clock even though the first three
    /// did. Charging the threshold per revolution keeps the forgiveness constant, so
    /// a spin of any length banks at the same rate. It also makes alternating
    /// direction cost exactly what continuing costs, which removes the only way to
    /// game a sub-1 threshold.
    /// </summary>
    private static void BankRevolutions(ref float progress, ref int count, float threshold)
    {
        if (threshold <= 0f)
            return;

        while (progress >= threshold)  { progress -= threshold; count++; }
        while (progress <= -threshold) { progress += threshold; count++; }
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

        if (_rollCount == 0 && _flipCount == 0)
        {
            // Airborne, but nothing ever came all the way round. Cleared rather than
            // banked: that was a jump, not a trick.
            Resolve(false, "no revolution");
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

        bool tooBanked  = Mathf.Abs(_arrivalRoll) > E.trickMaxLandingRollAngle;
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
    /// One ledger's worth: the first revolution at its own price, every one after
    /// it at the repeat price.
    /// </summary>
    private static float Ledger(int count, float first, float repeat)
    {
        if (count <= 0)
            return 0f;

        return first + repeat * (count - 1);
    }

    /// <summary>
    /// Energy for a given pair of revolution counts. Public and static so an
    /// inspector readout and the runtime cannot disagree about what a trick is
    /// worth: one formula, both callers.
    ///
    /// corkscrewScale is the flight's average diagonal-ness already converted to a
    /// multiplier, so this stays a pure function of numbers a caller can supply.
    /// </summary>
    public static float PayoutFor(int rollCount, int flipCount, float corkscrewScale, EnergyTuning tuning)
    {
        float raw = Ledger(rollCount, tuning.trickBarrelRollEnergy, tuning.trickBarrelRollRepeatEnergy)
                  + Ledger(flipCount, tuning.trickFlipEnergy,       tuning.trickFlipRepeatEnergy);

        float payout = raw * corkscrewScale;

        // Tidy the fractions the discount produces, and ONLY those. A clean trick is
        // already a round number because the four prices are, so rounding it could
        // only move a value that was set deliberately. The epsilon is for float
        // noise in the scale, not a tolerance.
        float step = tuning.trickPayoutRounding;
        if (step > 0f && payout > 0f && corkscrewScale < 0.999f)
        {
            // Half-up rather than Mathf.Round, which is banker's rounding and would
            // send 12.5 and 17.5 in opposite directions for no reason a player could
            // follow.
            payout = Mathf.Floor(payout / step + 0.5f) * step;

            // A trick that earned something must never round away to nothing.
            if (payout < step)
                payout = step;
        }

        return payout;
    }

    /// <summary>
    /// How much the corkscrew multiplier is discounting this flight: 1 for rotation
    /// flown clean, the tuned multiplier for rotation flown entirely on the
    /// diagonal, weighted by how much turning happened at each moment.
    /// </summary>
    private float CorkscrewScale =>
        _mixWeight > 0f
            ? Mathf.Lerp(1f, E.trickCorkscrewMultiplier, _mixSum / _mixWeight)
            : 1f;

    /// <summary>Payout the current flight would earn if it landed cleanly right now.</summary>
    private float PendingPayout() => PayoutFor(_rollCount, _flipCount, CorkscrewScale, E);

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
        // Sampled before the grant so the gizmo can name WHY a payout was refused.
        // Grant does not clear the freeze, but reading it first keeps the two facts
        // independent of each other's order.
        bool empBlocked = energy != null && energy.IsEmpFrozen;

        float granted = payout > 0f ? energy.Grant(payout) : 0f;

        _dbgLastPayout      = payout;
        _dbgLastGranted     = granted;
        _dbgLastRolls       = _rollCount;
        _dbgLastFlips       = _flipCount;
        _dbgLastWasBanked   = banked;
        _dbgLastEmpBlocked  = empBlocked;
        _dbgLastReason      = reason;
        _dbgLastResolveTime = Time.unscaledTime;

        // Raised BEFORE the reset, deliberately: the payout has already been granted
        // by this point, so the flight's state is complete and consistent, and a
        // subscriber can read BarrelRollCount and FlipCount to describe what just
        // happened. Raising it after would hand every listener zeroes.
        OnTrickResolved?.Invoke(banked, payout, granted);

        _armed        = false;
        _rollCount    = 0;
        _flipCount    = 0;
        _rollProgress = 0f;
        _flipProgress = 0f;
        _mixSum       = 0f;
        _mixWeight    = 0f;
        _settling     = false;
        _settleTimer  = 0f;
    }

    // -------------------------------------------------------------------------
    // 🎨 Debug Gizmos
    // -------------------------------------------------------------------------
#if UNITY_EDITOR
    /// <summary>
    /// Sits above the Energy label so the payout can be read against the pool it is
    /// paying into. Shows progress toward the next revolution as well as the banked
    /// count, because the banked count alone cannot tell you whether a trick was
    /// nearly round or barely started.
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

        string banked = $"roll x{_rollCount}  flip x{_flipCount}";
        string toward = $"(+{_rollProgress:F2} / {_flipProgress:F2})";

        if (_settling)
        {
            label = $"TRICK LANDING {banked} -> {PendingPayout():F0}  hold {_settleTimer:F2}s\n"
                  + $"  arrived bank {_arrivalRoll:F0}deg  nose {_arrivalFlip:F0}deg";
            color = Color.yellow;
        }
        else if (_armed)
        {
            float pending = PendingPayout();
            label = pending > 0f
                ? $"TRICK {banked} -> {pending:F0}  {toward}"
                : $"TRICK {toward} (nothing round yet)";
            color = Color.cyan;
        }
        else if (Time.unscaledTime - _dbgLastResolveTime < 2f)
        {
            if (_dbgLastWasBanked)
            {
                // Naming the shortfall matters: "earned 25, got 4" reads as a full
                // pool, where a bare "+4" reads as the payout being wrong.
                label = _dbgLastGranted < _dbgLastPayout - 0.01f
                    ? $"TRICK BANKED +{_dbgLastGranted:F0} of {_dbgLastPayout:F0}  ({(_dbgLastEmpBlocked ? "EMP frozen" : "pool full")})"
                    : $"TRICK BANKED +{_dbgLastGranted:F0}  (roll x{_dbgLastRolls} flip x{_dbgLastFlips})";
                color = Color.green;
            }
            else
            {
                label = $"TRICK LOST: {_dbgLastReason}  (roll x{_dbgLastRolls} flip x{_dbgLastFlips})";
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
