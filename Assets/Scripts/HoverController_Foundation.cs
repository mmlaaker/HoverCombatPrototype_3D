using UnityEngine;

/// <summary>
/// HoverController_Foundation v1.2
///
/// v1.2: Air control torque. Propulsion pushes airborne pitch/roll intent via
///       SetAirControl(pitch, roll, weight) while drift is held airborne;
///       per-axis torque authority (roll runs hotter than pitch), and
///       pitch/roll damping crossfades toward airControlDamping by the weight
///       so sustained rotation is actually reachable. Still the single
///       attitude authority: Propulsion applies no attitude torque itself.
///
/// v1.1: Hard landing spring give. On an airborne-to-grounded edge above a speed
///       threshold, spring forces are suppressed for a short front-loaded window
///       so the chassis slams onto its collider before popping back to ride
///       height. Fires OnHardLanding(severity). Feel only; no damage, no lockout.
///
/// Holds the chassis at hover height via per-point spring-dampers, levels it to
/// the ground normal, damps tilt, and runs two recovery paths:
///
///   Upright unstick: chassis touching ground but not flipped. A short tapering
///                    upward push frees it. Horizontal speed is intentionally
///                    NOT gated (unstick must work after any landing velocity);
///                    vertical speed IS gated so dynamic belly scrapes don't
///                    receive pulse trains while the hover springs are working.
///
///   Flip recovery:   chassis touching ground, flipped, and slow. Righting
///                    torque rotates it back upright. Longer delay so a flip
///                    reads as a real consequence.
///
/// Both paths can be suspended via SetRecoveryEnabled(bool) for EMP, scripted
/// events, or ability hooks.
///
/// Tuning lives on a VehicleTuningProfile asset (profile.foundation). Scene refs
/// (hoverPoints, groundLayers) and runtime state stay on the component.
///
/// Physics contract: never writes to rb.angularVelocity, rb.rotation, or
/// rb.linearVelocity. All motion is AddForce / AddTorque.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class HoverController_Foundation : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // 📦 Tuning Profile
    // -------------------------------------------------------------------------
    [Header("📦 Tuning")]
    [Tooltip("Vehicle tuning profile (shared SO). All numeric tuning lives here. Required.")]
    [SerializeField] private VehicleTuningProfile profile;

    /// <summary>Shorthand for profile.foundation. Used at every read site below.</summary>
    private FoundationTuning F => profile.foundation;

    // -------------------------------------------------------------------------
    // 🧩 Hover Points
    // -------------------------------------------------------------------------
    [Header("🧩 Hover Points")]
    [Tooltip("Empty transforms positioned at the bottom of the chassis (one per corner is typical). " +
             "Each casts a ray straight down to find the ground. Assign by hand from the hover point children.")]
    [SerializeField] private Transform[] hoverPoints;

    // -------------------------------------------------------------------------
    // 🌍 Ground Interaction
    // -------------------------------------------------------------------------
    [Header("🌍 Ground Interaction")]
    [Tooltip("Layers treated as ground for hover detection.")]
    [SerializeField] private LayerMask groundLayers = ~0;

    [Tooltip("Draws hover rays in the scene view.")]
    [SerializeField] private bool drawDebugRays = true;

    [Tooltip("Optional global debug toggle. When assigned, overrides Draw Debug Rays. " +
             "Create via Assets > Create > Hover > Debug Settings.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.enableDebugGizmos : drawDebugRays;

    // -------------------------------------------------------------------------
    // 📡 Collision Tracking
    // -------------------------------------------------------------------------
    /// <summary>
    /// Tracks ground-layer collision contact. OnCollisionStay fires every physics
    /// step while contact persists, refreshing a timestamp. Contact state is a
    /// timestamp, not a boolean, per the timestamps-over-booleans principle:
    /// an OnCollisionExit-cleared bool goes false when exiting ONE of two
    /// simultaneously touching ground colliders, and needs the exit callback to
    /// fire at all (not guaranteed on collider destruction). The timestamp simply
    /// goes stale. floorContactNormal is only read behind IsContactingFloor, so
    /// staleness there is harmless.
    /// Both flip recovery and unstick read these values; neither writes them.
    /// </summary>
    private void OnCollisionStay(Collision collision)
    {
        if ((groundLayers.value & (1 << collision.gameObject.layer)) == 0)
            return;

        lastGroundContactTime = Time.time;

        int count = collision.GetContacts(_contactBuffer);
        if (count == 0)
            return;

        // Two separate signals, and they must stay separate.
        //
        //   lastGroundContactTime / IsContactingGround: touching ANYTHING. Flip
        //   recovery and the downed lockout need this, because a craft flipped
        //   against a wall is still very much down and must still recover.
        //
        //   lastFloorContactTime / floorContactNormal: touching something
        //   FLAT ENOUGH TO BE STUCK ON. Only unstick uses this. It used to use
        //   the all-contact average, so a wall's horizontal normal became the
        //   unstick push direction and the craft shoved itself off any wall it
        //   touched, once every unstickRecoveryDelay. Hover cannot lift a craft
        //   off a wall, so there is nothing there to unstick from.
        float minNormalY = Mathf.Cos(F.unstickMaxSurfaceAngle * Mathf.Deg2Rad);

        Vector3 floorSum   = Vector3.zero;
        int     floorCount = 0;

        for (int i = 0; i < count; i++)
        {
            Vector3 n = _contactBuffer[i].normal;
            if (n.y < minNormalY)
                continue;

            floorSum += n;
            floorCount++;
        }

        // Averaged across floor contacts only, so touching a wall and the floor
        // at once still yields a clean upward push instead of a diagonal one.
        if (floorCount > 0)
        {
            lastFloorContactTime = Time.time;
            floorContactNormal   = (floorSum / floorCount).normalized;
        }
    }

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------
    private Rigidbody rb;
    private HoverController_Energy energy;

    // Pre-allocated buffer for OnCollisionStay contact reads. Avoids per-call
    // ContactPoint[] allocation on every physics tick.
    private readonly System.Collections.Generic.List<ContactPoint> _contactBuffer = new(8);

    // Pre-allocated buffer for the hover raycasts. RaycastNonAlloc is used
    // instead of Raycast so self-hits can be skipped in favour of the next
    // hit along the ray (see ApplyHoverForces). Sized well above the number of
    // colliders a single 9.5m sensor ray can plausibly cross.
    private readonly RaycastHit[] _hoverHitBuffer = new RaycastHit[16];

    private float   unstickTimer;            // counts up while upright and contacting ground
    private float   flipTimer;               // counts up while flipped, slow, and contacting ground
    private float   lastGroundContactTime = float.NegativeInfinity; // any contact, refreshed by OnCollisionStay
    private float   lastFloorContactTime  = float.NegativeInfinity; // floor-like contact only (see OnCollisionStay)
    private Vector3 floorContactNormal;

    /// <summary>
    /// True while ground contact is fresh. OnCollisionStay refreshes the
    /// timestamp every physics step during contact; two fixed steps of grace
    /// covers callback ordering within a step.
    /// </summary>
    private bool IsContactingGround => Time.time - lastGroundContactTime <= Time.fixedDeltaTime * 2f;

    /// <summary>
    /// True while contact with a surface flat enough to be stuck on is fresh.
    /// Strictly narrower than IsContactingGround, which counts walls. Only the
    /// unstick path reads this; recovery deliberately uses the wider signal.
    /// </summary>
    private bool IsContactingFloor => Time.time - lastFloorContactTime <= Time.fixedDeltaTime * 2f;

    // Aim pitch target, written by Propulsion via SetAimPitch each FixedUpdate
    // while strafe mode is active. Degrees in Unity euler-X convention
    // (negative = nose up). Weight is the strafe blend (0 = no target).
    private float aimPitchDegrees;
    private float aimPitchWeight;

    // Air control intent, written by Propulsion via SetAirControl each
    // FixedUpdate while airborne with drift held. Inputs are raw stick -1..1;
    // weight is the air-control blend already suppressed by strafe precedence
    // (0 = inactive).
    private float airControlPitchInput;
    private float airControlRollInput;
    private float airControlWeight;
    private float   unstickFiredFlashTimer;  // drives the fired-impulse gizmo
    private bool    firedFlashWasFlip;       // which path set the flash (label only)
    private bool    rightingAuthorized;      // true after flip timer threshold; cleared when craft rights
    private bool    recoveryEnabled = true;
    private float   unstickForceTimer;       // counts down while sustained unstick lift is being applied
    private Vector3 unstickForceDir;         // direction cached at trigger time, held for duration
    private float   _effectiveHoverHeight;   // ride height actually targeted this tick (ceiling duck)
    private float   hardLandingTimer;        // counts down while hard-landing spring suppression is active
    private float   hardLandingSeverity;     // 0..1, cached at detection so the taper scales with impact

    /// <summary>True when at least one hover point has a ground hit this frame.</summary>
    public bool IsHoverGrounded { get; private set; }

    /// <summary>Average ground normal this frame (Vector3.up when airborne).</summary>
    public Vector3 AverageGroundNormal { get; private set; }

    /// <summary>
    /// True while the craft is flipped AND resting against the ground: the state
    /// flip recovery exists to undo. Propulsion reads this to lock out the jump
    /// and all commanded torque, so a flip costs the player the full recovery
    /// time instead of being cancellable.
    ///
    /// Both terms are load-bearing:
    ///
    ///   Tilt alone is not enough. Air control is gated on this, and a barrel
    ///   roll passes through the flip threshold every single time, so a
    ///   tilt-only test would cut the player's authority in the middle of the
    ///   air tricks the system exists for.
    ///
    ///   Contact alone is not enough either. IsHoverGrounded cannot stand in for
    ///   it: on its flank the craft's sensor rays point sideways and find
    ///   nothing, so it reads as airborne while physically lying on the floor.
    ///   That is exactly the hole that let a downed player hold drift and lever
    ///   themselves upright against the ground with attitude thrusters, at
    ///   measured 4.79 rad/s, beating the recovery outright.
    ///
    /// See UpdateDownedState for the exact condition and why it also latches.
    /// </summary>
    public bool IsDowned { get; private set; }

    // -------------------------------------------------------------------------
    // 📢 Events
    // -------------------------------------------------------------------------

    /// <summary>
    /// Fired once when the vehicle lands harder than hardLandingMinSpeed.
    /// Parameter is severity 0..1 mapping min..max impact speed. Pure feel hook
    /// today (VFX, camera punch); gameplay consequences can subscribe later.
    /// </summary>
    public event System.Action<float> OnHardLanding;

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /// <summary>
    /// Sets the aim pitch target for the leveling torque. Called by Propulsion
    /// every FixedUpdate while strafe mode is active (weight = strafe blend),
    /// and with (0, 0) on strafe exit or EMP freeze.
    ///
    /// Degrees use Unity's euler-X convention: negative = nose up.
    /// While weight > 0, leveling drives toward the ground normal rotated by
    /// this pitch, at aimPitchTrackingStrength on the pitch axis only. This
    /// makes leveling the single torque authority over attitude; Propulsion
    /// applies no pitch torque of its own (competing forces cause jitter).
    /// </summary>
    public void SetAimPitch(float degrees, float weight)
    {
        aimPitchDegrees = degrees;
        aimPitchWeight  = Mathf.Clamp01(weight);
    }

    /// <summary>
    /// Sets the airborne pitch/roll control intent. Called by Propulsion every
    /// FixedUpdate (weight = air-control blend, strafe-suppressed), and with
    /// (0, 0, 0) when inactive or on EMP freeze.
    ///
    /// Sign conventions (applied here, not by the caller):
    ///   pitchInput +1 (stick up)    = nose down (+local X), arcade car standard.
    ///   rollInput  +1 (stick right) = roll right (-local Z), matching the
    ///   chassis-bank convention in Propulsion.
    /// </summary>
    public void SetAirControl(float pitchInput, float rollInput, float weight)
    {
        airControlPitchInput = Mathf.Clamp(pitchInput, -1f, 1f);
        airControlRollInput  = Mathf.Clamp(rollInput,  -1f, 1f);
        airControlWeight     = Mathf.Clamp01(weight);
    }

    /// <summary>
    /// Enables or disables both flip recovery and ground unstick.
    /// Use for EMP effects, scripted events, or ability interactions.
    /// When disabled, timers are reset so recovery doesn't fire immediately on re-enable.
    /// </summary>
    public void SetRecoveryEnabled(bool enabled)
    {
        recoveryEnabled = enabled;
        if (!enabled)
        {
            unstickTimer       = 0f;
            flipTimer          = 0f;
            rightingAuthorized = false;
        }
    }

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------
    private void Awake()
    {
        rb     = GetComponent<Rigidbody>();
        energy = GetComponent<HoverController_Energy>();

        if (profile == null)
        {
            Debug.LogError(
                $"[Foundation] '{name}': VehicleTuningProfile is not assigned. " +
                $"Assign one in the inspector. Hover disabled.",
                this
            );
            enabled = false;
            return;
        }

        ValidateHoverPoints();
    }

    private void ValidateHoverPoints()
    {
        if (hoverPoints == null || hoverPoints.Length == 0)
        {
            Debug.LogError(
                $"[Foundation] '{name}': hoverPoints array is empty. " +
                $"Assign HoverPoint transforms in the Inspector. Hover disabled.",
                this
            );
            enabled = false;
            return;
        }

        for (int i = 0; i < hoverPoints.Length; i++)
        {
            if (hoverPoints[i] == null)
            {
                Debug.LogError(
                    $"[Foundation] '{name}': hoverPoints[{i}] is null. " +
                    $"Fix the missing reference in the Inspector. Hover disabled.",
                    this
                );
                enabled = false;
                return;
            }
        }
    }

    private void FixedUpdate()
    {
        // EMP freeze: all Foundation forces off. Default Unity gravity still pulls
        // the chassis down; existing angular velocity persists with no leveling or
        // damping, so the vehicle tumbles for the freeze duration.
        if (energy != null && energy.IsEmpFrozen)
        {
            IsHoverGrounded     = false;
            AverageGroundNormal = Vector3.up;
            IsDowned            = false;   // EMP already removes all control
            return;
        }

        ApplyHoverForces();
        ApplyExtraGravity();
        ApplyLevelingTorque();
        ApplyAirControlTorque();
        ApplyPitchRollDamping();
        HandleRecovery();
        UpdateDownedState();
        ApplyUnstickForce();
    }

    private void Update()
    {
        // Tick the debug flash timer in Update so it runs at game framerate.
        // OnDrawGizmos runs on the editor's schedule (unpredictable, sometimes
        // multiple times per frame, sometimes suppressed) and must not mutate state.
        if (unstickFiredFlashTimer > 0f)
            unstickFiredFlashTimer = Mathf.Max(0f, unstickFiredFlashTimer - Time.deltaTime);
    }

    // -------------------------------------------------------------------------
    // 🧠 Per-Point Spring-Damper Lift
    // -------------------------------------------------------------------------
    /// <summary>
    /// Raycast that never returns this craft's own geometry, taking the nearest
    /// surviving hit. RaycastNonAlloc does not sort, hence the explicit min.
    /// attachedRigidbody resolves every chassis collider to this Rigidbody, so
    /// one comparison covers the whole hierarchy.
    ///
    /// Self-skipping is not optional here. groundLayers defaults to Everything
    /// and the chassis colliders sit on the vehicle's own layer, so a sensor ray
    /// can and does hit this craft: once ApplyChassisBank rolls meshRoot past
    /// ~15.5 degrees it swings Rim_FR / Rim_RR straight into their own hover
    /// rays. The ray then returned ~0.04m instead of ~6.8m, compression read as
    /// nearly the whole hoverHeight, and one flank's spring jumped from 12.5 to
    /// 121 m/s^2 -- about 6x the craft's weight, on one side, pushed along the
    /// RIM's surface normal instead of the ground's. That was the drift flip.
    /// </summary>
    private bool RaycastIgnoringSelf(Vector3 origin, Vector3 direction, float range, out RaycastHit nearest)
    {
        int count = Physics.RaycastNonAlloc(origin, direction, _hoverHitBuffer, range,
                                            groundLayers, QueryTriggerInteraction.Ignore);

        bool found = false;
        nearest = default;

        for (int i = 0; i < count; i++)
        {
            if (_hoverHitBuffer[i].collider.attachedRigidbody == rb)
                continue;

            if (!found || _hoverHitBuffer[i].distance < nearest.distance)
            {
                nearest = _hoverHitBuffer[i];
                found   = true;
            }
        }

        return found;
    }

    /// <summary>
    /// Ride height the springs actually target this tick, reduced when something
    /// overhead would otherwise be crushed into.
    ///
    /// The springs have no force ceiling, so a ceiling below ride height is a
    /// SOFT-LOCK rather than a nuisance: net upward push is 64 m/s^2 per metre of
    /// compression, and friction against the ceiling eats the drive. Measured on
    /// the default chassis at 8m of clearance, full throttle produced a top speed
    /// of 0.7 m/s and 0.4m of travel in two seconds. At 9m it reached 59.9 m/s.
    /// The entire transition is one metre wide, which is far too sharp to leave to
    /// level-design discipline when the failure is an immobilised player.
    ///
    /// Both probes fire from the hover-point centroid so their sum is the ABSOLUTE
    /// floor-to-ceiling gap, which does not depend on where in that gap the craft
    /// currently sits. Deriving the target from the ceiling distance alone would
    /// make it a function of current height and feed back on itself.
    ///
    /// One central probe, not one per hover point, deliberately: per-corner ducking
    /// under a sloped ceiling would drive the four points to different targets and
    /// tilt the chassis.
    /// </summary>
    private float ComputeEffectiveHoverHeight()
    {
        if (!F.enableCeilingDuck || hoverPoints.Length == 0)
            return F.hoverHeight;

        Vector3 centroid = Vector3.zero;
        foreach (Transform point in hoverPoints)
            centroid += point.position;
        centroid /= hoverPoints.Length;

        Vector3 up = transform.up;

        // Anything higher than this cannot constrain us, so there is no reason to
        // look for it.
        float probeRange = F.hoverHeight + F.ceilingClearance;

        if (!RaycastIgnoringSelf(centroid, up, probeRange, out RaycastHit ceiling))
            return F.hoverHeight;

        if (!RaycastIgnoringSelf(centroid, -up, F.sensorRange, out RaycastHit floor))
            return F.hoverHeight;

        float gap = floor.distance + ceiling.distance;

        return Mathf.Clamp(gap - F.ceilingClearance, F.minDuckHoverHeight, F.hoverHeight);
    }

    private void ApplyHoverForces()
    {
        bool wasHoverGrounded = IsHoverGrounded;

        IsHoverGrounded     = false;
        AverageGroundNormal = Vector3.up;

        // Hard landing: tick the suppression window (front-loaded taper, same
        // idiom as unstick). liftFactor is exactly 1 when the timer is idle, so
        // the normal hot path pays one comparison.
        if (hardLandingTimer > 0f)
            hardLandingTimer = Mathf.Max(0f, hardLandingTimer - Time.fixedDeltaTime);

        float liftFactor = 1f;
        if (hardLandingTimer > 0f)
        {
            float progress = hardLandingTimer / Mathf.Max(0.01f, F.hardLandingSuppressDuration);
            liftFactor = 1f - F.hardLandingSuppressStrength * hardLandingSeverity * progress;
        }

        // Total downward acceleration the springs have to hold against: Unity's own
        // gravity plus the extraGravityMultiplier added in ApplyExtraGravity. Hoisted
        // out of the loop; every point feeds forward an equal share of it.
        float gravityMagnitude = Physics.gravity.magnitude * (1f + F.extraGravityMultiplier);

        // Reduced when something overhead would otherwise be crushed into. Applied
        // uniformly to all four points so a sloped ceiling cannot tilt the chassis.
        _effectiveHoverHeight = ComputeEffectiveHoverHeight();

        Vector3 normalSum     = Vector3.zero;
        int     groundedCount = 0;

        foreach (Transform point in hoverPoints)
        {
            Vector3 rayDir = -point.up;

            // QueryTriggerInteraction.Ignore (inside the helper): trigger volumes
            // (pickups, ability fields) must never feed the hover springs.
            if (!RaycastIgnoringSelf(point.position, rayDir, F.sensorRange, out RaycastHit hit))
            {
                if (ShouldDrawDebug)
                    Debug.DrawRay(point.position, rayDir * F.sensorRange, Color.red);
                continue;
            }

            float compression         = _effectiveHoverHeight - hit.distance;
            float velocityAlongNormal = Vector3.Dot(rb.GetPointVelocity(point.position), hit.normal);

            // Gravity feedforward: each point carries its share of the chassis weight,
            // so the spring term only has to correct ERROR instead of also holding the
            // craft up. Without this the springs must compress until k*x equals weight,
            // which parked ride height a fixed gravity/stiffness below hoverHeight
            // (0.98m at the old 39.24 / 40). Now hoverHeight is literal, and liftStrength
            // is free to be tuned purely for how hard the chassis resists being pushed
            // around -- height and looseness stop being the same knob inverted.
            //
            // Scaled by hit.normal.y so it supports exactly the NORMAL component of
            // weight (G*cos0) and leaves the tangential component unopposed. Feeding it
            // forward along world up instead would cancel gravity outright and glue the
            // chassis to slopes.
            float gravityShare = gravityMagnitude * hit.normal.y / hoverPoints.Length;

            float springForce = compression * F.liftStrength
                              - velocityAlongNormal * F.liftDamping
                              + gravityShare;

            // Clamp at zero: the spring only pushes, never pulls. Load-bearing for
            // jump feel — without it the damping term would fight jump takeoff.
            springForce = Mathf.Max(springForce, 0f);

            // Hard landing spring give: momentum carries the chassis through
            // hover height onto its collider while the taper restores strength.
            springForce *= liftFactor;

            // ForceMode.Acceleration: mass independent, like every other force in
            // the controllers (jump, dodge, unstick, gravity, all torques).
            // liftStrength/liftDamping are accelerations, so one tuning profile
            // behaves identically across vehicle masses. Values retuned from
            // Force-mode by dividing by vehicle mass (1000): 10000 -> 10,
            // 1500 -> 1.5. Identical forces at mass 1000; feel unchanged.
            rb.AddForceAtPosition(hit.normal * springForce, point.position, ForceMode.Acceleration);

            normalSum    += hit.normal;
            groundedCount++;

            if (ShouldDrawDebug)
                Debug.DrawRay(point.position, rayDir * hit.distance, Color.green);
        }

        if (groundedCount > 0)
        {
            IsHoverGrounded     = true;
            AverageGroundNormal = (normalSum / groundedCount).normalized;

            if (!wasHoverGrounded)
                DetectHardLanding();
        }
    }

    // -------------------------------------------------------------------------
    // 💥 Hard Landing Detection
    // -------------------------------------------------------------------------
    /// <summary>
    /// Called on the airborne-to-grounded edge. Samples descent speed along the
    /// ground normal (rb.linearVelocity is pre-integration here, so this frame's
    /// spring forces have not contaminated the read) and, above the threshold,
    /// arms the spring-suppression window and fires OnHardLanding.
    /// </summary>
    private void DetectHardLanding()
    {
        if (!F.enableHardLanding)
            return;

        float impactSpeed = -Vector3.Dot(rb.linearVelocity, AverageGroundNormal);

        if (impactSpeed < F.hardLandingMinSpeed)
            return;

        hardLandingSeverity = Mathf.InverseLerp(F.hardLandingMinSpeed, F.hardLandingMaxSpeed, impactSpeed);
        hardLandingTimer    = F.hardLandingSuppressDuration;

        OnHardLanding?.Invoke(hardLandingSeverity);

        if (ShouldDrawDebug)
            Debug.Log($"[Foundation] Hard landing: {impactSpeed:F1} m/s, severity {hardLandingSeverity:F2}", this);
    }

    // -------------------------------------------------------------------------
    // 🌎 Extra Gravity
    // -------------------------------------------------------------------------
    /// <summary>
    /// extraGravityMultiplier always applies. It increases the effective weight of the
    /// chassis and helps it sit firmly on the hover springs.
    /// extraFallGravity applies only while airborne AND DESCENDING. Gravity is otherwise
    /// symmetric, so the only way to kill float used to be raising it globally -- which
    /// also flattened jump arcs and forced the large jump impulses that compensate for it.
    /// Weighting the descent alone lets the ascent stay generous (air-trick window) while
    /// the landing still reads as decisive.
    /// Both are tuning knobs for physical character, not player input.
    /// </summary>
    private void ApplyExtraGravity()
    {
        if (F.extraGravityMultiplier > 0f)
            rb.AddForce(Physics.gravity * F.extraGravityMultiplier, ForceMode.Acceleration);

        if (IsHoverGrounded)
            return;

        // World-space Y, matching the world-up jump impulse. Reading the body axis here
        // would flip the sign mid-flip and yank the chassis upward during air control.
        float airGravity = rb.linearVelocity.y < 0f ? F.extraFallGravity : 0f;

        if (airGravity > 0f)
            rb.AddForce(Vector3.down * airGravity, ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // ⚖️ Torque-Based Attitude Alignment (ground normal + aim pitch)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Single torque authority over attitude. Two responsibilities, one torque:
    ///
    ///   Grounded leveling: align chassis up to AverageGroundNormal (original role,
    ///               unchanged behavior when no aim target is set).
    ///
    ///   Aim pitch:  while Propulsion sets a target via SetAimPitch (strafe mode),
    ///               the target up is the ground normal rotated by the aim pitch.
    ///               Axis split: the pitch-axis torque component runs at
    ///               aimPitchTrackingStrength, everything else (roll, bump
    ///               following) stays at base leveling strength, so FPS-snappy aim
    ///               tuning (150 vs 12) never stiffens the ride over bumps.
    ///               Replaces the old design where Propulsion torqued the nose
    ///               *against* this leveling (competing forces cause jitter).
    ///               pitchRollDamping is the oscillation killer.
    ///
    /// Airborne with an aim target, the pitch axis still tracks (aiming mid-jump
    /// matters in combat); AverageGroundNormal is Vector3.up there, and the align
    /// component gets zero strength, preserving free airborne attitude.
    /// </summary>
    private void ApplyLevelingTorque()
    {
        // Aim tracking is honored when hover springs are engaged or the craft is
        // truly in the air. While scraping ground without hover (belly/roof
        // contact), recovery paths own attitude and aim must not fight them.
        bool aiming = aimPitchWeight > 0.001f
                   && (IsHoverGrounded || !IsContactingGround);

        float baseStrength = IsHoverGrounded ? F.levelingTorqueStrength : 0f;

        if (!aiming && baseStrength <= 0f)
            return;

        Vector3 targetUp = aiming
            ? Quaternion.AngleAxis(aimPitchDegrees, transform.right) * AverageGroundNormal
            : AverageGroundNormal;

        Vector3 torqueAxis = Vector3.Cross(transform.up, targetUp);

        if (aiming)
        {
            Vector3 pitchPart = Vector3.Project(torqueAxis, transform.right);
            Vector3 alignPart = torqueAxis - pitchPart;

            float aimStrength = Mathf.Lerp(baseStrength, F.aimPitchTrackingStrength, aimPitchWeight);
            rb.AddTorque(pitchPart * aimStrength + alignPart * baseStrength, ForceMode.Acceleration);
        }
        else
        {
            rb.AddTorque(torqueAxis * baseStrength, ForceMode.Acceleration);
        }
    }

    // -------------------------------------------------------------------------
    // 🛩 Air Control Torque (airborne pitch/roll from Propulsion intent)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Applies pitch/roll torque from the air-control intent set by Propulsion
    /// (drift held while airborne). Independent per-axis authority: roll runs
    /// hotter than pitch so a car-shaped craft barrel rolls faster than it flips.
    ///
    /// Steady-state rate per axis is torque / airControlDamping (both are
    /// angular accelerations); ApplyPitchRollDamping lerps its damping toward
    /// airControlDamping by this weight so the rates are actually reachable.
    ///
    /// No grounded gate needed: airborne, leveling baseStrength is already 0 so
    /// nothing fights this; on landing the weight decays over the blend window
    /// (~0.15s) and grounded leveling wins immediately. No enable check here:
    /// enableAirControl gates at the intent source in Propulsion, same as
    /// enableStrafe gates SetAimPitch.
    /// </summary>
    private void ApplyAirControlTorque()
    {
        if (airControlWeight <= 0.001f)
            return;

        Vector3 torqueLocal = new Vector3(
             airControlPitchInput * F.airPitchTorque,   // +X = nose down
            0f,
            -airControlRollInput * F.airRollTorque      // -Z = roll right
        ) * airControlWeight;

        rb.AddRelativeTorque(torqueLocal, ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // ⚙️ Torque-Based Pitch/Roll Damping
    // -------------------------------------------------------------------------
    private void ApplyPitchRollDamping()
    {
        // Aim pitch damping: extra pitch-rate damping while aiming, scaled by
        // the strafe blend. pitchRollDamping alone (8) underdamps the aim
        // tracking spring at strength 150 (~35% flick overshoot), but raising it
        // globally would stiffen terrain response too. This keeps the two
        // tunable independently: ride damping vs aim settle.
        float pitchDamping = F.pitchRollDamping + F.aimPitchDamping * aimPitchWeight;
        float rollDamping  = F.pitchRollDamping;

        // Air control needs sustained rotation; ride damping (8) would cap roll
        // at half the intended rate and double the effective torque cost. Lerp
        // toward the dedicated air-control damping by air-control weight. Safe
        // against double-dip with aim damping: strafe precedence in Propulsion
        // makes aimPitchWeight and airControlWeight complementary, never both high.
        if (airControlWeight > 0f)
        {
            pitchDamping = Mathf.Lerp(pitchDamping, F.airControlDamping, airControlWeight);
            rollDamping  = Mathf.Lerp(rollDamping,  F.airControlDamping, airControlWeight);
        }

        if (pitchDamping <= 0f && rollDamping <= 0f)
            return;

        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);

        Vector3 dampingLocal = new Vector3(
            -localAngVel.x * pitchDamping,
            0f,
            -localAngVel.z * rollDamping
        );

        rb.AddRelativeTorque(dampingLocal, ForceMode.Acceleration);
    }

    // -------------------------------------------------------------------------
    // 🔄 Flip Recovery + 📌 Ground Unstick  (split paths)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Two fully independent recovery paths:
    ///
    ///   Upright unstick: contacting ground AND not flipped AND vertically
    ///                    settled (horizontal speed not gated). After
    ///                    unstickRecoveryDelay, begins a sustained upward push
    ///                    window via ApplyUnstickForce.
    ///
    ///   Flip recovery:   contacting ground AND flipped AND slow.
    ///                    After flipRecoveryDelay, sets rightingAuthorized which gates
    ///                    the righting torque each FixedUpdate until the craft rights
    ///                    or hover springs re-engage.
    ///
    /// 180° degeneracy: Cross(transform.up, Vector3.up) is zero when perfectly inverted.
    /// A forward pitch bias breaks the symmetry.
    /// </summary>
    private void HandleRecovery()
    {
        if (!recoveryEnabled)
            return;

        float tiltAngle = Vector3.Angle(transform.up, Vector3.up);
        bool  isFlipped = tiltAngle >= F.flipRecoveryAngleThreshold;
        bool  isSlow    = rb.linearVelocity.sqrMagnitude < F.flipRecoverySpeedThreshold * F.flipRecoverySpeedThreshold;

        // --- Righting torque (runs every frame while authorized) ---
        // Disarm on attitude ONLY, and at a LOWER angle than it armed at.
        //
        // The hysteresis is load-bearing. Releasing at flipRecoveryAngleThreshold
        // handed the craft over at exactly the angle where two hover points still
        // reach the ground and the resulting one-sided lift balances
        // levelingTorqueStrength: a stable equilibrium at ~78 degrees. Measured on
        // real terrain, the craft armed, righted from 112 to 80, released, sat at
        // 74-83 for a full second without converging, drifted back over the
        // threshold and re-armed -- indefinitely. Righting has to carry it clear
        // of that band before letting go.
        //
        // This was missed the first time because the 8-pose regression ran on a
        // flat plane, where the craft rotates cleanly through the band instead of
        // settling into it. Flat ground is not a sufficient test for this.
        //
        // Min() keeps the release at or below the arm angle if the two are ever
        // tuned to cross, which would otherwise disarm on the same frame it armed.
        float releaseAngle = Mathf.Min(F.flipRecoveryReleaseAngle, F.flipRecoveryAngleThreshold);

        // The previous condition also disarmed on
        // IsHoverGrounded, which broke recovery outright: a craft lying on its
        // flank still lands two hover rays on the floor anywhere between roughly
        // 40 and 88 degrees of tilt, so the righting torque revoked itself
        // MID-ROTATION, every time, at ~84 degrees. Past 90 there are no ray hits
        // at all, so there is no lift and ApplyLevelingTorque contributes zero
        // (baseStrength is 0 when not hover-grounded); the craft coasted over on
        // momentum and fell back. Measured limit cycle: 102 -> 127 -> 84 -> 134,
        // never recovering, with the timer visibly counting and never completing.
        //
        // Removing the term does not let righting fight a craft riding a wall or
        // a loop, because arming already requires a full flipRecoveryDelay of
        // ground contact under flipRecoverySpeedThreshold. Anything carrying
        // enough momentum to hold a loop is an order of magnitude above that gate
        // and can never arm. A craft genuinely pinned at >80 degrees and under
        // 2 m/s is stuck by definition and should be righted.
        if (tiltAngle < releaseAngle)
            rightingAuthorized = false;

        if (rightingAuthorized && F.flipRecoveryTorque > 0f)
        {
            Vector3 torqueAxis = Vector3.Cross(transform.up, Vector3.up);

            if (torqueAxis.sqrMagnitude < 0.001f)
                torqueAxis = transform.right * 0.1f;

            rb.AddTorque(torqueAxis.normalized * F.flipRecoveryTorque, ForceMode.Acceleration);
        }

        // --- Upright unstick path ---
        // Gate: contacting ground AND upright AND vertically settled.
        // Horizontal speed is intentionally NOT gated (unstick must work after
        // any landing). Vertical speed IS gated: a genuinely stuck craft has
        // settled vertically, while a dynamic belly scrape has not. Without
        // this gate, sustained scrapes received a lift pulse every
        // unstickRecoveryDelay on top of working hover springs (a ~5Hz kick
        // train at default tuning).
        bool verticallySettled = Mathf.Abs(rb.linearVelocity.y) < F.unstickMaxVerticalSpeed;

        // IsContactingFloor, not IsContactingGround: rubbing a wall is not being
        // stuck to it, and must not arm the lift.
        if (IsContactingFloor && !isFlipped && verticallySettled)
        {
            unstickTimer += Time.fixedDeltaTime;

            if (unstickTimer >= F.unstickRecoveryDelay)
            {
                unstickTimer = 0f;

                unstickForceDir = floorContactNormal.sqrMagnitude > 0.01f
                    ? floorContactNormal
                    : Vector3.up;

                unstickForceTimer      = F.unstickLiftDuration;
                unstickFiredFlashTimer = 0.5f;
                firedFlashWasFlip      = false;

                if (ShouldDrawDebug)
                    Debug.DrawRay(transform.position, unstickForceDir * 2f, Color.cyan);
            }
        }
        else
        {
            unstickTimer = 0f;
        }

        // --- Flip recovery path ---
        // Gate: contacting ground AND flipped AND slow.
        if (IsContactingGround && isFlipped && isSlow)
        {
            flipTimer += Time.fixedDeltaTime;

            if (flipTimer >= F.flipRecoveryDelay)
            {
                flipTimer          = 0f;
                rightingAuthorized = true;

                unstickFiredFlashTimer = 0.5f;
                firedFlashWasFlip      = true;

                if (ShouldDrawDebug)
                    Debug.DrawRay(transform.position, Vector3.up * 2f, Color.magenta);
            }
        }
        else
        {
            flipTimer = 0f;
        }
    }

    // -------------------------------------------------------------------------
    // 🚫 Downed State (control lockout)
    // -------------------------------------------------------------------------
    /// <summary>
    /// Two terms, ORed, and each covers a case the other cannot.
    ///
    ///   Resting term (contact AND flipped): locks control the instant the craft
    ///   comes to rest inverted, before flipRecoveryDelay has elapsed, so the
    ///   window before recovery arms is not a free escape.
    ///
    ///   Latch term (rightingAuthorized): holds the lockout through the rest of
    ///   the recovery even after ground contact is lost. Load-bearing: as the
    ///   righting torque swings the craft back through ~82 degrees the hover
    ///   springs re-acquire and fling it clear of the ground, and without this
    ///   term the lockout released mid-flip at 137 degrees of tilt and handed
    ///   back both air control and the air jump. Authority is only revoked when
    ///   tilt falls under flipRecoveryAngleThreshold, so this releases on being
    ///   upright, which is the intended contract.
    ///
    /// Why contact and not IsHoverGrounded: a craft hovering a wall or a loop is
    /// tilted past the threshold but never TOUCHES it, so contact stays false and
    /// it keeps full control. A craft lying on its flank does touch. IsHoverGrounded
    /// gets this exactly backwards -- on its flank the sensor rays point sideways
    /// and find nothing, so it reads as airborne.
    ///
    /// Deliberately NOT downed: a craft tumbling in mid-air after a hit. No
    /// contact and recovery has never armed, so air control still answers and a
    /// mid-air save stays available. The lockout is the price of coming to rest
    /// inverted, not of being knocked around.
    /// </summary>
    private void UpdateDownedState()
    {
        bool restingInverted = IsContactingGround
                            && Vector3.Angle(transform.up, Vector3.up) >= F.flipRecoveryAngleThreshold;

        IsDowned = recoveryEnabled && (rightingAuthorized || restingInverted);
    }

    // -------------------------------------------------------------------------
    // 📌 Sustained Unstick Lift
    // -------------------------------------------------------------------------
    /// <summary>
    /// Applies the unstick lift over a short window each FixedUpdate.
    /// Force is front-loaded: strongest on the first frame, tapering to zero.
    /// Reads as a gentle invisible lift rather than a visible punch.
    /// Mass independent so behavior is consistent across vehicles.
    /// </summary>
    private void ApplyUnstickForce()
    {
        if (unstickForceTimer <= 0f)
            return;

        float progress       = unstickForceTimer / Mathf.Max(0.01f, F.unstickLiftDuration);
        float forceMagnitude = F.unstickLiftForce * progress;

        rb.AddForce(unstickForceDir * forceMagnitude, ForceMode.Acceleration);

        unstickForceTimer = Mathf.Max(0f, unstickForceTimer - Time.fixedDeltaTime);
    }

#if UNITY_EDITOR
    // -------------------------------------------------------------------------
    // 🎨 Unstick Debug Gizmos
    // -------------------------------------------------------------------------
    /// <summary>
    /// Live recovery state in the Scene view during play.
    ///
    ///   Hover point spheres: green (ray hitting ground) / red (no hit)
    ///   Cyan line:           floorContactNormal while touching floor (absent on walls)
    ///   Yellow bar + label:  unstickTimer progress toward unstickRecoveryDelay
    ///   Orange bar + label:  flipTimer progress toward flipRecoveryDelay
    ///   Magenta sphere:      recovery fired flash, visible for 0.5s
    /// </summary>
    private void OnDrawGizmos()
    {
        if (!ShouldDrawDebug || !Application.isPlaying)
            return;

        // unstickFiredFlashTimer is ticked in Update. Read-only here.

        // --- Hover point spheres ---
        if (hoverPoints != null)
        {
            foreach (Transform point in hoverPoints)
            {
                if (point == null) continue;
                Gizmos.color = IsHoverGrounded ? Color.green : Color.red;
                Gizmos.DrawWireSphere(point.position, 0.15f);
            }
        }

        // --- Ceiling duck (blue). Only drawn while actually ducking, so seeing it
        //     at all means low geometry is squatting the craft. If a craft is
        //     pinned and this is NOT showing, the duck is not engaging and the
        //     ceiling probe is the thing to look at. ---
        if (profile != null && _effectiveHoverHeight < F.hoverHeight - 0.01f)
        {
            Gizmos.color = new Color(0.3f, 0.6f, 1f);
            Gizmos.DrawRay(transform.position, transform.up * F.ceilingClearance);
            Gizmos.DrawWireSphere(transform.position + transform.up * F.ceilingClearance, 0.2f);

            UnityEditor.Handles.color = Gizmos.color;
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 3.2f,
                $"DUCKING  ride {_effectiveHoverHeight:F2} / {F.hoverHeight:F2}m"
            );
        }

        // --- Floor contact normal (cyan). Absent while only touching walls,
        //     which is itself the useful signal: no floor contact, no unstick. ---
        if (IsContactingFloor && floorContactNormal.sqrMagnitude > 0.01f)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawRay(transform.position, floorContactNormal * 1.5f);
            Gizmos.DrawWireSphere(transform.position + floorContactNormal * 1.5f, 0.1f);
        }

        // --- Upright unstick timer bar (yellow) ---
        if (IsContactingFloor && unstickTimer > 0f && profile != null)
        {
            float progress  = Mathf.Clamp01(unstickTimer / Mathf.Max(0.01f, F.unstickRecoveryDelay));
            Gizmos.color    = Color.Lerp(Color.yellow, new Color(1f, 0.8f, 0f), progress);
            Vector3 start   = transform.position + Vector3.up * 0.3f;
            Vector3 end     = start + transform.right * (progress * 2f);
            Gizmos.DrawLine(start, end);
            Gizmos.DrawWireSphere(end, 0.08f);

            UnityEditor.Handles.color = Gizmos.color;
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 1.5f,
                $"Unstick {unstickTimer:F2} / {F.unstickRecoveryDelay:F2}s"
            );
        }

        // --- Flip recovery diagnostics ---
        // Shown whenever the craft is tilted enough to care, not only while the timer is
        // already running, because the interesting failure is the timer NOT starting (or
        // resetting). Every gate is printed with its live value so a stuck craft says
        // exactly which condition is blocking it instead of leaving it to guesswork.
        if (profile != null && Application.isPlaying)
        {
            float tiltNow = Vector3.Angle(transform.up, Vector3.up);
            if (tiltNow >= F.flipRecoveryAngleThreshold * 0.5f)
            {
                bool  flippedNow = tiltNow >= F.flipRecoveryAngleThreshold;
                float speedNow   = rb.linearVelocity.magnitude;
                bool  slowNow    = speedNow < F.flipRecoverySpeedThreshold;
                bool  contactNow = IsContactingGround;

                // Authority is reported FIRST. The old chain tested the arming
                // gates before rightingAuthorized, so a craft that was already
                // authorized and applying righting torque printed
                // "BLOCKED: moving too fast" -- the arming gates are irrelevant
                // once authority is held, and that label sent a whole debugging
                // session after the wrong condition. The gates below describe
                // only what is stopping the timer from ARMING.
                string blocker =
                      rightingAuthorized ? "RIGHTING"
                    : !contactNow  ? "BLOCKED: no ground contact"
                    : !flippedNow  ? "BLOCKED: tilt below threshold"
                    : !slowNow     ? "BLOCKED: moving too fast"
                    : "counting up";

                UnityEditor.Handles.color = rightingAuthorized ? Color.magenta
                                          : (contactNow && flippedNow && slowNow) ? new Color(1f, 0.6f, 0f)
                                          : Color.red;
                UnityEditor.Handles.Label(
                    transform.position + Vector3.up * 2.6f,
                    $"FLIP {flipTimer:F2}/{F.flipRecoveryDelay:F2}s  {blocker}\n" +
                    $"  tilt {tiltNow:F0}deg (need {F.flipRecoveryAngleThreshold:F0})\n" +
                    $"  speed {speedNow:F2} (need < {F.flipRecoverySpeedThreshold:F2})\n" +
                    $"  contact {contactNow}   hoverGrounded {IsHoverGrounded}   authorized {rightingAuthorized}"
                );
            }
        }

        // --- Flip recovery timer bar (orange) ---
        if (IsContactingGround && flipTimer > 0f && profile != null)
        {
            float progress  = Mathf.Clamp01(flipTimer / Mathf.Max(0.01f, F.flipRecoveryDelay));
            Gizmos.color    = Color.Lerp(new Color(1f, 0.4f, 0f), Color.red, progress);
            Vector3 start   = transform.position + Vector3.up * 0.6f;
            Vector3 end     = start + transform.right * (progress * 2f);
            Gizmos.DrawLine(start, end);
            Gizmos.DrawWireSphere(end, 0.08f);

            UnityEditor.Handles.color = Gizmos.color;
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 2.0f,
                $"Flip {flipTimer:F2} / {F.flipRecoveryDelay:F2}s"
            );
        }

        // --- Fired-impulse flash ---
        if (unstickFiredFlashTimer > 0f)
        {
            float alpha     = unstickFiredFlashTimer / 0.5f;
            Gizmos.color    = new Color(1f, 0f, 1f, alpha);
            Gizmos.DrawWireSphere(transform.position, 0.6f);
            Gizmos.DrawWireSphere(transform.position, 0.9f);

            UnityEditor.Handles.color = new Color(1f, 0f, 1f, alpha);
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 2f,
                firedFlashWasFlip ? "FLIP RECOVERY ARMED" : "UNSTICK FIRED"
            );
        }
    }
#endif
}
