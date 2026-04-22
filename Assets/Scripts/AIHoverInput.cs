using UnityEngine;

/// <summary>
/// AIHoverInput v2.0
/// -----------------
/// AI implementation of IHoverInputProvider. Drives the hover vehicle controller
/// using the same interface as PlayerHoverInput — no changes to any other system required.
///
/// Attach this to an AI vehicle prefab in place of PlayerHoverInput.
///
/// v2.0 changes:
///   * Dynamic target selection — finds the nearest living enemy via VehicleHealth
///     instead of a single hardcoded playerTransform reference. AI vehicles now
///     fight each other in multi-vehicle matches, not just the player.
///   * Target rescan runs every targetRescanInterval seconds to avoid FindObjectsByType
///     every frame. Cached target is validated each tick (null/dead check).
///   * Flee now picks a waypoint in the safest direction within the roam area
///     instead of driving straight backward from the threat — avoids wall-pinning.
///   * turnResponsiveness scaling simplified — no framerate compensation needed.
///
/// Behavior states:
///   Roam  — Default. Picks random waypoints within a bounding area and cruises toward them.
///            Fires at the nearest enemy if within range and within firing arc.
///   Flee  — Triggered by incoming damage or health dropping below fleeHealthThreshold.
///            Picks a waypoint away from the threat and boosts toward it.
///            Returns to Roam after fleeDuration seconds if health is above threshold.
///   Dead  — All outputs zeroed. Entered when VehicleHealth reports death.
///            Vehicle coasts to a stop naturally via propulsion drag.
///
/// Not yet implemented (deferred):
///   * Strafe mode (StrafeHeld, StrafeX always false/0)
///   * Drift
///   * Jump
///   * Weapon cycling
/// </summary>
[RequireComponent(typeof(VehicleHealth))]
public class AIHoverInput : MonoBehaviour, IHoverInputProvider
{
    // -------------------------------------------------------------------------
    // IHoverInputProvider — outputs written each Update tick
    // -------------------------------------------------------------------------
    public float   ThrottleInput    { get; private set; }
    public float   TurnInput        { get; private set; }
    public float   CameraLookY      { get; private set; } // always 0 — AI has no camera
    public Vector2 AimInput         { get; private set; } // always zero
    public bool    StrafeHeld       { get; private set; } // always false — deferred
    public float   StrafeX          { get; private set; } // always 0   — deferred
    public bool    Boost            { get; private set; }
    public bool    Drift            { get; private set; } // always false — deferred
    public bool    Jump             { get; private set; } // always false — deferred
    public bool    FireHeld         { get; private set; }
    public bool    FirePressed      { get; private set; } // always false — AI uses FireHeld only
    public bool    CycleWeaponNext  { get; private set; } // always false — deferred
    public bool    CycleWeaponPrev  { get; private set; } // always false — deferred

    // -------------------------------------------------------------------------
    // 🎯 Target Selection
    // -------------------------------------------------------------------------
    [Header("🎯 Target Selection")]
    [Tooltip("How often (seconds) to rescan for the nearest enemy. " +
             "Avoids FindObjectsByType every frame. Recommended: 0.5–1.0.")]
    [Min(0.1f)]
    [SerializeField] private float targetRescanInterval = 0.5f;

    // -------------------------------------------------------------------------
    // 🗺 Roam
    // -------------------------------------------------------------------------
    [Header("🗺 Roam")]
    [Tooltip("Center of the roam area in world space.")]
    [SerializeField] private Vector3 roamCenter = Vector3.zero;

    [Tooltip("Radius of the roam area around roamCenter.")]
    [Min(5f)]
    [SerializeField] private float roamRadius = 40f;

    [Tooltip("Distance from waypoint at which the AI considers it reached and picks a new one.")]
    [Min(1f)]
    [SerializeField] private float waypointArrivalDistance = 5f;

    [Tooltip("Maximum seconds before a new waypoint is picked regardless of arrival. " +
             "Prevents the AI getting stuck on an unreachable point.")]
    [Min(2f)]
    [SerializeField] private float waypointTimeout = 8f;

    // -------------------------------------------------------------------------
    // 🔫 Firing
    // -------------------------------------------------------------------------
    [Header("🔫 Firing")]
    [Tooltip("Distance (m) within which the AI will fire at the current target.")]
    [Min(1f)]
    [SerializeField] private float fireRange = 30f;

    [Tooltip("Dot product threshold for firing. 1 = dead ahead only, 0 = 90 degrees. " +
             "Recommended: 0.7 (roughly 45 degree half-cone).")]
    [Range(0f, 1f)]
    [SerializeField] private float fireArcDot = 0.7f;

    // -------------------------------------------------------------------------
    // 🏃 Flee
    // -------------------------------------------------------------------------
    [Header("🏃 Flee")]
    [Tooltip("Health fraction (0..1) below which Flee state is entered and held.")]
    [Range(0f, 1f)]
    [SerializeField] private float fleeHealthThreshold = 0.3f;

    [Tooltip("Seconds to remain in Flee state after taking damage before returning to Roam. " +
             "Ignored if health is still below fleeHealthThreshold.")]
    [Min(0.5f)]
    [SerializeField] private float fleeDuration = 3f;

    // -------------------------------------------------------------------------
    // 🕹 Steering
    // -------------------------------------------------------------------------
    [Header("🕹 Steering")]
    [Tooltip("How sharply the AI turns toward its target heading. " +
             "This is a signed dot-to-turn multiplier — 1 is fully responsive, lower values are lazier.")]
    [Range(0.1f, 1f)]
    [SerializeField] private float turnResponsiveness = 0.8f;

    // -------------------------------------------------------------------------
    // 🧱 Obstacle Avoidance
    // -------------------------------------------------------------------------
    [Header("🧱 Obstacle Avoidance")]
    [Tooltip("Maximum distance (m) to cast rays ahead for wall detection.")]
    [Min(1f)]
    [SerializeField] private float avoidanceRayDistance = 12f;

    [Tooltip("Number of rays in the forward fan. Spread symmetrically across avoidanceFanAngle. " +
             "Odd numbers recommended so there is always a center ray. Minimum 3.")]
    [Range(3, 9)]
    [SerializeField] private int avoidanceRayCount = 5;

    [Tooltip("Half-angle (degrees) of the ray fan. Rays spread from -fanAngle to +fanAngle.")]
    [Range(10f, 60f)]
    [SerializeField] private float avoidanceFanAngle = 35f;

    [Tooltip("Layers treated as obstacles. Default: everything.")]
    [SerializeField] private LayerMask obstacleMask = ~0;

    [Tooltip("When the closest obstacle is at or below this distance, avoidance steering " +
             "fully overrides waypoint steering and throttle drops to minimum.")]
    [Min(0.5f)]
    [SerializeField] private float avoidanceUrgentDistance = 4f;

    [Tooltip("Minimum throttle when obstacle avoidance is at maximum urgency. " +
             "Keeps the vehicle creeping forward so it can steer out. 0 = full stop allowed.")]
    [Range(0f, 0.5f)]
    [SerializeField] private float avoidanceMinThrottle = 0.15f;

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------
    private enum AIState { Roam, Flee, Dead }
    private AIState     _state = AIState.Roam;

    private VehicleHealth _health;
    private Vector3       _currentWaypoint;
    private float         _waypointTimer;
    private float         _fleeTimer;

    /// <summary>Current combat target. Dynamically selected, may be null.</summary>
    private VehicleHealth _currentTarget;

    /// <summary>Countdown until next target rescan.</summary>
    private float _rescanTimer;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------
    private void Awake()
    {
        _health = GetComponent<VehicleHealth>();
        _health.OnDamaged += HandleDamaged;
        _health.OnDeath   += HandleDeath;

        PickNewWaypoint();
        _rescanTimer = 0f; // scan immediately on first frame
    }

    private void OnDestroy()
    {
        if (_health != null)
        {
            _health.OnDamaged -= HandleDamaged;
            _health.OnDeath   -= HandleDeath;
        }
    }

    private void Update()
    {
        TickTargetSelection();

        switch (_state)
        {
            case AIState.Roam: UpdateRoam(); break;
            case AIState.Flee: UpdateFlee(); break;
            case AIState.Dead: ZeroAllOutputs(); break;
        }
    }

    // -------------------------------------------------------------------------
    // 🎯 Target Selection
    // -------------------------------------------------------------------------
    /// <summary>
    /// Periodically rescans all living VehicleHealth instances to find the nearest
    /// enemy. Between rescans, validates the cached target is still alive.
    /// FindObjectsByType is gated behind a timer so it doesn't run every frame.
    /// </summary>
    private void TickTargetSelection()
    {
        // Quick validation: clear dead/destroyed targets immediately
        if (_currentTarget != null && !_currentTarget.IsAlive)
            _currentTarget = null;

        _rescanTimer -= Time.deltaTime;
        if (_rescanTimer > 0f)
            return;

        _rescanTimer = targetRescanInterval;

        var allVehicles = FindObjectsByType<VehicleHealth>(FindObjectsSortMode.None);
        VehicleHealth bestTarget = null;
        float bestDistSq = float.MaxValue;

        foreach (var vehicle in allVehicles)
        {
            // Skip self
            if (vehicle == _health)
                continue;

            // Skip dead vehicles
            if (!vehicle.IsAlive)
                continue;

            float distSq = (vehicle.transform.position - transform.position).sqrMagnitude;
            if (distSq < bestDistSq)
            {
                bestDistSq = distSq;
                bestTarget = vehicle;
            }
        }

        _currentTarget = bestTarget;
    }

    /// <summary>
    /// Returns the current target's transform, or null if no target.
    /// </summary>
    private Transform TargetTransform => _currentTarget != null ? _currentTarget.transform : null;

    // -------------------------------------------------------------------------
    // State: Roam
    // -------------------------------------------------------------------------
    private void UpdateRoam()
    {
        AdvanceWaypointTimer();

        Vector3 toWaypoint = _currentWaypoint - transform.position;
        toWaypoint.y = 0f;

        if (toWaypoint.magnitude < waypointArrivalDistance)
            PickNewWaypoint();

        SteerToward(_currentWaypoint);
        ThrottleInput = 1f;
        Boost         = false;

        // Obstacle avoidance — override steering and throttle when walls are near
        ApplyObstacleAvoidance();

        UpdateFiring();
    }

    // -------------------------------------------------------------------------
    // State: Flee
    // -------------------------------------------------------------------------
    private void UpdateFlee()
    {
        // Tick flee timer — return to roam when it expires AND health is above threshold
        _fleeTimer -= Time.deltaTime;
        if (_fleeTimer <= 0f && _health.HealthNormalized > fleeHealthThreshold)
        {
            _state = AIState.Roam;
            PickNewWaypoint();
            return;
        }

        // Pick a flee waypoint away from the threat, clamped to the roam area.
        // If no target, just roam to the current waypoint.
        if (TargetTransform != null)
        {
            // Only re-pick if we've arrived at (or are close to) the current flee waypoint
            Vector3 toWaypoint = _currentWaypoint - transform.position;
            toWaypoint.y = 0f;
            if (toWaypoint.magnitude < waypointArrivalDistance)
                PickFleeWaypoint(TargetTransform.position);

            SteerToward(_currentWaypoint);
        }

        ThrottleInput = 1f;
        Boost         = true;
        FireHeld      = false; // don't fire while fleeing

        // Obstacle avoidance — even while fleeing, don't drive into walls
        ApplyObstacleAvoidance();
    }

    // -------------------------------------------------------------------------
    // Steering helper
    // -------------------------------------------------------------------------
    /// <summary>
    /// Computes TurnInput by measuring how far the target is from dead ahead.
    /// Uses signed cross product on the horizontal plane — no Atan2 needed.
    /// The cross product naturally produces values in [-1, 1] for targets within
    /// 90 degrees. turnResponsiveness scales the result for tighter or lazier
    /// tracking. No framerate compensation needed — Propulsion applies the
    /// torque in FixedUpdate with its own timestep.
    /// </summary>
    private void SteerToward(Vector3 worldTarget)
    {
        Vector3 toTarget = (worldTarget - transform.position);
        toTarget.y = 0f;
        if (toTarget.sqrMagnitude < 0.01f) return;
        toTarget.Normalize();

        Vector3 forward = transform.forward;
        forward.y = 0f;
        forward.Normalize();

        // Signed angle: positive = target is to the right, negative = left
        float cross = Vector3.Cross(forward, toTarget).y;
        TurnInput = Mathf.Clamp(cross * turnResponsiveness, -1f, 1f);
    }

    // -------------------------------------------------------------------------
    // 🧱 Obstacle Avoidance
    // -------------------------------------------------------------------------
    /// <summary>
    /// Casts a fan of rays forward. When obstacles are detected, blends a
    /// steering correction into TurnInput and reduces ThrottleInput proportionally
    /// to the closest hit distance.
    ///
    /// Each ray that hits contributes a "steer away" impulse weighted by proximity
    /// (closer hits dominate). The center ray uses the wall normal to decide
    /// direction; side rays steer away from their side of the fan.
    ///
    /// Called after SteerToward so the waypoint heading is already in TurnInput.
    /// Avoidance adds on top — at high urgency it fully overrides the waypoint steer.
    /// </summary>
    private void ApplyObstacleAvoidance()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f;

        float closestDist   = avoidanceRayDistance;
        float weightedSteer = 0f;
        float totalWeight   = 0f;

        for (int i = 0; i < avoidanceRayCount; i++)
        {
            float t     = avoidanceRayCount == 1 ? 0.5f : (float)i / (avoidanceRayCount - 1);
            float angle = Mathf.Lerp(-avoidanceFanAngle, avoidanceFanAngle, t);

            Vector3 dir = Quaternion.Euler(0f, angle, 0f) * transform.forward;

            if (!Physics.Raycast(origin, dir, out RaycastHit hit, avoidanceRayDistance, obstacleMask))
                continue;

            float proximity = 1f - (hit.distance / avoidanceRayDistance);

            if (hit.distance < closestDist)
                closestDist = hit.distance;

            // Decide which way to steer away from this hit.
            // Side rays: steer opposite to the ray's side of the fan.
            // Center ray (angle near zero): use wall normal to pick the better direction.
            float steerAway;
            if (Mathf.Abs(angle) < 1f)
            {
                float normalCross = Vector3.Cross(transform.forward, hit.normal).y;
                steerAway = normalCross >= 0f ? 1f : -1f;
            }
            else
            {
                steerAway = angle < 0f ? 1f : -1f;
            }

            weightedSteer += steerAway * proximity;
            totalWeight   += proximity;
        }

        // No hits — no avoidance needed
        if (totalWeight < 0.001f)
            return;

        // Urgency ramps from 0 (hit at max range) to 1 (hit at urgent distance or closer)
        float urgency = Mathf.InverseLerp(avoidanceRayDistance, avoidanceUrgentDistance, closestDist);

        // Steering: blend avoidance into existing TurnInput. At full urgency, avoidance dominates.
        float avoidSteer = Mathf.Clamp(weightedSteer / totalWeight, -1f, 1f);
        TurnInput = Mathf.Clamp(Mathf.Lerp(TurnInput, avoidSteer, urgency), -1f, 1f);

        // Throttle: reduce proportionally so the AI slows near walls
        ThrottleInput = Mathf.Lerp(ThrottleInput, avoidanceMinThrottle, urgency);
    }

    // -------------------------------------------------------------------------
    // Firing helper
    // -------------------------------------------------------------------------
    private void UpdateFiring()
    {
        if (TargetTransform == null) { FireHeld = false; return; }

        float dist = Vector3.Distance(transform.position, TargetTransform.position);
        if (dist > fireRange)   { FireHeld = false; return; }

        Vector3 toTarget = (TargetTransform.position - transform.position).normalized;
        float   dot      = Vector3.Dot(transform.forward, toTarget);

        FireHeld = dot >= fireArcDot;
    }

    // -------------------------------------------------------------------------
    // Waypoint helpers
    // -------------------------------------------------------------------------
    private void PickNewWaypoint()
    {
        Vector2 randCircle = Random.insideUnitCircle * roamRadius;
        _currentWaypoint   = roamCenter + new Vector3(randCircle.x, 0f, randCircle.y);
        _waypointTimer     = 0f;
    }

    /// <summary>
    /// Picks a flee waypoint in the direction away from the threat, clamped to
    /// the roam area. Adds some randomness to prevent predictable straight-line
    /// retreat that pins the AI against walls.
    /// </summary>
    private void PickFleeWaypoint(Vector3 threatPosition)
    {
        Vector3 awayDir = (transform.position - threatPosition);
        awayDir.y = 0f;

        if (awayDir.sqrMagnitude < 0.01f)
            awayDir = transform.forward; // degenerate case: on top of threat
        else
            awayDir.Normalize();

        // Add random spread (up to 45 degrees each side) to avoid predictable lines
        float randomAngle = Random.Range(-45f, 45f);
        awayDir = Quaternion.Euler(0f, randomAngle, 0f) * awayDir;

        // Place waypoint at roam radius distance, then clamp to roam area
        Vector3 candidate = transform.position + awayDir * roamRadius * 0.7f;
        Vector3 offset    = candidate - roamCenter;
        offset.y = 0f;
        if (offset.magnitude > roamRadius)
            candidate = roamCenter + offset.normalized * roamRadius;

        _currentWaypoint = candidate;
        _waypointTimer   = 0f;
    }

    private void AdvanceWaypointTimer()
    {
        _waypointTimer += Time.deltaTime;
        if (_waypointTimer >= waypointTimeout)
            PickNewWaypoint();
    }

    // -------------------------------------------------------------------------
    // Event handlers
    // -------------------------------------------------------------------------
    private void HandleDamaged(float currentHealth, float maxHealth)
    {
        if (_state == AIState.Dead) return;

        _state     = AIState.Flee;
        _fleeTimer = fleeDuration;

        // Immediately pick a flee waypoint so we don't drive toward the threat
        if (TargetTransform != null)
            PickFleeWaypoint(TargetTransform.position);
    }

    private void HandleDeath()
    {
        _state = AIState.Dead;
        ZeroAllOutputs();
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------
    private void ZeroAllOutputs()
    {
        ThrottleInput   = 0f;
        TurnInput       = 0f;
        CameraLookY     = 0f;
        AimInput        = Vector2.zero;
        StrafeHeld      = false;
        StrafeX         = 0f;
        Boost           = false;
        Drift           = false;
        Jump            = false;
        FireHeld        = false;
        FirePressed     = false;
        CycleWeaponNext = false;
        CycleWeaponPrev = false;
    }

    // -------------------------------------------------------------------------
    // 🎨 Debug Gizmos
    // -------------------------------------------------------------------------
#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        // Roam area
        UnityEditor.Handles.color = new Color(0f, 1f, 1f, 0.15f);
        UnityEditor.Handles.DrawSolidDisc(roamCenter, Vector3.up, roamRadius);
        UnityEditor.Handles.color = Color.cyan;
        UnityEditor.Handles.DrawWireDisc(roamCenter, Vector3.up, roamRadius);

        if (!Application.isPlaying) return;

        // Current waypoint
        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(_currentWaypoint, 0.5f);
        Gizmos.DrawLine(transform.position, _currentWaypoint);

        // State label + current target
        string targetName = _currentTarget != null ? _currentTarget.name : "none";
        UnityEditor.Handles.color = Color.white;
        UnityEditor.Handles.Label(
            transform.position + Vector3.up * 3f,
            $"AI: {_state}  HP: {(_health != null ? _health.HealthNormalized * 100f : 0f):F0}%  Target: {targetName}"
        );

        // Line to current target
        if (_currentTarget != null)
        {
            Gizmos.color = FireHeld ? Color.red : new Color(1f, 0.5f, 0f, 0.4f);
            Gizmos.DrawLine(transform.position, _currentTarget.transform.position);
        }

        // Fire range
        Gizmos.color = FireHeld ? Color.red : new Color(1f, 0f, 0f, 0.2f);
        Gizmos.DrawWireSphere(transform.position, fireRange);

        // Obstacle avoidance rays
        Vector3 rayOrigin = transform.position + Vector3.up * 0.5f;
        for (int i = 0; i < avoidanceRayCount; i++)
        {
            float t     = avoidanceRayCount == 1 ? 0.5f : (float)i / (avoidanceRayCount - 1);
            float angle = Mathf.Lerp(-avoidanceFanAngle, avoidanceFanAngle, t);
            Vector3 dir = Quaternion.Euler(0f, angle, 0f) * transform.forward;

            if (Physics.Raycast(rayOrigin, dir, out RaycastHit hit, avoidanceRayDistance, obstacleMask))
            {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(rayOrigin, hit.point);
                Gizmos.DrawSphere(hit.point, 0.2f);
            }
            else
            {
                Gizmos.color = new Color(0f, 1f, 0f, 0.3f);
                Gizmos.DrawLine(rayOrigin, rayOrigin + dir * avoidanceRayDistance);
            }
        }
    }
#endif
}
