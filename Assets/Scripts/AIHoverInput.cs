using UnityEngine;

/// <summary>
/// AIHoverInput v1.0
/// -----------------
/// AI implementation of IHoverInputProvider. Drives the hover vehicle controller
/// using the same interface as PlayerHoverInput — no changes to any other system required.
///
/// Attach this to an AI vehicle prefab in place of PlayerHoverInput.
///
/// Behavior states:
///   Roam  — Default. Picks random waypoints within a bounding area and cruises toward them.
///            Fires at the player if within range and within firing arc.
///   Flee  — Triggered by incoming damage or health dropping below fleeHealthThreshold.
///            Drives directly away from the player with boost active.
///            Returns to Roam after fleeDuration seconds if health is above threshold.
///   Dead  — All outputs zeroed. Entered when VehicleHealth reports death.
///            Vehicle coasts to a stop naturally via propulsion drag.
///
/// Not yet implemented (deferred):
///   • Strafe mode (StrafeHeld, StrafeX always false/0)
///   • Drift
///   • Jump
///   • Weapon cycling
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
    // 🎯 Target
    // -------------------------------------------------------------------------
    [Header("🎯 Target")]
    [Tooltip("Assign the player vehicle's Transform here.")]
    [SerializeField] private Transform playerTransform;

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
    [Tooltip("Distance (m) within which the AI will fire at the player.")]
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
    // Runtime state
    // -------------------------------------------------------------------------
    private enum AIState { Roam, Flee, Dead }
    private AIState     _state = AIState.Roam;

    private VehicleHealth _health;
    private Vector3       _currentWaypoint;
    private float         _waypointTimer;
    private float         _fleeTimer;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------
    private void Awake()
    {
        _health = GetComponent<VehicleHealth>();
        _health.OnDamaged += HandleDamaged;
        _health.OnDeath   += HandleDeath;

        if (playerTransform == null)
            Debug.LogWarning(
                $"[AIHoverInput] '{name}': playerTransform is not assigned. " +
                "AI will roam but cannot react to the player.", this);

        PickNewWaypoint();
    }

    private void OnDestroy()
    {
        // Unsubscribe defensively in case the object is destroyed before health fires
        if (_health != null)
        {
            _health.OnDamaged -= HandleDamaged;
            _health.OnDeath   -= HandleDeath;
        }
    }

    private void Update()
    {
        switch (_state)
        {
            case AIState.Roam: UpdateRoam(); break;
            case AIState.Flee: UpdateFlee(); break;
            case AIState.Dead: ZeroAllOutputs(); break;
        }
    }

    // -------------------------------------------------------------------------
    // State: Roam
    // -------------------------------------------------------------------------
    private void UpdateRoam()
    {
        AdvanceWaypointTimer();

        Vector3 toWaypoint = _currentWaypoint - transform.position;
        toWaypoint.y = 0f; // ignore vertical — let Foundation handle hover height

        if (toWaypoint.magnitude < waypointArrivalDistance)
            PickNewWaypoint();

        SteerToward(_currentWaypoint);
        ThrottleInput = 1f;
        Boost         = false;

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

        if (playerTransform != null)
        {
            // Drive away from player — invert the player direction
            Vector3 fleeTarget = transform.position +
                                 (transform.position - playerTransform.position).normalized * 20f;
            SteerToward(fleeTarget);
        }

        ThrottleInput = 1f;
        Boost         = true;
        FireHeld      = false; // don't fire while fleeing
    }

    // -------------------------------------------------------------------------
    // Steering helper
    // -------------------------------------------------------------------------
    /// <summary>
    /// Computes TurnInput by measuring how far the target is from dead ahead.
    /// Uses signed cross product on the horizontal plane — no Atan2 needed.
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
        TurnInput = Mathf.Clamp(cross * turnResponsiveness / Time.deltaTime * Time.fixedDeltaTime,
                                -1f, 1f);
    }

    // -------------------------------------------------------------------------
    // Firing helper
    // -------------------------------------------------------------------------
    private void UpdateFiring()
    {
        if (playerTransform == null) { FireHeld = false; return; }

        float dist = Vector3.Distance(transform.position, playerTransform.position);
        if (dist > fireRange)   { FireHeld = false; return; }

        Vector3 toPlayer = (playerTransform.position - transform.position).normalized;
        float   dot      = Vector3.Dot(transform.forward, toPlayer);

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

        // Enter flee on any damage hit, or if health is critically low
        _state     = AIState.Flee;
        _fleeTimer = fleeDuration;
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

        // State label
        UnityEditor.Handles.color = Color.white;
        UnityEditor.Handles.Label(
            transform.position + Vector3.up * 3f,
            $"AI: {_state}  HP: {(_health != null ? _health.HealthNormalized * 100f : 0f):F0}%"
        );

        // Fire range
        if (playerTransform != null)
        {
            Gizmos.color = FireHeld ? Color.red : new Color(1f, 0f, 0f, 0.2f);
            Gizmos.DrawWireSphere(transform.position, fireRange);
        }
    }
#endif
}
