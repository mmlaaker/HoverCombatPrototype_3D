using UnityEngine;

/// <summary>
/// Drives all cosmetic particle VFX on the hover vehicle.
///
/// Three-tier system: idle, acceleration, and boost. Hover points and rear
/// exhaust particles blend smoothly between tiers based on throttle magnitude
/// and boost state. Side particles fire a burst on successful dodge.
///
/// Tier blending:
///   1. Throttle magnitude (smoothed) lerps between idle and accel tiers.
///   2. BoostLerp overrides toward boost tier on top of that.
///   Result = lerp( lerp(idle, accel, smoothedThrottle), boost, boostLerp )
///
/// Slot layout (inspector-assigned):
///   hoverPoints  -- one VFXSlot per thruster corner (expect 4)
///   rearExhaust  -- rear exhaust mounts (expect 2)
///   leftDodge    -- left side dodge mounts (expect 2)
///   rightDodge   -- right side dodge mounts (expect 2)
///
/// Each VFXSlot holds a ParticleSystem array, allowing multiple layers
/// (e.g. base glow + spark) per physical mount without hardcoding counts.
/// </summary>
public class HoverVehicleVFX : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // Tier definition
    // -------------------------------------------------------------------------
    [System.Serializable]
    public struct VFXTier
    {
        [Tooltip("Particles emitted per second.")]
        public float emissionRate;

        [Tooltip("Initial particle size.")]
        public float startSize;

        [Tooltip("Initial particle speed.")]
        public float startSpeed;

        [Tooltip("How long each particle lives (seconds). Affects trail length.")]
        public float startLifetime;
    }

    [System.Serializable]
    public class VFXSlot
    {
        public ParticleSystem[] particles;
    }

    // -------------------------------------------------------------------------
    // Slots
    // -------------------------------------------------------------------------
    [Header("Slots")]
    [SerializeField] private VFXSlot[] hoverPoints;
    [SerializeField] private VFXSlot[] rearExhaust;
    [SerializeField] private VFXSlot[] leftDodge;
    [SerializeField] private VFXSlot[] rightDodge;

    // -------------------------------------------------------------------------
    // Hover Point Tiers
    // -------------------------------------------------------------------------
    [Header("Hover Points -- Idle")]
    [SerializeField] private VFXTier hoverIdle = new VFXTier
        { emissionRate = 10f, startSize = 0.8f, startSpeed = 1f, startLifetime = 0.4f };

    [Header("Hover Points -- Acceleration")]
    [SerializeField] private VFXTier hoverAccel = new VFXTier
        { emissionRate = 25f, startSize = 1f, startSpeed = 2f, startLifetime = 0.5f };

    [Header("Hover Points -- Boost")]
    [SerializeField] private VFXTier hoverBoost = new VFXTier
        { emissionRate = 50f, startSize = 1.3f, startSpeed = 3.5f, startLifetime = 0.6f };

    // -------------------------------------------------------------------------
    // Rear Exhaust Tiers
    // -------------------------------------------------------------------------
    [Header("Rear Exhaust -- Idle")]
    [SerializeField] private VFXTier exhaustIdle = new VFXTier
        { emissionRate = 15f, startSize = 0.6f, startSpeed = 2f, startLifetime = 0.3f };

    [Header("Rear Exhaust -- Acceleration")]
    [SerializeField] private VFXTier exhaustAccel = new VFXTier
        { emissionRate = 40f, startSize = 0.9f, startSpeed = 4f, startLifetime = 0.4f };

    [Header("Rear Exhaust -- Boost")]
    [SerializeField] private VFXTier exhaustBoost = new VFXTier
        { emissionRate = 80f, startSize = 1.2f, startSpeed = 7f, startLifetime = 0.5f };

    // -------------------------------------------------------------------------
    // Blend
    // -------------------------------------------------------------------------
    [Header("Blend")]
    [Tooltip("How fast the throttle blend catches up to the actual input (lerp speed per second). " +
             "Higher = snappier response. Lower = smoother transitions.")]
    [Range(1f, 20f)]
    [SerializeField] private float throttleBlendSpeed = 8f;

    // -------------------------------------------------------------------------
    // Landing Dust
    // -------------------------------------------------------------------------
    // One-shot Instantiate rather than mounted slots: the WarFX prefabs are
    // authored as play-on-awake one-shots with auto-destruct, same pattern as
    // RocketProjectile's explosionPrefab.
    [Header("Landing Dust")]
    [Tooltip("One-shot dust prefab spawned at the ground on any hard landing. Try WFX_ExplosiveSmokeGround Small.")]
    [SerializeField] private GameObject landingDustPrefab;

    [Tooltip("Bigger one-shot spawned instead when severity meets Heavy Severity Threshold. Try WFX_ExplosiveSmokeGround Big.")]
    [SerializeField] private GameObject landingDustHeavyPrefab;

    [Tooltip("Severity (0..1) at or above which the heavy dust prefab is used instead of the standard one.")]
    [Range(0f, 1f)]
    [SerializeField] private float heavySeverityThreshold = 0.6f;

    [Tooltip("Layers used to find the ground point under the vehicle for dust placement. Match Foundation's Ground Layers.")]
    [SerializeField] private LayerMask dustGroundLayers = ~0;

    // -------------------------------------------------------------------------
    // Runtime
    // -------------------------------------------------------------------------
    private HoverController_Propulsion propulsion;
    private HoverController_Foundation foundation;
    private IHoverInputProvider        input;
    private float                      smoothedThrottle;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------
    private void Awake()
    {
        propulsion = GetComponent<HoverController_Propulsion>();
        foundation = GetComponent<HoverController_Foundation>();
        input      = GetComponent<IHoverInputProvider>();
    }

    private void OnEnable()
    {
        if (propulsion != null)
            propulsion.OnDodge += HandleDodge;

        if (foundation != null)
            foundation.OnHardLanding += HandleHardLanding;
    }

    private void OnDisable()
    {
        if (propulsion != null)
            propulsion.OnDodge -= HandleDodge;

        if (foundation != null)
            foundation.OnHardLanding -= HandleHardLanding;
    }

    private void Start()
    {
        // Hover points and rear exhaust run continuously.
        PlayAll(hoverPoints);
        PlayAll(rearExhaust);

        // Side dodge particles are burst-only -- stop them so they don't idle-emit.
        StopAll(leftDodge);
        StopAll(rightDodge);
    }

    private void Update()
    {
        // Smooth the throttle signal for visual blend.
        float rawThrottle = input != null ? Mathf.Abs(input.ThrottleInput) : 0f;
        smoothedThrottle  = Mathf.Lerp(smoothedThrottle, rawThrottle, throttleBlendSpeed * Time.deltaTime);

        float boost = propulsion != null ? propulsion.BoostLerp : 0f;

        // Blend: idle -> accel via throttle, then override toward boost.
        VFXTier hoverTier   = BlendTier(hoverIdle,   hoverAccel,   hoverBoost,   smoothedThrottle, boost);
        VFXTier exhaustTier = BlendTier(exhaustIdle,  exhaustAccel, exhaustBoost, smoothedThrottle, boost);

        ApplyTier(hoverPoints, hoverTier);
        ApplyTier(rearExhaust, exhaustTier);
    }

    // -------------------------------------------------------------------------
    // Dodge handler
    // -------------------------------------------------------------------------
    /// <summary>
    /// Called by Propulsion.OnDodge. localDir is the local-space dodge direction
    /// (x > 0 = right, x &lt; 0 = left, z &lt; 0 = backward).
    /// Plays the appropriate side particle systems.
    /// Backward dodges play both sides.
    /// </summary>
    private void HandleDodge(Vector3 localDir)
    {
        if (localDir.x > 0.1f)
        {
            PlayAll(rightDodge);
        }
        else if (localDir.x < -0.1f)
        {
            PlayAll(leftDodge);
        }
        else
        {
            PlayAll(leftDodge);
            PlayAll(rightDodge);
        }
    }

    // -------------------------------------------------------------------------
    // Hard landing handler
    // -------------------------------------------------------------------------
    /// <summary>
    /// Called by Foundation.OnHardLanding. Spawns a one-shot dust burst at the
    /// ground point below the vehicle; heavy prefab at high severity.
    /// </summary>
    private void HandleHardLanding(float severity)
    {
        GameObject prefab = severity >= heavySeverityThreshold && landingDustHeavyPrefab != null
            ? landingDustHeavyPrefab
            : landingDustPrefab;

        if (prefab == null)
            return;

        Vector3 spawnPos = Physics.Raycast(transform.position, Vector3.down, out RaycastHit hit, 15f,
                                           dustGroundLayers, QueryTriggerInteraction.Ignore)
            ? hit.point
            : transform.position + Vector3.down * 3f;

        Instantiate(prefab, spawnPos, Quaternion.identity);
    }

    // -------------------------------------------------------------------------
    // Tier blending
    // -------------------------------------------------------------------------
    private static VFXTier BlendTier(VFXTier idle, VFXTier accel, VFXTier boost, float throttle, float boostLerp)
    {
        // First blend: idle -> accel based on throttle.
        VFXTier throttled;
        throttled.emissionRate = Mathf.Lerp(idle.emissionRate, accel.emissionRate, throttle);
        throttled.startSize    = Mathf.Lerp(idle.startSize,    accel.startSize,    throttle);
        throttled.startSpeed   = Mathf.Lerp(idle.startSpeed,   accel.startSpeed,   throttle);
        throttled.startLifetime = Mathf.Lerp(idle.startLifetime, accel.startLifetime, throttle);

        // Second blend: throttled -> boost based on boostLerp.
        VFXTier result;
        result.emissionRate  = Mathf.Lerp(throttled.emissionRate,  boost.emissionRate,  boostLerp);
        result.startSize     = Mathf.Lerp(throttled.startSize,     boost.startSize,     boostLerp);
        result.startSpeed    = Mathf.Lerp(throttled.startSpeed,    boost.startSpeed,    boostLerp);
        result.startLifetime = Mathf.Lerp(throttled.startLifetime, boost.startLifetime, boostLerp);

        return result;
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------
    private static void ApplyTier(VFXSlot[] slots, VFXTier tier)
    {
        if (slots == null) return;
        foreach (var slot in slots)
        {
            if (slot?.particles == null) continue;
            foreach (var ps in slot.particles)
            {
                if (ps == null) continue;
                var em   = ps.emission;
                var main = ps.main;
                em.rateOverTime    = tier.emissionRate;
                main.startSize     = tier.startSize;
                main.startSpeed    = tier.startSpeed;
                main.startLifetime = tier.startLifetime;
            }
        }
    }

    private static void PlayAll(VFXSlot[] slots)
    {
        if (slots == null) return;
        foreach (var slot in slots)
        {
            if (slot?.particles == null) continue;
            foreach (var ps in slot.particles)
                if (ps != null) ps.Play();
        }
    }

    private static void StopAll(VFXSlot[] slots)
    {
        if (slots == null) return;
        foreach (var slot in slots)
        {
            if (slot?.particles == null) continue;
            foreach (var ps in slot.particles)
                if (ps != null) ps.Stop(true, ParticleSystemStopBehavior.StopEmitting);
        }
    }

#if UNITY_EDITOR
    /// <summary>
    /// Applies a specific tier to all continuous particle systems (hover points + rear exhaust).
    /// Called by the custom editor preview buttons in edit mode.
    /// </summary>
    public void EditorPreviewTier(int tierIndex)
    {
        VFXTier hoverTier, exhaustTier;
        switch (tierIndex)
        {
            case 0: hoverTier = hoverIdle;  exhaustTier = exhaustIdle;  break;
            case 1: hoverTier = hoverAccel; exhaustTier = exhaustAccel; break;
            case 2: hoverTier = hoverBoost; exhaustTier = exhaustBoost; break;
            default: return;
        }
        ApplyTier(hoverPoints, hoverTier);
        ApplyTier(rearExhaust, exhaustTier);
    }
#endif
}
