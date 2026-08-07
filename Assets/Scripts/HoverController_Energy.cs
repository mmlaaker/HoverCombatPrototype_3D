using System;
using UnityEngine;

/// <summary>
/// HoverController_Energy v1.0
///
/// Shared resource pool for non-damaging mobility abilities (boost, jump, dodge,
/// shield, EMP). Weapons do NOT spend energy; they only check IsEmpFrozen.
///
/// Tuning lives on a VehicleTuningProfile asset (profile.energy). Runtime state
/// (Energy value, EMP freeze timer, regen lockout) stays on the component.
///
/// Contracts:
///   Continuous abilities call TryConsume every frame they are active.
///   Instantaneous abilities call TryConsume once on activation.
///   Energy doesn't care which. It just debits and gates.
///
///   Any TryConsume call (success OR failure) resets the regen lockout. This is
///   intentional: if you're trying to spend, you're "demanding" from the pool, and
///   regen is held back as a consequence. A player who mashes boost at zero energy
///   prolongs their own recovery. Panic spending is punished.
///
///   EMP freeze is additive: multiple hits stack freeze duration. Both consumption
///   and regen are suspended for the duration.
///
/// Owns no physics. Pure resource manager.
/// </summary>
public class HoverController_Energy : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // 📦 Tuning Profile
    // -------------------------------------------------------------------------
    [Header("📦 Tuning")]
    [Tooltip("Vehicle tuning profile (shared SO). All numeric tuning lives here. Required.")]
    [SerializeField] private VehicleTuningProfile profile;

    /// <summary>Shorthand for profile.energy. Used at every read site below.</summary>
    private EnergyTuning E => profile.energy;

    // -------------------------------------------------------------------------
    // 📢 Events
    // -------------------------------------------------------------------------

    /// <summary>Fired when the pool reaches zero for the first time after being non-empty.</summary>
    public event Action OnEnergyDepleted;

    /// <summary>Fired when regen begins after the lockout delay expires.</summary>
    public event Action OnRegenStarted;

    /// <summary>Fired when an EMP freeze is applied. Parameter is total freeze duration remaining.</summary>
    public event Action<float> OnEmpFreezeApplied;

    /// <summary>Fired when an EMP freeze expires.</summary>
    public event Action OnEmpFreezeLifted;

    // -------------------------------------------------------------------------
    // 🛠 Debug
    // -------------------------------------------------------------------------
    [Header("🛠 Debug")]
    [Tooltip("Optional global debug toggle. When assigned, controls gizmo visibility.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    // -------------------------------------------------------------------------
    // Public read-only state
    // -------------------------------------------------------------------------

    /// <summary>Current energy value. Range: 0..maxEnergy.</summary>
    public float Energy { get; private set; }

    /// <summary>Current energy as a 0..1 fraction. Safe for UI fill bars.</summary>
    public float EnergyNormalized
    {
        get
        {
            if (profile == null) return 0f;
            float max = E.maxEnergy;
            return max > 0f ? Energy / max : 0f;
        }
    }

    /// <summary>True while the pool is EMP-frozen. Abilities should treat this as "cannot spend."</summary>
    public bool IsEmpFrozen => empFreezeRemaining > 0f;

    /// <summary>True while energy is actively regenerating this frame.</summary>
    public bool IsRegenerating { get; private set; }

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------

    /// <summary>
    /// Time remaining on the EMP freeze. Additive on repeat hits.
    /// While > 0, both consumption and regen are suspended; the pool is frozen.
    /// Pure runtime state, not tuning.
    /// </summary>
    private float empFreezeRemaining;

    /// <summary>
    /// Unscaled time of the last TryConsume call (success or failure).
    /// Used to compute timeSinceLastConsume in TickRegen.
    /// A timestamp is immune to Update / FixedUpdate ordering: a per-frame bool
    /// reset at the start of Update would be cleared before TickRegen reads it if
    /// FixedUpdate runs in between. The timestamp holds its value until the next
    /// consume call, so the lockout is reliable regardless of who calls when.
    /// </summary>
    private float lastConsumeTime = float.NegativeInfinity;

    /// <summary>True after pool hits zero. Cleared when pool rises above zero.</summary>
    private bool isDepleted;

    /// <summary>True while the EMP freeze timer was > 0 last frame. Used to detect expiry.</summary>
    private bool wasEmpFrozen;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------

    private void Awake()
    {
        if (profile == null)
        {
            Debug.LogError(
                $"[Energy] '{name}': VehicleTuningProfile is not assigned. " +
                $"Assign one in the inspector. Energy disabled.",
                this
            );
            enabled = false;
            return;
        }

        Energy = Mathf.Clamp(E.startingEnergy, 0f, E.maxEnergy);
    }

    private void Update()
    {
        TickEmpFreeze();
        TickRegen();
        TickDepletionState();
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /// <summary>
    /// Request to consume <paramref name="amount"/> energy this frame.
    ///
    /// Returns true and debits the pool when:
    ///   The pool has sufficient reserves (Energy >= amount).
    ///   The system is not EMP-frozen.
    ///
    /// Returns false and debits nothing otherwise.
    ///
    /// Continuous abilities (boost) call this every frame.
    /// Instantaneous abilities (jump, dodge) call this once on activation.
    ///
    /// Any call this frame (success OR failure) suppresses regen for regenDelay
    /// seconds. See class summary for design rationale.
    /// </summary>
    public bool TryConsume(float amount)
    {
        // Record the time of this consume attempt regardless of outcome.
        // Resets the regen lockout. See class summary for rationale.
        lastConsumeTime = Time.unscaledTime;

        if (IsEmpFrozen)
            return false;

        if (Energy < amount)
            return false;

        Energy = Mathf.Max(0f, Energy - amount);
        return true;
    }

    /// <summary>
    /// Applies an EMP freeze for <paramref name="duration"/> seconds.
    /// Additive: multiple hits stack freeze time.
    /// During freeze, TryConsume always returns false and regen is suspended.
    /// </summary>
    public void ApplyEmpFreeze(float duration)
    {
        if (duration <= 0f)
            return;

        empFreezeRemaining += duration;
        OnEmpFreezeApplied?.Invoke(empFreezeRemaining);
    }

    // -------------------------------------------------------------------------
    // Private tick methods
    // -------------------------------------------------------------------------

    /// <summary>Counts down the EMP freeze timer and fires the lifted event on expiry.</summary>
    private void TickEmpFreeze()
    {
        if (empFreezeRemaining <= 0f)
        {
            if (wasEmpFrozen)
            {
                wasEmpFrozen = false;
                OnEmpFreezeLifted?.Invoke();
            }
            return;
        }

        empFreezeRemaining = Mathf.Max(0f, empFreezeRemaining - Time.deltaTime);
        wasEmpFrozen = true;
    }

    /// <summary>
    /// Handles regen lockout timing and energy restoration.
    ///
    /// Regen runs when:
    ///   1. The regen lockout has expired (timeSinceLastConsume >= regenDelay).
    ///   2. The system is not EMP-frozen.
    ///   3. The pool is not already full.
    ///
    /// OnRegenStarted fires once when regen transitions from inactive to active.
    /// </summary>
    private void TickRegen()
    {
        bool wasRegenerating = IsRegenerating;

        float timeSinceLastConsume = Time.unscaledTime - lastConsumeTime;
        bool  recentlyConsumed     = timeSinceLastConsume < E.regenDelay;
        bool  canRegen             = !recentlyConsumed && !IsEmpFrozen && Energy < E.maxEnergy;

        IsRegenerating = canRegen;

        if (IsRegenerating && !wasRegenerating)
            OnRegenStarted?.Invoke();

        if (IsRegenerating)
            Energy = Mathf.Min(E.maxEnergy, Energy + E.regenRate * Time.deltaTime);
    }

    /// <summary>Tracks depletion state and fires OnEnergyDepleted once per depletion event.</summary>
    private void TickDepletionState()
    {
        if (!isDepleted && Energy <= 0f)
        {
            isDepleted = true;
            OnEnergyDepleted?.Invoke();
        }
        else if (isDepleted && Energy > 0f)
        {
            isDepleted = false;
        }
    }

    // -------------------------------------------------------------------------
    // 🎨 Debug Gizmos
    // -------------------------------------------------------------------------
#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (!Application.isPlaying)
            return;

        if (debugSettings != null && !debugSettings.IsEnabled(HoverDebugCategory.Resources))
            return;

        if (profile == null)
            return;

        // Energy bar above the chassis. Green full, red empty, cyan while frozen.
        float normalized = EnergyNormalized;
        Color barColor   = IsEmpFrozen
            ? Color.cyan
            : Color.Lerp(Color.red, Color.green, normalized);

        Gizmos.color = barColor;
        Vector3 barStart = transform.position + Vector3.up * 2.5f;
        Vector3 barEnd   = barStart + transform.right * (normalized * 2f);
        Gizmos.DrawLine(barStart, barEnd);
        Gizmos.DrawWireSphere(barEnd, 0.07f);

        UnityEditor.Handles.color = barColor;
        UnityEditor.Handles.Label(
            transform.position + Vector3.up * 3f,
            IsEmpFrozen
                ? $"ENERGY [EMP FROZEN {empFreezeRemaining:F1}s] {Energy:F0}/{E.maxEnergy:F0}"
                : $"ENERGY {Energy:F0}/{E.maxEnergy:F0}{(IsRegenerating ? " ↑" : "")}"
        );
    }
#endif
}
