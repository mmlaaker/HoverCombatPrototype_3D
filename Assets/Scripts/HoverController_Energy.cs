using System;
using UnityEngine;

/// <summary>
/// HoverController_Energy v1.0
/// ----------------------------
/// Responsibilities:
///   • Maintains the shared energy pool for all hover abilities (boost, jump, shield, EMP).
///   • Regenerates energy when no ability is actively consuming.
///   • Enforces a configurable regen lockout delay after the last spend.
///   • Provides TryConsume(float) as the single gate all ability systems call.
///   • Supports external EMP freeze: halts both consumption and regen for a set duration.
///   • Broadcasts events for UI and audio hooks.
///
/// Design contracts:
///   • Abilities call TryConsume every frame they are active (continuous) or once on
///     activation (instantaneous). Energy does not care which — it just debits and gates.
///   • Regen suppression is demand-driven: any TryConsume call this frame resets the
///     regen lockout timer. No ability needs to explicitly signal "I stopped."
///   • EMP freeze is additive: multiple hits extend the freeze duration.
///   • This script owns no physics. It is a pure resource manager.
///
/// Defaults are tuned so a full tank provides ~5 seconds of continuous boost.
/// All values are Inspector-tunable.
/// </summary>
public class HoverController_Energy : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // ⚡ Pool
    // -------------------------------------------------------------------------
    [Header("⚡ Pool")]
    [Tooltip("Maximum energy. Treat as an abstract unit — tune relative to ability costs.")]
    [Min(1f)]
    [SerializeField] private float maxEnergy = 100f;

    [Tooltip("Starting energy on spawn. Set to maxEnergy to start full.")]
    [Min(0f)]
    [SerializeField] private float startingEnergy = 100f;

    // -------------------------------------------------------------------------
    // 🔋 Regeneration
    // -------------------------------------------------------------------------
    [Header("🔋 Regeneration")]
    [Tooltip("Energy restored per second when no ability is consuming and the lockout has expired.")]
    [Min(0f)]
    [SerializeField] private float regenRate = 20f;

    [Tooltip("Seconds after the last consume call before regen resumes. Set to 0 to disable.")]
    [Min(0f)]
    [SerializeField] private float regenDelay = 1f;

    // -------------------------------------------------------------------------
    // 🧊 EMP Freeze
    // -------------------------------------------------------------------------
    [Header("🧊 EMP Freeze")]
    [Tooltip("Remaining EMP freeze duration (seconds). Additive on repeat hits. " +
             "While > 0, both consumption and regen are suspended — pool is frozen.")]
    [SerializeField, Min(0f)] private float empFreezeRemaining;

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
    // Public read-only state
    // -------------------------------------------------------------------------

    /// <summary>Current energy value. Range: 0..maxEnergy.</summary>
    public float Energy { get; private set; }

    /// <summary>Current energy as a 0..1 fraction. Safe for UI fill bars.</summary>
    public float EnergyNormalized => maxEnergy > 0f ? Energy / maxEnergy : 0f;

    /// <summary>True while the pool is EMP-frozen. Abilities should treat this as "cannot spend."</summary>
    public bool IsEmpFrozen => empFreezeRemaining > 0f;

    /// <summary>True while energy is actively regenerating this frame.</summary>
    public bool IsRegenerating { get; private set; }

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------

    /// <summary>
    /// Seconds since last TryConsume call. When this exceeds regenDelay, regen resumes.
    /// Reset to 0 every frame that TryConsume is called. Tracked in Update so it
    /// accumulates at wall-clock speed regardless of fixed timestep.
    /// </summary>
    private float timeSinceLastConsume;

    /// <summary>
    /// Whether a TryConsume call was made this Update frame.
    /// Set at the start of each Update, written by TryConsume, read by regen logic.
    /// </summary>
    private bool consumedThisFrame;

    /// <summary>True after pool hits zero; cleared when pool rises above zero.</summary>
    private bool isDepleted;

    /// <summary>True while EMP freeze timer was > 0 last frame — used to detect expiry.</summary>
    private bool wasEmpFrozen;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------

    private void Awake()
    {
        Energy = Mathf.Clamp(startingEnergy, 0f, maxEnergy);
    }

    private void Update()
    {
        // Reset per-frame consume flag. TryConsume sets this if called this frame.
        consumedThisFrame = false;

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
    /// Returns true and debits the pool if:
    ///   • The pool has sufficient reserves (Energy >= amount).
    ///   • The system is not EMP-frozen.
    ///
    /// Returns false and debits nothing otherwise.
    ///
    /// Continuous abilities (boost) call this every frame.
    /// Instantaneous abilities (jump) call this once on activation.
    ///
    /// Any successful or failed call this frame suppresses regen for regenDelay seconds.
    /// This prevents regen ticking during rapid failed attempts at the floor.
    /// </summary>
    public bool TryConsume(float amount)
    {
        // Mark that consumption was attempted this frame regardless of outcome.
        // This resets the regen lockout timer.
        consumedThisFrame = true;

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

    /// <summary>
    /// Counts down the EMP freeze timer and fires the lifted event on expiry.
    /// </summary>
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
    ///   1. No consume call was made this frame (consumedThisFrame == false).
    ///   2. The regen lockout has expired (timeSinceLastConsume >= regenDelay).
    ///   3. The system is not EMP-frozen.
    ///   4. The pool is not already full.
    ///
    /// OnRegenStarted fires once when regen transitions from inactive to active.
    /// </summary>
    private void TickRegen()
    {
        bool wasRegenerating = IsRegenerating;

        if (consumedThisFrame)
        {
            timeSinceLastConsume = 0f;
            IsRegenerating = false;
        }
        else
        {
            timeSinceLastConsume += Time.deltaTime;
        }

        bool lockoutExpired  = timeSinceLastConsume >= regenDelay;
        bool canRegen        = !consumedThisFrame && lockoutExpired && !IsEmpFrozen && Energy < maxEnergy;

        IsRegenerating = canRegen;

        if (IsRegenerating && !wasRegenerating)
            OnRegenStarted?.Invoke();

        if (IsRegenerating)
            Energy = Mathf.Min(maxEnergy, Energy + regenRate * Time.deltaTime);
    }

    /// <summary>
    /// Tracks depletion state and fires OnEnergyDepleted once per depletion event.
    /// </summary>
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

        // Energy bar above the craft — green full, red empty, blue while frozen.
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
                ? $"ENERGY [EMP FROZEN {empFreezeRemaining:F1}s] {Energy:F0}/{maxEnergy:F0}"
                : $"ENERGY {Energy:F0}/{maxEnergy:F0}{(IsRegenerating ? " ↑" : "")}"
        );
    }
#endif
}
