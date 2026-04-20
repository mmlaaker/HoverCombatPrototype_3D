using System;
using UnityEngine;

/// <summary>
/// VehicleHealth v1.0
/// -------------------
/// Responsibilities:
///   • Maintains the vehicle's HP pool. No regeneration — damage is permanent until death.
///   • Fires OnDamaged, OnDeath events for UI, audio, and game state hooks.
///   • On death: disables this GameObject and broadcasts OnDeath. Respawn is external.
///   • Implements IDamageable — place this on the vehicle root so
///     ParticleWeaponCollision.GetComponentInParent finds it.
///
/// Design contracts:
///   • This script owns no physics and no respawn logic.
///   • Invulnerability window (e.g. post-respawn) can be set via SetInvulnerable().
///   • EMP freeze does not block damage — that lives in HoverController_Energy.
///   • Death fires once per life. The GameObject is disabled immediately after.
/// </summary>
[DefaultExecutionOrder(-10)] // Initialize before weapons and energy
public class VehicleHealth : MonoBehaviour, IDamageable
{
    // -------------------------------------------------------------------------
    // ❤️ Pool
    // -------------------------------------------------------------------------

    [Header("❤️ Health")]
    [Tooltip("Maximum HP.")]
    [Min(1f)]
    [SerializeField] private float maxHealth = 100f;

    [Tooltip("Starting HP on spawn. Set to maxHealth to start full.")]
    [Min(0f)]
    [SerializeField] private float startingHealth = 100f;

    // -------------------------------------------------------------------------
    // 📢 Events
    // -------------------------------------------------------------------------

    /// <summary>
    /// Fired whenever damage is applied. Parameters: currentHealth, maxHealth.
    /// UI should subscribe here rather than polling every frame.
    /// </summary>
    public event Action<float, float> OnDamaged;

    /// <summary>
    /// Fired once when HP reaches zero. GameObject is disabled after this fires.
    /// Wire respawn logic here.
    /// </summary>
    public event Action OnDeath;

    // -------------------------------------------------------------------------
    // Public read-only state
    // -------------------------------------------------------------------------

    /// <summary>Current HP. Range: 0..maxHealth.</summary>
    public float Health { get; private set; }

    /// <summary>Current HP as a 0..1 fraction. Safe for UI fill bar.</summary>
    public float HealthNormalized => maxHealth > 0f ? Health / maxHealth : 0f;

    /// <summary>True if HP is above zero.</summary>
    public bool IsAlive { get; private set; }

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------

    [Header("🛠 Debug")]
    [Tooltip("Optional global debug toggle. When assigned, controls gizmo visibility.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    private bool  _isInvulnerable;
    private float _invulnerableTimer;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------

    private void Awake()
    {
        Health  = Mathf.Clamp(startingHealth, 0f, maxHealth);
        IsAlive = Health > 0f;
    }

    private void Update()
    {
        if (_invulnerableTimer > 0f)
        {
            _invulnerableTimer -= Time.deltaTime;
            if (_invulnerableTimer <= 0f)
                _isInvulnerable = false;
        }
    }

    // -------------------------------------------------------------------------
    // IDamageable
    // -------------------------------------------------------------------------

    /// <summary>
    /// Apply incoming damage. Called by ParticleWeaponCollision and projectile hit handlers.
    /// No-ops if dead or invulnerable.
    /// </summary>
    public void TakeDamage(float amount)
    {
        if (!IsAlive || _isInvulnerable || amount <= 0f)
            return;

        Health = Mathf.Max(0f, Health - amount);
        OnDamaged?.Invoke(Health, maxHealth);

        if (Health <= 0f)
            HandleDeath();
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /// <summary>
    /// Grants invulnerability for <paramref name="duration"/> seconds.
    /// Useful for post-respawn grace period. Replaces existing duration — not additive.
    /// </summary>
    public void SetInvulnerable(float duration)
    {
        if (duration <= 0f) return;
        _isInvulnerable    = true;
        _invulnerableTimer = duration;
    }

    /// <summary>
    /// Restores HP to max and re-enables the vehicle. Call from respawn logic.
    /// </summary>
    public void Respawn()
    {
        Health  = maxHealth;
        IsAlive = true;
        gameObject.SetActive(true);
    }

    // -------------------------------------------------------------------------
    // Private
    // -------------------------------------------------------------------------

    private void HandleDeath()
    {
        IsAlive = false;
        OnDeath?.Invoke();
        gameObject.SetActive(false);
    }

    // -------------------------------------------------------------------------
    // 🎨 Debug Gizmos
    // -------------------------------------------------------------------------
#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;

        if (debugSettings != null && !debugSettings.enableDebugGizmos) return;

        float normalized = HealthNormalized;
        Gizmos.color = Color.Lerp(Color.red, Color.green, normalized);

        Vector3 barStart = transform.position + Vector3.up * 2f;
        Vector3 barEnd   = barStart + transform.right * (normalized * 2f);
        Gizmos.DrawLine(barStart, barEnd);
        Gizmos.DrawWireSphere(barEnd, 0.07f);

        UnityEditor.Handles.color = Gizmos.color;
        UnityEditor.Handles.Label(
            transform.position + Vector3.up * 2.5f,
            $"HP {Health:F0}/{maxHealth:F0}{(_isInvulnerable ? " [INVULN]" : "")}"
        );
    }
#endif
}
