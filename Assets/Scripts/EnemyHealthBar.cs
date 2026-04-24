using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// EnemyHealthBar v1.0
/// --------------------
/// Responsibilities:
///   • Drives a world-space health bar for an enemy (or any) vehicle.
///   • Reads VehicleHealth from the parent hierarchy; subscribes to OnDamaged/OnDeath
///     and updates the fill Image's fillAmount. No per-frame health polling.
///   • Optional billboard: rotates to match the main camera each LateUpdate so the
///     bar stays readable from any angle.
///   • Optional auto-hide while at full HP, so undamaged enemies stay visually clean.
///
/// Setup:
///   • Create an empty child under the enemy vehicle root, positioned ~1.5-2m above it.
///   • Add a Canvas with Render Mode: World Space. Scale it down (e.g. 0.005 on all axes)
///     so it reads as a small bar, not a full screen.
///   • Inside the canvas add an Image. Set Image Type: Filled, Fill Method: Horizontal,
///     Fill Origin: Left. Optionally add a darker background Image behind it.
///   • Add this component to the canvas GameObject (or the child pivot that holds it).
///   • Assign the fill Image to 'fillImage'. Target auto-resolves from the parent vehicle.
///
/// Design contracts:
///   • This script owns no gameplay state. It only reads VehicleHealth and renders UI.
///   • Null-safe: missing target or fillImage logs a warning and skips.
///   • Respawn-safe: OnEnable re-subscribes and syncs from current HealthNormalized, so
///     a re-enabled vehicle shows a full bar again without special handling.
/// </summary>
public class EnemyHealthBar : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // 🎯 Target
    // -------------------------------------------------------------------------

    [Header("🎯 Target")]
    [Tooltip("Vehicle whose health drives this bar. If left empty, auto-resolves via " +
             "GetComponentInParent<VehicleHealth>() in Awake.")]
    [SerializeField] private VehicleHealth target;

    // -------------------------------------------------------------------------
    // 🖼 UI
    // -------------------------------------------------------------------------

    [Header("🖼 UI")]
    [Tooltip("Filled image representing remaining HP. Image Type: Filled, Fill Method: Horizontal.")]
    [SerializeField] private Image fillImage;

    [Tooltip("Optional root GameObject toggled by 'hideWhenFull'. " +
             "Defaults to fillImage's GameObject when empty — assign a parent if you also " +
             "want a background Image to hide with the fill.")]
    [SerializeField] private GameObject visibilityRoot;

    [Tooltip("Color at full HP.")]
    [SerializeField] private Color colorFull  = new Color(0.2f, 0.9f, 0.3f);

    [Tooltip("Color at zero HP.")]
    [SerializeField] private Color colorEmpty = new Color(0.9f, 0.15f, 0.1f);

    // -------------------------------------------------------------------------
    // 📷 Billboard
    // -------------------------------------------------------------------------

    [Header("📷 Billboard")]
    [Tooltip("When true, rotates this transform to match Camera.main each LateUpdate so " +
             "the bar always faces the player.")]
    [SerializeField] private bool billboardToCamera = true;

    [Tooltip("When true, the visibility root is hidden while HP is full. " +
             "The bar appears on first damage and stays visible until death or respawn.")]
    [SerializeField] private bool hideWhenFull = true;

    // -------------------------------------------------------------------------
    // Runtime
    // -------------------------------------------------------------------------

    private Transform _tf;
    private Camera    _cam;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------

    private void Awake()
    {
        _tf = transform;

        if (target == null)
            target = GetComponentInParent<VehicleHealth>();

        if (visibilityRoot == null && fillImage != null)
            visibilityRoot = fillImage.gameObject;

        if (target == null)
            Debug.LogWarning("[EnemyHealthBar] No VehicleHealth found on parent.", this);

        if (fillImage == null)
            Debug.LogWarning("[EnemyHealthBar] fillImage is not assigned.", this);
    }

    private void OnEnable()
    {
        if (target != null)
        {
            target.OnDamaged += HandleDamaged;
            target.OnDeath   += HandleDeath;
        }

        // Sync covers initial spawn and any re-enable after Respawn().
        SyncFromTarget();
    }

    private void OnDisable()
    {
        if (target != null)
        {
            target.OnDamaged -= HandleDamaged;
            target.OnDeath   -= HandleDeath;
        }
    }

    private void LateUpdate()
    {
        if (!billboardToCamera) return;

        if (_cam == null) _cam = Camera.main;
        if (_cam == null) return;

        // Match camera rotation — the canonical Unity billboard for world-space UI.
        _tf.rotation = _cam.transform.rotation;
    }

    // -------------------------------------------------------------------------
    // Event handlers
    // -------------------------------------------------------------------------

    private void HandleDamaged(float current, float max)
    {
        Apply(max > 0f ? current / max : 0f);
    }

    private void HandleDeath()
    {
        Apply(0f);
    }

    // -------------------------------------------------------------------------
    // Internal
    // -------------------------------------------------------------------------

    private void SyncFromTarget()
    {
        if (target == null) return;
        Apply(target.HealthNormalized);
    }

    private void Apply(float normalized)
    {
        normalized = Mathf.Clamp01(normalized);

        if (fillImage != null)
        {
            fillImage.fillAmount = normalized;
            fillImage.color      = Color.Lerp(colorEmpty, colorFull, normalized);
        }

        if (hideWhenFull && visibilityRoot != null)
        {
            bool shouldShow = normalized < 0.999f;
            if (visibilityRoot.activeSelf != shouldShow)
                visibilityRoot.SetActive(shouldShow);
        }
    }
}
