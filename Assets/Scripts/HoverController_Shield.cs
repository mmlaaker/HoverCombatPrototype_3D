using System;
using UnityEngine;

/// <summary>
/// HoverController_Shield v1.0
///
/// Fixed-duration invulnerability burst. Pays a flat energy cost on activation,
/// runs for shieldDuration seconds, and blocks all damage during that window.
/// Cannot be cancelled by the player. An incoming EMP impact is absorbed by an
/// active shield: the shield deactivates and the freeze is not applied.
/// Activation is still blocked while the vehicle is already EMP-frozen.
///
/// Tuning lives on a VehicleTuningProfile asset (profile.shield). Runtime state
/// (IsActive, TimeRemaining) stays on the component.
///
/// Contracts:
///   Activation costs are paid once via energy.TryConsume(shieldEnergyCost).
///   While IsActive, VehicleHealth.TakeDamage early-outs to zero damage applied.
///   While IsActive, repeated presses are no-ops (no second debit, no extension).
///   EMP absorption is driven by EmpProjectile.Consume calling TryAbsorbEmp on hit.
///   The OnEmpFreezeApplied subscription remains as a defensive safety net for
///   any future freeze source that bypasses the projectile.
///
/// Owns no physics. Pure ability state machine.
/// </summary>
[RequireComponent(typeof(HoverController_Energy))]
public class HoverController_Shield : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // 📦 Tuning Profile
    // -------------------------------------------------------------------------
    [Header("📦 Tuning")]
    [Tooltip("Vehicle tuning profile (shared SO). All numeric tuning lives here. Required.")]
    [SerializeField] private VehicleTuningProfile profile;

    /// <summary>Shorthand for profile.shield. Used at every read site below.</summary>
    private ShieldTuning S => profile.shield;

    // -------------------------------------------------------------------------
    // 📢 Events
    // -------------------------------------------------------------------------

    /// <summary>Fired when the shield successfully activates.</summary>
    public event Action OnShieldActivated;

    /// <summary>Fired when the shield ends, whether by timer expiry or EMP cancel.</summary>
    public event Action OnShieldDeactivated;

    // -------------------------------------------------------------------------
    // ✨ VFX
    // -------------------------------------------------------------------------
    // Drives a shield-mesh renderer's material color via MaterialPropertyBlock.
    // On activation the color lerps from inactiveColor to activeColor over
    // fadeInDuration; on deactivation it lerps back over fadeOutDuration. Both
    // RGB and alpha are interpolated, so a designer can shift hue/saturation
    // between states or just drive alpha by keeping RGB the same in both colors.
    // MaterialPropertyBlock keeps the material shared so vehicles that use this
    // profile do not get individual material instances.
    //
    // Setup expectations:
    //   The shield mesh is a child GameObject with its own Renderer. The shader
    //   on its material exposes a Color property whose name matches colorProperty
    //   below (URP default: "_BaseColor"). The shader should treat alpha 0 as
    //   fully invisible.
    //
    //   The Renderer field is optional. Leave it null on vehicles that have no
    //   shield mesh; the gameplay layer is unaffected.

    [Header("✨ VFX")]
    [Tooltip("Renderer that displays the shield mesh. Optional. " +
             "Null = gameplay only, no visual.")]
    [SerializeField] private Renderer shieldRenderer;

    [Tooltip("Material color property driven by the fade. Must be a Color on the renderer's shader. " +
             "Legacy Particles/Additive: '_TintColor'. URP Lit: '_BaseColor'. Built-in: '_Color'.")]
    [SerializeField] private string colorProperty = "_TintColor";

    [Tooltip("Color (RGB + alpha) when the shield is fully inactive. " +
             "Typical: alpha = 0 so the mesh is invisible at rest.")]
    [SerializeField] private Color inactiveColor = new Color(0f, 0f, 0f, 0f);

    [Tooltip("Color (RGB + alpha) when the shield is fully active. " +
             "Typical: alpha = 1 for full visibility.")]
    [SerializeField] private Color activeColor = new Color(0.4f, 0.7f, 1f, 1f);

    [Tooltip("Seconds to lerp from inactiveColor to activeColor when the shield activates.")]
    [Min(0f)]
    [SerializeField] private float fadeInDuration = 0.15f;

    [Tooltip("Seconds to lerp from activeColor to inactiveColor when the shield ends " +
             "(timer expiry or EMP cancel).")]
    [Min(0f)]
    [SerializeField] private float fadeOutDuration = 0.25f;

    // -------------------------------------------------------------------------
    // 🛠 Debug
    // -------------------------------------------------------------------------
    [Header("🛠 Debug")]
    [Tooltip("Optional global debug toggle. When assigned, controls gizmo visibility.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    // -------------------------------------------------------------------------
    // Public read-only state
    // -------------------------------------------------------------------------

    /// <summary>True while the shield is active and blocking damage.</summary>
    public bool IsActive { get; private set; }

    /// <summary>Seconds remaining on the active shield. Zero when inactive.</summary>
    public float TimeRemaining { get; private set; }

    // -------------------------------------------------------------------------
    // Runtime references
    // -------------------------------------------------------------------------

    private IHoverInputProvider input;
    private HoverController_Energy energy;

    // Fade runtime. _currentT and _targetT are 0..1 lerp parameters between
    // inactiveColor and activeColor. 0 = inactive, 1 = active.
    private MaterialPropertyBlock _propertyBlock;
    private int _colorPropertyId;
    private float _currentT;
    private float _targetT;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------

    private void Awake()
    {
        if (profile == null)
        {
            Debug.LogError(
                $"[Shield] '{name}': VehicleTuningProfile is not assigned. " +
                $"Assign one in the inspector. Shield disabled.",
                this
            );
            enabled = false;
            return;
        }

        input  = GetComponent<IHoverInputProvider>();
        energy = GetComponent<HoverController_Energy>();

        if (input == null)
        {
            Debug.LogError(
                $"[Shield] '{name}': No IHoverInputProvider found. " +
                $"Attach PlayerHoverInput or AIHoverInput. Shield disabled.",
                this
            );
            enabled = false;
        }

        if (shieldRenderer != null)
        {
            _propertyBlock   = new MaterialPropertyBlock();
            _colorPropertyId = Shader.PropertyToID(colorProperty);
            _currentT        = 0f;
            _targetT         = 0f;
            ApplyColor();
            shieldRenderer.gameObject.SetActive(false);
        }
    }

    private void OnEnable()
    {
        if (energy != null)
            energy.OnEmpFreezeApplied += HandleEmpFreeze;
    }

    private void OnDisable()
    {
        if (energy != null)
            energy.OnEmpFreezeApplied -= HandleEmpFreeze;

        if (IsActive)
            Deactivate();
    }

    private void Update()
    {
        if (IsActive)
        {
            TimeRemaining -= Time.deltaTime;
            if (TimeRemaining <= 0f)
                Deactivate();
        }
        else if (input.ShieldPressed)
        {
            TryActivate();
        }

        // Fade ticks every frame so the fade-out continues running after IsActive
        // flips false. No-op when no renderer is assigned or alpha already matches target.
        TickFade();
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /// <summary>
    /// Attempts to activate the shield. Returns true on success.
    ///
    /// Returns false (no state change, no debit) when:
    ///   Shield is disabled in tuning.
    ///   Shield is already active.
    ///   Energy is EMP-frozen.
    ///   Energy reserves are below shieldEnergyCost.
    ///
    /// Cost is paid via energy.TryConsume on success only. Note that energy.TryConsume
    /// resets the regen lockout on any call; that anti-spam contract is intentional.
    /// </summary>
    public bool TryActivate()
    {
        if (!S.enableShield) return false;
        if (IsActive) return false;
        if (energy.IsEmpFrozen) return false;

        if (!energy.TryConsume(S.shieldEnergyCost))
            return false;

        IsActive      = true;
        TimeRemaining = S.shieldDuration;
        OnShieldActivated?.Invoke();

        if (shieldRenderer != null)
        {
            shieldRenderer.gameObject.SetActive(true);
            _targetT = 1f;
        }

        return true;
    }

    /// <summary>
    /// Called by incoming EMP impacts to ask the shield to absorb the hit.
    /// Returns true if the shield was active and absorbed the EMP (and has
    /// now deactivated). Returns false if the shield was inactive -- caller
    /// should apply the normal EMP freeze in that case.
    /// </summary>
    public bool TryAbsorbEmp()
    {
        if (!IsActive) return false;
        Deactivate();
        return true;
    }

    // -------------------------------------------------------------------------
    // Private
    // -------------------------------------------------------------------------

    private void Deactivate()
    {
        if (!IsActive) return;

        IsActive      = false;
        TimeRemaining = 0f;
        OnShieldDeactivated?.Invoke();

        if (shieldRenderer != null)
            _targetT = 0f;
    }

    private void HandleEmpFreeze(float _)
    {
        if (IsActive)
            Deactivate();
    }

    private void TickFade()
    {
        if (shieldRenderer == null) return;

        if (Mathf.Approximately(_currentT, _targetT))
        {
            // Fully faded out: disable the VFX GameObject until the next activation.
            if (_targetT <= 0f && shieldRenderer.gameObject.activeSelf)
                shieldRenderer.gameObject.SetActive(false);
            return;
        }

        float duration = _targetT > _currentT ? fadeInDuration : fadeOutDuration;
        float rate     = duration > 0f ? 1f / duration : float.PositiveInfinity;
        _currentT      = Mathf.MoveTowards(_currentT, _targetT, rate * Time.deltaTime);
        ApplyColor();
    }

    private void ApplyColor()
    {
        if (shieldRenderer == null) return;
        Color c = Color.Lerp(inactiveColor, activeColor, _currentT);
        shieldRenderer.GetPropertyBlock(_propertyBlock);
        _propertyBlock.SetColor(_colorPropertyId, c);
        shieldRenderer.SetPropertyBlock(_propertyBlock);
    }

    // -------------------------------------------------------------------------
    // 🎨 Debug Gizmos
    // -------------------------------------------------------------------------
#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (!Application.isPlaying) return;
        if (debugSettings != null && !debugSettings.enableDebugGizmos) return;
        if (!IsActive) return;

        Gizmos.color = new Color(1f, 0.85f, 0.2f, 0.6f);
        Gizmos.DrawWireSphere(transform.position, 1.5f);

        UnityEditor.Handles.color = Gizmos.color;
        UnityEditor.Handles.Label(
            transform.position + Vector3.up * 3.5f,
            $"SHIELD {TimeRemaining:F1}s"
        );
    }
#endif
}
