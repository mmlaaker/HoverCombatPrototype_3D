using UnityEngine;
using UnityEngine.UI;
using TMPro;

/// <summary>
/// VehicleHUD v1.3
/// ----------------
/// v1.3 changes:
///   • OnRegenStarted subscription, unsubscription and its empty HandleRegenStarted
///     handler all deleted, along with the event itself on HoverController_Energy.
///     v1.2 below removed the local flag this handler existed to set and left the
///     wiring in place, so the HUD had been paying for an event that did nothing for
///     a version. Regen colour still reads _energy.IsRegenerating directly; behaviour
///     is unchanged. If a regen cue is ever wanted, add it deliberately.
///
/// v1.2 changes:
///   • _isRegenerating local flag removed entirely. All regen color and Update gate
///     decisions now read _energy.IsRegenerating directly — the authoritative source.
///     Previously the local flag could get stuck true while energy was actively draining
///     (e.g. during boost), causing the bar to show regen color while depleting.
///   • HandleRegenStarted and HandleEnergyDepleted simplified accordingly.
///     (HandleRegenStarted has since been deleted outright; see v1.3.)
///   • Update gate expanded: SyncEnergy runs whenever pool is not full, regenerating,
///     or EMP-frozen — covers active drain (boost) without any local tracking flags.
///
/// v1.1 changes:
///   • SyncAll moved from OnEnable to Start. OnEnable now only subscribes to events.
///     This follows the Unity Awake/Start contract: Awake = initialize self,
///     Start = interact with others. Eliminates the need for DefaultExecutionOrder
///     attributes to guarantee energy/health values are initialized before the HUD reads them.
///
/// Responsibilities:
///   • Displays health bar, energy bar, active weapon name, ammo count,
///     and missile lock progress.
///   • Event-driven: subscribes to VehicleHealth, HoverController_Energy,
///     and HoverController_Weapons. Does not poll per frame except when the
///     energy pool is not at full (covers drain, regen, and EMP freeze).
///   • EMP freeze state is reflected on the energy bar (color shift to cyan).
///   • Missile lock progress bar is shown/hidden based on active weapon type.
///
/// Setup:
///   • Assign the vehicle root GameObject to 'vehicleRoot' in the Inspector.
///   • All three controller components must be on that root or its children.
///   • Wire UI references in the Inspector — see region headers below.
///   • This component can live on any GameObject (e.g. a HUD canvas child).
///
/// Design contracts:
///   • This script owns no game state. It only reads and displays.
///   • Null-safe on all UI references — missing assignments log a warning and skip.
///   • Unsubscribes from all events OnDisable to prevent stale delegate leaks.
/// </summary>
public class VehicleHUD : MonoBehaviour
{
    // =========================================================================
    // 🔗 Vehicle Reference
    // =========================================================================

    [Header("🔗 Vehicle")]
    [Tooltip("Root GameObject of the player vehicle. Must have VehicleHealth, " +
             "HoverController_Energy, and HoverController_Weapons.")]
    [SerializeField] private GameObject vehicleRoot;

    // =========================================================================
    // ❤️ Health
    // =========================================================================

    [Header("❤️ Health")]
    [Tooltip("Fill image for the health bar. Set Image Type to Filled, Fill Method to Horizontal.")]
    [SerializeField] private Image healthFill;

    [Tooltip("Optional: text showing current HP (e.g. '82').")]
    [SerializeField] private TextMeshProUGUI healthText;

    [Tooltip("Color of the health bar at full HP.")]
    [SerializeField] private Color healthColorFull  = new Color(0.2f, 0.9f, 0.3f);

    [Tooltip("Color of the health bar at zero HP.")]
    [SerializeField] private Color healthColorEmpty = new Color(0.9f, 0.15f, 0.1f);

    // =========================================================================
    // ⚡ Energy
    // =========================================================================

    [Header("⚡ Energy")]
    [Tooltip("Fill image for the energy bar. Set Image Type to Filled, Fill Method to Horizontal.")]
    [SerializeField] private Image energyFill;

    [Tooltip("Optional: text showing current energy (e.g. '74').")]
    [SerializeField] private TextMeshProUGUI energyText;

    [Tooltip("Color of the energy bar during normal operation.")]
    [SerializeField] private Color energyColorNormal = new Color(0.2f, 0.7f, 1f);

    [Tooltip("Color of the energy bar while EMP-frozen.")]
    [SerializeField] private Color energyColorEmpFrozen = Color.cyan;

    [Tooltip("Color of the energy bar while regenerating.")]
    [SerializeField] private Color energyColorRegen = new Color(0.4f, 1f, 0.6f);

    [Tooltip("Color of the energy bar while shield is active. Top priority over EMP and regen.")]
    [SerializeField] private Color energyColorShieldActive = new Color(1f, 0.85f, 0.2f);

    // =========================================================================
    // 🔫 Weapon
    // =========================================================================

    [Header("🔫 Weapon")]
    [Tooltip("Text label showing the active weapon's display name.")]
    [SerializeField] private TextMeshProUGUI weaponNameText;

    [Tooltip("Text label showing ammo count. Shows '∞' for unlimited weapons.")]
    [SerializeField] private TextMeshProUGUI ammoText;

    // =========================================================================
    // 🎯 Reticle
    // =========================================================================

    [Header("🎯 Reticle")]
    [Tooltip("UI Image used as a crosshair/reticle. Fades in during strafe mode, hidden in drive mode. " +
             "Place this at the center of the screen on the HUD canvas.")]
    [SerializeField] private Image reticleImage;

    [Tooltip("Reticle color at full strafe. Alpha is driven by StrafeModeBlend — " +
             "set the desired visible alpha here (e.g. 0.9).")]
    [SerializeField] private Color reticleColor = new Color(1f, 1f, 1f, 0.9f);

    [Tooltip("Fallback projection distance (meters) when the aim raycast hits nothing — " +
             "e.g. aiming at open sky. Also caps the raycast's search distance.")]
    [SerializeField] private float reticleProjectionDistance = 200f;

    [Tooltip("Layers the aim raycast will consider when picking reticle distance. " +
             "The owning vehicle's layer is stripped automatically at runtime. " +
             "Exclude VFX, water, or other non-solid layers you don't want to lock onto.")]
    [SerializeField] private LayerMask reticleRaycastMask = ~0;

    [Tooltip("How quickly the reticle's projection distance tracks changes. Higher values " +
             "snap onto new targets faster; lower values smooth out flicker when the ray " +
             "transitions between near and far surfaces. 15 is a good starting point.")]
    [SerializeField] private float reticleFollowSpeed = 15f;

    [Tooltip("Canvas-space nudge applied to the reticle after projection. Use this to " +
             "compensate for a consistent mis-aim — e.g. bullets land slightly low-right " +
             "of the reticle. Units are canvas pixels: X+ = right, Y+ = up.")]
    [SerializeField] private Vector2 reticleScreenOffset = Vector2.zero;

    // =========================================================================
    // 🎯 Missile Lock
    // =========================================================================

    [Header("🎯 Missile Lock")]
    [Tooltip("Root object for the missile lock UI group. Shown only when a missile weapon is active.")]
    [SerializeField] private GameObject missileLockGroup;

    [Tooltip("Fill image for missile lock progress. Set Image Type to Filled.")]
    [SerializeField] private Image missileLockFill;

    [Tooltip("Text label showing lock state (e.g. 'SCANNING', 'LOCKED').")]
    [SerializeField] private TextMeshProUGUI missileLockStateText;

    [Tooltip("Color while scanning for a lock.")]
    [SerializeField] private Color lockColorScanning = Color.yellow;

    [Tooltip("Color when lock is confirmed.")]
    [SerializeField] private Color lockColorLocked = Color.green;

    // =========================================================================
    // Runtime references
    // =========================================================================

    private VehicleHealth              _health;
    private HoverController_Energy     _energy;
    private HoverController_Weapons    _weapons;
    private HoverController_Propulsion _propulsion;
    private HoverController_Shield     _shield;

    // Only EMP freeze is tracked locally — it has no per-frame readable equivalent
    // on the energy component that covers the full freeze window reliably.
    private bool _isEmpFrozen;
    private bool _isShieldActive;
    private bool _wasRegenerating;

    // Smoothed reticle projection distance. Tracks the aim raycast hit distance,
    // lerped by reticleFollowSpeed so the reticle doesn't snap when the ray transitions
    // between near and far surfaces.
    private float _reticleDistance;

    // =========================================================================
    // Unity lifecycle
    // =========================================================================

    private void Awake()
    {
        if (vehicleRoot == null)
        {
            Debug.LogError("[VehicleHUD] vehicleRoot is not assigned.", this);
            enabled = false;
            return;
        }

        _health     = vehicleRoot.GetComponentInChildren<VehicleHealth>();
        _energy     = vehicleRoot.GetComponentInChildren<HoverController_Energy>();
        _weapons    = vehicleRoot.GetComponentInChildren<HoverController_Weapons>();
        _propulsion = vehicleRoot.GetComponentInChildren<HoverController_Propulsion>();
        _shield     = vehicleRoot.GetComponentInChildren<HoverController_Shield>();

        if (_health     == null) Debug.LogWarning("[VehicleHUD] VehicleHealth not found on vehicleRoot.", this);
        if (_energy     == null) Debug.LogWarning("[VehicleHUD] HoverController_Energy not found on vehicleRoot.", this);
        if (_weapons    == null) Debug.LogWarning("[VehicleHUD] HoverController_Weapons not found on vehicleRoot.", this);
        if (_propulsion == null) Debug.LogWarning("[VehicleHUD] HoverController_Propulsion not found on vehicleRoot.", this);
        if (_shield     == null) Debug.LogWarning("[VehicleHUD] HoverController_Shield not found on vehicleRoot.", this);

        // Strip the owning vehicle's layer so the aim raycast can't self-hit.
        // VehicleLayerAssigner runs at execution order -20, so the layer is settled by now.
        reticleRaycastMask &= ~(1 << vehicleRoot.layer);

        // Initialize smoothed distance to the fallback so the first frame doesn't lerp from 0.
        _reticleDistance = reticleProjectionDistance;
    }

    private void OnEnable()
    {
        if (_health != null)
        {
            _health.OnDamaged += HandleHealthChanged;
            _health.OnDeath   += HandleDeath;
        }

        if (_energy != null)
        {
            _energy.OnEmpFreezeApplied += HandleEmpFreezeApplied;
            _energy.OnEmpFreezeLifted  += HandleEmpFreezeLifted;
            _energy.OnEnergyDepleted   += HandleEnergyDepleted;
        }

        if (_weapons != null)
        {
            _weapons.OnWeaponSwitched          += HandleWeaponSwitched;
            _weapons.OnWeaponFired             += HandleWeaponFired;
            _weapons.OnMissileLockStateChanged += HandleLockStateChanged;
        }

        if (_shield != null)
        {
            _shield.OnShieldActivated   += HandleShieldActivated;
            _shield.OnShieldDeactivated += HandleShieldDeactivated;
        }
    }

    private void Start()
    {
        // All Awakes have run by this point — safe to read initial values.
        SyncAll();
    }

    private void OnDisable()
    {
        if (_health != null)
        {
            _health.OnDamaged -= HandleHealthChanged;
            _health.OnDeath   -= HandleDeath;
        }

        if (_energy != null)
        {
            _energy.OnEmpFreezeApplied -= HandleEmpFreezeApplied;
            _energy.OnEmpFreezeLifted  -= HandleEmpFreezeLifted;
            _energy.OnEnergyDepleted   -= HandleEnergyDepleted;
        }

        if (_weapons != null)
        {
            _weapons.OnWeaponSwitched          -= HandleWeaponSwitched;
            _weapons.OnWeaponFired             -= HandleWeaponFired;
            _weapons.OnMissileLockStateChanged -= HandleLockStateChanged;
        }

        if (_shield != null)
        {
            _shield.OnShieldActivated   -= HandleShieldActivated;
            _shield.OnShieldDeactivated -= HandleShieldDeactivated;
        }
    }

    /// <summary>
    /// Runs SyncEnergy whenever the pool is not full, regenerating, or EMP-frozen.
    /// This covers all active energy states (drain, regen, freeze) without any
    /// local tracking flags that can go stale.
    /// When the pool is full and idle, this becomes a no-op at minimal cost.
    /// </summary>
    private void Update()
    {
        if (_energy == null) return;

        bool isRegen = _energy.IsRegenerating;

        // Run SyncEnergy while the pool is changing or on the frame regen stops
        // (so the color snaps back to normal on the same frame regen completes).
        if (_energy.EnergyNormalized < 1f || isRegen || _wasRegenerating || _isEmpFrozen || _isShieldActive)
            SyncEnergy();

        _wasRegenerating = isRegen;

        SyncReticle();

        // HandleLockStateChanged only fires on transitions, so the fill bar
        // would snap 0 → 1 instead of animating. Poll while Scanning to tween it.
        if (_weapons != null
            && missileLockFill != null
            && _weapons.CurrentLockState == HoverController_Weapons.MissileLockState.Scanning)
        {
            missileLockFill.fillAmount = _weapons.LockProgress;
        }
    }

    // =========================================================================
    // Event handlers — Health
    // =========================================================================

    private void HandleHealthChanged(float current, float max)
    {
        SetFill(healthFill, current / max);
        SetText(healthText, Mathf.CeilToInt(current).ToString());
        if (healthFill != null)
            healthFill.color = Color.Lerp(healthColorEmpty, healthColorFull, current / max);
    }

    private void HandleDeath()
    {
        SetFill(healthFill, 0f);
        SetText(healthText, "0");
        if (healthFill != null)
            healthFill.color = healthColorEmpty;
    }

    // =========================================================================
    // Event handlers — Energy
    // =========================================================================

    private void HandleEmpFreezeApplied(float duration)
    {
        _isEmpFrozen = true;
    }

    private void HandleEmpFreezeLifted()
    {
        _isEmpFrozen = false;
        // Color corrects on next SyncEnergy call in Update.
    }

    private void HandleEnergyDepleted()
    {
        // Force an immediate sync so the bar snaps to empty without waiting for Update.
        SyncEnergy();
    }

    // =========================================================================
    // Event handlers — Shield
    // =========================================================================

    private void HandleShieldActivated()
    {
        _isShieldActive = true;
        SyncEnergy();
    }

    private void HandleShieldDeactivated()
    {
        _isShieldActive = false;
        SyncEnergy();
    }

    // =========================================================================
    // Event handlers — Weapons
    // =========================================================================

    private void HandleWeaponSwitched(int slotIndex)
    {
        SyncWeapon();
        bool isMissile = _weapons.ActiveSlot?.definition?.type == WeaponType.Missile;
        SetMissileLockGroupVisible(isMissile);
    }

    private void HandleWeaponFired(int slotIndex, int ammoRemaining)
    {
        SetText(ammoText, ammoRemaining == -1 ? "∞" : ammoRemaining.ToString());
    }

    private void HandleLockStateChanged(HoverController_Weapons.MissileLockState state)
    {
        if (missileLockFill != null)
            missileLockFill.fillAmount = _weapons.LockProgress;

        if (missileLockStateText != null)
        {
            missileLockStateText.text = state switch
            {
                HoverController_Weapons.MissileLockState.Scanning => "SCANNING",
                HoverController_Weapons.MissileLockState.Locked    => "LOCKED",
                HoverController_Weapons.MissileLockState.Committed => "FIRED",
                _                                                  => ""
            };
        }

        if (missileLockFill != null)
        {
            missileLockFill.color = state == HoverController_Weapons.MissileLockState.Locked
                ? lockColorLocked
                : lockColorScanning;
        }
    }

    // =========================================================================
    // Sync helpers
    // =========================================================================

    private void SyncAll()
    {
        SyncHealth();
        SyncEnergy();
        SyncWeapon();
        SyncReticle();
    }

    private void SyncHealth()
    {
        if (_health == null) return;
        float normalized = _health.HealthNormalized;
        SetFill(healthFill, normalized);
        SetText(healthText, Mathf.CeilToInt(_health.Health).ToString());
        if (healthFill != null)
            healthFill.color = Color.Lerp(healthColorEmpty, healthColorFull, normalized);
    }

    private void SyncEnergy()
    {
        if (_energy == null) return;

        float normalized = _energy.EnergyNormalized;
        SetFill(energyFill, normalized);
        SetText(energyText, Mathf.CeilToInt(_energy.Energy).ToString());

        // Color priority: shield active > EMP frozen > regenerating > normal.
        if (energyFill != null)
        {
            energyFill.color = _isShieldActive
                ? energyColorShieldActive
                : _isEmpFrozen
                    ? energyColorEmpFrozen
                    : _energy.IsRegenerating
                        ? energyColorRegen
                        : energyColorNormal;
        }
    }

    private void SyncWeapon()
    {
        if (_weapons == null) return;

        var slot = _weapons.ActiveSlot;
        if (slot?.definition == null)
        {
            SetText(weaponNameText, "—");
            SetText(ammoText, "—");
            return;
        }

        SetText(weaponNameText, slot.definition.displayName);
        SetText(ammoText, slot.currentAmmo == -1 ? "∞" : slot.currentAmmo.ToString());
    }

    private void SyncReticle()
    {
        if (reticleImage == null || _propulsion == null) return;

        float blend = _propulsion.StrafeModeBlend;
        reticleImage.enabled = blend > 0f;
        if (blend <= 0f) return;

        // Alpha fade with strafe blend
        Color c = reticleColor;
        c.a = reticleColor.a * blend;
        reticleImage.color = c;

        // Project the aim direction onto screen space.
        // Uses the same yaw + pitch (no roll) as HoverController_Aim.
        Camera cam = Camera.main;
        if (cam == null) return;

        Transform vt = vehicleRoot.transform;
        float pitch = vt.eulerAngles.x;
        if (pitch > 180f) pitch -= 360f;
        float yaw = vt.eulerAngles.y;
        Vector3 aimDir = Quaternion.Euler(pitch, yaw, 0f) * Vector3.forward;

        // Raycast forward along the aim ray. If it hits something, use that distance so
        // the reticle lands on the target surface; otherwise fall back to the configured
        // projection distance. Smoothed to prevent snap when the ray transitions between
        // near and far surfaces (e.g. target passes out of LOS and the ray reaches sky).
        float targetDistance = reticleProjectionDistance;
        if (Physics.Raycast(vt.position, aimDir, out RaycastHit hit,
                            reticleProjectionDistance, reticleRaycastMask,
                            QueryTriggerInteraction.Ignore))
        {
            targetDistance = hit.distance;
        }

        _reticleDistance = Mathf.Lerp(_reticleDistance, targetDistance,
                                      1f - Mathf.Exp(-reticleFollowSpeed * Time.deltaTime));

        Vector3 aimWorldPoint = vt.position + aimDir * _reticleDistance;

        Vector3 screenPos = cam.WorldToScreenPoint(aimWorldPoint);
        if (screenPos.z < 0f) return; // behind camera

        RectTransform reticleRect = reticleImage.rectTransform;
        RectTransform canvasRect = reticleRect.parent as RectTransform;
        if (canvasRect != null &&
            RectTransformUtility.ScreenPointToLocalPointInRectangle(
                canvasRect, screenPos, null, out Vector2 localPoint))
        {
            reticleRect.localPosition = localPoint + reticleScreenOffset;
        }
    }

    private void SetMissileLockGroupVisible(bool visible)
    {
        if (missileLockGroup != null)
            missileLockGroup.SetActive(visible);
    }

    // =========================================================================
    // Null-safe UI helpers
    // =========================================================================

    private static void SetFill(Image image, float normalized)
    {
        if (image != null)
            image.fillAmount = Mathf.Clamp01(normalized);
    }

    private static void SetText(TextMeshProUGUI label, string value)
    {
        if (label != null)
            label.text = value;
    }
}
