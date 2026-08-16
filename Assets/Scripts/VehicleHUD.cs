using UnityEngine;
using UnityEngine.UI;
using TMPro;

/// <summary>
/// VehicleHUD v1.5
/// ----------------
/// v1.5 changes:
///   • The trick tracker reports what the trick was WORTH, not what fitted in the
///     pool. It used to print the granted energy, which is the pool's leftover
///     headroom and therefore an arbitrary number — a flip worth 25 landed at 82
///     energy read "+18", and the payout rounding to increments of five looked
///     broken when it was not. Now "+25", with a "(Full)" mark when the pool could
///     not take all of it and "(EMP)" when a freeze refused it outright. See
///     HandleTrickResolved.
///
/// v1.4 changes:
///   • Reticle no longer follows aim raycast depth. SyncReticle projects the aim
///     direction to a fixed reticleProjectionDistance instead of to whatever the ray
///     hit, which closes the long-standing "reticle jars when aiming on slopes"
///     complaint (TODO 2.8). Measurement and full reasoning are in SyncReticle.
///   • reticleRaycastMask and reticleFollowSpeed deleted — both existed only to serve
///     that raycast, and leaving them serialized would imply the raycast still runs.
///     reticleScreenOffset survives and is now the only way to nudge resting position,
///     alongside reticleProjectionDistance itself.
///   • Costs one Physics.Raycast per frame less, and the HUD no longer reads
///     vehicleRoot.layer, so it is no longer a VehicleLayerAssigner ordering consumer.
///
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

    [Tooltip("Convergence distance (meters) the crosshair is projected to. This is a FIXED " +
             "distance on purpose — it deliberately does not follow what the aim ray hits. " +
             "Raise it to sit the crosshair nearer the horizon, lower it to pull it down " +
             "toward the craft. Measured at 200: the crosshair rests ~438px above screen " +
             "centre; at 50 it rests ~381px, at 10 it rests ~200px. Below about 20 the " +
             "crosshair starts drifting toward the craft and reads as a chase marker rather " +
             "than a sight, so treat 20 as the practical floor.")]
    [SerializeField] private float reticleProjectionDistance = 200f;

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
    // 🌀 Trick tracker
    // =========================================================================
    [Header("🌀 Trick Tracker")]
    [Tooltip("Counts revolutions while a trick is in escrow, then reports the outcome. " +
             "Leave unassigned to disable the tracker entirely.")]
    [SerializeField] private TextMeshProUGUI trickText;

    [Tooltip("Colour while the trick is still in the air and can still be lost.")]
    [SerializeField] private Color trickColorPending = Color.white;

    [Tooltip("Colour when the trick is landed and paid. Match it to the energy bar, " +
             "because the payout IS energy and the two should read as the same currency.")]
    [SerializeField] private Color trickColorLanded = new Color(0.2f, 0.7f, 1f);

    [Tooltip("Colour when the trick is lost, either by landing badly or by going down.")]
    [SerializeField] private Color trickColorLost = new Color(0.9f, 0.15f, 0.1f);

    [Tooltip("How long the outcome holds at full opacity before it starts to fade.")]
    [SerializeField] private float trickOutcomeHold = 0.8f;

    [Tooltip("How long the outcome takes to fade away after the hold.")]
    [SerializeField] private float trickOutcomeFade = 0.5f;

    [Tooltip("Append the energy earned to a landed trick. Turn off for a cleaner read " +
             "once the payouts are tuned and you no longer need to see the number.")]
    [SerializeField] private bool trickShowPayout = true;

    // =========================================================================
    // Runtime references
    // =========================================================================

    private VehicleHealth              _health;
    private HoverController_Energy     _energy;
    private HoverController_Weapons    _weapons;
    private HoverController_Propulsion _propulsion;
    private HoverController_Shield     _shield;
    private HoverController_Tricks     _tricks;

    // Outcome playback. While this is running the tracker stops following the live
    // counts, because the flight it was describing is over and the last thing the
    // player saw should be what happened to it.
    private bool  _trickShowingOutcome;
    private float _trickOutcomeTimer;
    private Color _trickOutcomeColor;

    // Only EMP freeze is tracked locally — it has no per-frame readable equivalent
    // on the energy component that covers the full freeze window reliably.
    private bool _isEmpFrozen;
    private bool _isShieldActive;
    private bool _wasRegenerating;

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
        _tricks     = vehicleRoot.GetComponentInChildren<HoverController_Tricks>();

        if (_health     == null) Debug.LogWarning("[VehicleHUD] VehicleHealth not found on vehicleRoot.", this);
        if (_energy     == null) Debug.LogWarning("[VehicleHUD] HoverController_Energy not found on vehicleRoot.", this);
        if (_weapons    == null) Debug.LogWarning("[VehicleHUD] HoverController_Weapons not found on vehicleRoot.", this);
        if (_propulsion == null) Debug.LogWarning("[VehicleHUD] HoverController_Propulsion not found on vehicleRoot.", this);
        if (_shield     == null) Debug.LogWarning("[VehicleHUD] HoverController_Shield not found on vehicleRoot.", this);

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

        // Deliberately not warned about when missing, unlike the modules above: the
        // tracker is optional, and an AI craft has no Tricks component at all.
        if (_tricks != null)
            _tricks.OnTrickResolved += HandleTrickResolved;
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

        if (_tricks != null)
            _tricks.OnTrickResolved -= HandleTrickResolved;
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
        SyncTrickTracker();

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
    // Trick tracker
    // =========================================================================

    /// <summary>
    /// Follows the live revolution counts while a trick is in the air, and stands
    /// aside while an outcome is playing out.
    ///
    /// Polled rather than event-driven, unlike the rest of this class, because the
    /// counter changes continuously with rotation rather than on discrete
    /// transitions. Only the OUTCOME is an event, and that one genuinely is a
    /// moment. Same reasoning as the missile lock fill, which is polled while
    /// Scanning for exactly the same reason.
    /// </summary>
    private void SyncTrickTracker()
    {
        if (trickText == null)
            return;

        if (_trickShowingOutcome)
        {
            _trickOutcomeTimer -= Time.deltaTime;

            if (_trickOutcomeTimer <= 0f)
            {
                _trickShowingOutcome = false;
                trickText.text = string.Empty;
                return;
            }

            // The timer runs hold-then-fade as one countdown, so the fade is simply
            // the last trickOutcomeFade seconds of it.
            Color fading = _trickOutcomeColor;
            fading.a = trickOutcomeFade > 0f
                ? Mathf.Clamp01(_trickOutcomeTimer / trickOutcomeFade)
                : 1f;
            trickText.color = fading;
            return;
        }

        if (_tricks == null || !_tricks.IsTracking)
        {
            if (trickText.text.Length > 0)
                trickText.text = string.Empty;
            return;
        }

        string label = BuildTrickLabel(_tricks.BarrelRollCount, _tricks.FlipCount);

        // Empty until a full revolution is on the board. A trick reads as a trick
        // once it has come all the way round, and showing a fraction of one would
        // put a number on screen for every wobble.
        if (label.Length == 0)
        {
            if (trickText.text.Length > 0)
                trickText.text = string.Empty;
            return;
        }

        trickText.text  = label;
        trickText.color = trickColorPending;
    }

    /// <summary>"Barrel Roll x2 + Flip x1", or empty when nothing has come round yet.</summary>
    private static string BuildTrickLabel(int rolls, int flips)
    {
        if (rolls > 0 && flips > 0) return $"Barrel Roll x{rolls} + Flip x{flips}";
        if (rolls > 0)              return $"Barrel Roll x{rolls}";
        if (flips > 0)              return $"Flip x{flips}";
        return string.Empty;
    }

    /// <summary>
    /// Freezes the label at whatever the flight earned and recolours it by outcome.
    /// Reads the counts live because Tricks raises this before it clears them.
    ///
    /// Shows the PAYOUT, not the granted energy. The two differ only when the pool
    /// could not take the whole thing, and showing the granted figure made the
    /// number unreadable as a price: the pool sits at an arbitrary level because
    /// regen is continuous, so the leftover headroom is arbitrary too, and a trick
    /// worth a flat 25 would print a different number every time. The shortfall is
    /// still reported, as a mark rather than a different number, so the readout
    /// never claims energy that was not gained.
    /// </summary>
    private void HandleTrickResolved(bool banked, float payout, float granted)
    {
        if (trickText == null || _tricks == null)
            return;

        string label = BuildTrickLabel(_tricks.BarrelRollCount, _tricks.FlipCount);

        // Nothing ever appeared, so nothing should flash. A flight that never
        // completed a revolution is not a failed trick, it is just a jump.
        if (label.Length == 0)
        {
            _trickShowingOutcome = false;
            trickText.text = string.Empty;
            return;
        }

        if (banked && trickShowPayout && payout > 0f)
        {
            label += $"   +{Mathf.RoundToInt(payout)}";

            // Named separately rather than lumped together as one "not all of it"
            // mark, because they are different facts and the player's response to
            // each is different: a full pool means spend some, a freeze means wait.
            // The epsilon is for float noise in the grant, not a tolerance.
            if (granted < payout - 0.01f)
                label += (_energy != null && _energy.IsEmpFrozen) ? "  (EMP)" : "  (Full)";
        }

        _trickOutcomeColor   = banked ? trickColorLanded : trickColorLost;
        _trickOutcomeColor.a = 1f;

        trickText.text       = label;
        trickText.color      = _trickOutcomeColor;
        _trickShowingOutcome = true;
        _trickOutcomeTimer   = trickOutcomeHold + trickOutcomeFade;
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

        // Projected at a FIXED distance, deliberately not at the aim ray's hit distance.
        //
        // This used to raycast and project to whatever the ray hit. That is the more
        // "correct" impact marker, but it made the crosshair unusable, because the camera
        // sits behind and above the craft: sliding the world point along the aim line
        // sweeps it across the screen by parallax. Measured 2026-08-13, strafe rig:
        // 2m -> 21px BELOW screen centre, 200m -> 438px ABOVE it. A 459px swing on a
        // 1153px screen, driven entirely by what happened to be under the ray. Aiming
        // across a crest or past a wall threw the crosshair 40% of the screen height.
        // reticleFollowSpeed only smeared that over a few frames; it never removed it.
        //
        // The same measurement showed the swing is safe to discard. The rig is
        // LockToTargetNoRoll, so the camera pitches WITH the chassis: across the full
        // 23.7 degrees of strafe aim travel the crosshair moved ~10px. Depth outweighed
        // aim 45:1. So depth was contributing almost nothing but noise, and pinning it
        // costs almost none of the aim honesty it appeared to provide.
        //
        // The aim DIRECTION is untouched and still comes from the vehicle, sharing one
        // ray with HoverController_Aim.ComputeAimRotation, so the crosshair and the guns
        // can never disagree about where the shot goes. Muzzles fire parallel to this
        // ray, so a far convergence point is also the honest one: parallel rays share a
        // vanishing point, and the residual error at short range is the muzzle offset,
        // not the terrain.
        Vector3 aimWorldPoint = vt.position + aimDir * reticleProjectionDistance;

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
