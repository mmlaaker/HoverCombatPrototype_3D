using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// HoverController_Weapons v1.6
/// ----------------------------
/// Responsibilities:
///   • Manages a list of weapon slots, each referencing a shared WeaponDefinition asset.
///   • Cycles the active weapon slot on player input (D-Pad left/right, wraps).
///   • Gates all firing through EMP freeze check (HoverController_Energy.IsEmpFrozen).
///   • Dispatches fire to muzzle transforms or particle emitters assigned per slot.
///   • Supports four firing behaviors driven by WeaponType:
///       - SingleShot  : fires once per FirePressed event (Shotgun, Rail).
///       - Automatic   : fires repeatedly while FireHeld, gated by fire rate (Minigun).
///       - Missile     : FireHeld drives lock scanning; FirePressed commits on confirmed lock.
///       - Mine        : fires once per FirePressed; spawns at muzzle, no projectile velocity.
///   • Missile lock state machine: Idle → Scanning → Locked → Committed → Idle.
///   • Minigun wind-up: fire rate scales with sustained hold time via AnimationCurve.
///   • Broadcasts events for UI and audio hooks.
///
/// Design contracts:
///   • Weapons do NOT consume energy. Ammo is the only resource managed here.
///   • EMP freeze is read from HoverController_Energy.IsEmpFrozen — no energy is spent.
///   • This script owns no physics. Projectile behavior lives on the projectile prefab.
///   • WeaponDefinition is a shared ScriptableObject asset. All vehicles reference the
///     same assets — change damage or fire rate once and every vehicle updates.
///   • Vehicle-specific scene references (muzzlePoints, particleEmitters) live in
///     WeaponSlot, not in WeaponDefinition.
///
/// v1.6 changes:
///   • Input error message updated to match GetComponent standard across all scripts.
///   • Committed missile lock state now broadcasts OnMissileLockStateChanged explicitly
///     before resetting to Idle. Previously Committed was never surfaced to subscribers.
///   • Unreachable Committed switch case removed (state is entered and reset in the
///     same frame inside the Locked case — it can never be entered via the switch).
///   • ScanForLockTarget now gated to Idle/Scanning states only — no OverlapSphere
///     cost when already Locked or during non-missile weapon use.
///   • Physics.OverlapSphere replaced with Physics.OverlapSphereNonAlloc using a
///     pre-allocated Collider[16] buffer. Zero heap allocation per scan call.
///
/// v1.5 changes:
///   • WeaponDefinition promoted to ScriptableObject (own file: WeaponDefinition.cs).
///     Create assets via Assets → Create → Weapons → Weapon Definition.
///   • WeaponType and ProjectileMode moved to their own files.
///   • muzzlePoints and particleEmitters moved from WeaponDefinition into WeaponSlot.
///
/// v1.4 changes:
///   • useWindUp bool added. When false, windUpScale is always 1.0 — no lockout on release.
///   • useMissileLock bool added. When false, Missile type fires as dumbfire.
///
/// v1.3 changes:
///   • particleEmitter (single) replaced with particleEmitters (List).
///
/// v1.2 changes:
///   • ProjectileMode enum added (Instantiated / ParticleSystem).
///
/// v1.1 changes:
///   • Muzzle points moved into WeaponDefinition (now in WeaponSlot as of v1.5).
/// </summary>
[DefaultExecutionOrder(0)]
[RequireComponent(typeof(HoverController_Energy))]
public class HoverController_Weapons : MonoBehaviour
{
    // =========================================================================
    // 🎰 Weapon Slot
    // =========================================================================

    /// <summary>
    /// Per-vehicle runtime wrapper around a shared WeaponDefinition asset.
    /// Holds vehicle-specific scene references and all runtime state.
    /// One slot per entry in the weaponSlots list.
    /// </summary>
    [Serializable]
    public class WeaponSlot
    {
        [Tooltip("Shared weapon definition asset. Drag a WeaponDefinition asset from the Project window. " +
                 "All vehicles using the same asset share its damage, fire rate, and tuning.")]
        public WeaponDefinition definition;

        [Tooltip("Spawn origins for Instantiated projectiles. All fire simultaneously. " +
                 "Assign child transforms from this vehicle's mesh. " +
                 "Ignored when projectileMode is ParticleSystem.")]
        public List<Transform> muzzlePoints = new List<Transform>();

        [Tooltip("Pre-placed ParticleSystem emitters for this weapon on this vehicle. " +
                 "All start and stop together. Assign child ParticleSystems from this vehicle's hierarchy. " +
                 "Only used when projectileMode is ParticleSystem. " +
                 "Fire is suppressed if this list is empty in that mode.")]
        public List<ParticleSystem> particleEmitters = new List<ParticleSystem>();

        [Tooltip("Parent transform grouping all VFX for this weapon (e.g. VFX_MachineGuns). " +
                 "HoverController_Aim rotates this node to apply intentional aim direction. " +
                 "Children (individual emitters) follow automatically. " +
                 "Only needed for ParticleSystem mode weapons. Leave null for Instantiated weapons.")]
        public Transform vfxMount;

        /// <summary>Current ammo remaining. -1 indicates unlimited (maxAmmo == 0).</summary>
        [HideInInspector] public int currentAmmo;

        /// <summary>Seconds until this slot can fire again.</summary>
        [HideInInspector] public float cooldownRemaining;

        /// <summary>Wind-up progress for Automatic weapons. Range 0..1.</summary>
        [HideInInspector] public float windUpProgress;

        /// <summary>Initializes runtime state from the definition.</summary>
        public void Initialize()
        {
            if (definition == null) return;
            currentAmmo       = definition.maxAmmo == 0 ? -1 : definition.startingAmmo;
            cooldownRemaining = 0f;
            windUpProgress    = 0f;
        }

        /// <summary>True if ammo is unlimited or at least one round remains.</summary>
        public bool HasAmmo => currentAmmo == -1 || currentAmmo > 0;

        /// <summary>True when the cooldown has expired.</summary>
        public bool IsReady => cooldownRemaining <= 0f;
    }

    // =========================================================================
    // 🎯 Missile Lock State
    // =========================================================================

    /// <summary>
    /// State machine states for the missile lock-on system.
    /// Only meaningful when the active slot is WeaponType.Missile with useMissileLock true.
    /// </summary>
    public enum MissileLockState
    {
        /// <summary>No scan in progress.</summary>
        Idle,

        /// <summary>FireHeld is held and a valid target is in cone. Lock timer accumulating.</summary>
        Scanning,

        /// <summary>Lock confirmed. Waiting for FirePressed to commit.</summary>
        Locked,

        /// <summary>Missile committed and fired. Returns to Idle next frame.</summary>
        Committed
    }

    // =========================================================================
    // Inspector fields
    // =========================================================================

    [Header("🔫 Weapon Slots")]
    [Tooltip("Ordered list of weapon slots. Each references a shared WeaponDefinition asset " +
             "and holds vehicle-specific muzzle points, particle emitters, and runtime state.")]
    [SerializeField] private List<WeaponSlot> weaponSlots = new List<WeaponSlot>();

    [Header("🎯 Missile Lock")]
    [Tooltip("Layer mask for missile lock-on targets. Set to enemy vehicle layers only. " +
             "Default ~0 will lock onto terrain and props — always configure this.")]
    [SerializeField] private LayerMask lockTargetLayers = ~0;

    [Header("🛠 Debug")]
    [Tooltip("Draw muzzle points, emitter origins, and missile lock cone in the Scene view.")]
    [SerializeField] private bool drawDebug = true;

    // =========================================================================
    // 📢 Events
    // =========================================================================

    /// <summary>Fired when the active weapon slot changes. Parameter is the new slot index.</summary>
    public event Action<int> OnWeaponSwitched;

    /// <summary>Fired after every fire event. Parameters: slot index, ammo remaining (-1 = unlimited).</summary>
    public event Action<int, int> OnWeaponFired;

    /// <summary>Fired when a slot's ammo reaches zero.</summary>
    public event Action<int> OnAmmoDepletedForSlot;

    /// <summary>Fired on every missile lock state transition.</summary>
    public event Action<MissileLockState> OnMissileLockStateChanged;

    /// <summary>Fired when missile lock transitions to Locked.</summary>
    public event Action OnMissileLocked;

    // =========================================================================
    // Public read-only state
    // =========================================================================

    /// <summary>Index of the currently active weapon slot.</summary>
    public int ActiveSlotIndex { get; private set; }

    /// <summary>The currently active weapon slot. Null if slots list is empty.</summary>
    public WeaponSlot ActiveSlot => (weaponSlots != null && weaponSlots.Count > 0)
        ? weaponSlots[ActiveSlotIndex]
        : null;

    /// <summary>Current missile lock state.</summary>
    public MissileLockState CurrentLockState { get; private set; }

    /// <summary>Lock progress as 0..1 fraction. 1 = fully locked.</summary>
    public float LockProgress { get; private set; }

    /// <summary>Current lock target transform. Null when not scanning.</summary>
    public Transform LockTarget { get; private set; }

    // =========================================================================
    // Private runtime state
    // =========================================================================

    private IHoverInputProvider input;
    private HoverController_Energy energy;
    private HoverController_Aim aim; // optional — null if not present on vehicle

    private float lockTimer;
    private MissileLockState prevLockState;

    // Pre-allocated buffer for missile lock scans. OverlapSphereNonAlloc writes
    // into this array — no heap allocation per frame. Size 16 is generous for a
    // game with a small vehicle roster; increase if lock targets exceed this count.
    private readonly Collider[] _lockScanBuffer = new Collider[16];

    // =========================================================================
    // Unity lifecycle
    // =========================================================================

    private void Awake()
    {
        input  = GetComponent<IHoverInputProvider>();
        energy = GetComponent<HoverController_Energy>();
        aim    = GetComponent<HoverController_Aim>(); // optional — no error if absent

        if (input == null)
            Debug.LogError("[HoverController_Weapons] No IHoverInputProvider found on this GameObject. " +
                           "Attach PlayerHoverInput or an AI implementation.", this);
        if (energy == null)
            Debug.LogError("[HoverController_Weapons] No HoverController_Energy found.", this);

        InitializeSlots();
        aim?.NotifySlotChanged(ActiveSlot);
    }

    private void Update()
    {
        if (input == null || energy == null)
            return;

        TickCooldowns();
        TickCycleWeapon();

        if (energy.IsEmpFrozen)
            return;

        var slot = ActiveSlot;
        if (slot == null || slot.definition == null)
            return;

        switch (slot.definition.type)
        {
            case WeaponType.SingleShot: TickSingleShot(slot); break;
            case WeaponType.Automatic:  TickAutomatic(slot);  break;
            case WeaponType.Missile:    TickMissile(slot);    break;
            case WeaponType.Mine:       TickMine(slot);       break;
        }
    }

    // =========================================================================
    // Initialization
    // =========================================================================

    private void InitializeSlots()
    {
        for (int i = 0; i < weaponSlots.Count; i++)
        {
            if (weaponSlots[i] == null || weaponSlots[i].definition == null)
            {
                Debug.LogWarning($"[HoverController_Weapons] Slot {i} has no definition assigned.", this);
                continue;
            }
            weaponSlots[i].Initialize();
        }

        ActiveSlotIndex  = 0;
        CurrentLockState = MissileLockState.Idle;
        prevLockState    = MissileLockState.Idle;
        LockProgress     = 0f;
        LockTarget       = null;
        lockTimer        = 0f;
    }

    // =========================================================================
    // Cooldown tick
    // =========================================================================

    private void TickCooldowns()
    {
        foreach (var slot in weaponSlots)
        {
            if (slot == null) continue;
            if (slot.cooldownRemaining > 0f)
                slot.cooldownRemaining = Mathf.Max(0f, slot.cooldownRemaining - Time.deltaTime);
        }
    }

    // =========================================================================
    // Weapon cycling
    // =========================================================================

    private void TickCycleWeapon()
    {
        bool next = input.CycleWeaponNext;
        bool prev = input.CycleWeaponPrev;

        if ((!next && !prev) || weaponSlots.Count <= 1)
            return;

        int direction = next ? 1 : -1;
        int candidate = (ActiveSlotIndex + direction + weaponSlots.Count) % weaponSlots.Count;

        int attempts = 0;
        while ((weaponSlots[candidate] == null || weaponSlots[candidate].definition == null)
               && attempts < weaponSlots.Count)
        {
            candidate = (candidate + direction + weaponSlots.Count) % weaponSlots.Count;
            attempts++;
        }

        if (candidate == ActiveSlotIndex)
            return;

        if (ActiveSlot?.definition?.type == WeaponType.Missile)
            ResetLockState();

        ActiveSlotIndex = candidate;
        aim?.NotifySlotChanged(ActiveSlot);
        OnWeaponSwitched?.Invoke(ActiveSlotIndex);
    }

    // =========================================================================
    // Firing behaviors
    // =========================================================================

    private void TickSingleShot(WeaponSlot slot)
    {
        if (!input.FirePressed) return;
        if (!slot.IsReady)      return;
        if (!slot.HasAmmo)      return;

        FireAllMuzzles(slot);
    }

    /// <summary>
    /// Automatic: fires repeatedly while FireHeld.
    /// useWindUp false: windUpScale is always 1.0, constant fire rate from frame one.
    /// useWindUp true: fire rate scales via AnimationCurve over windUpDuration.
    /// ParticleSystem mode: drives emitter Play()/Stop() directly.
    /// Instantiated mode: spawns a projectile prefab per fire event.
    /// </summary>
    private void TickAutomatic(WeaponSlot slot)
    {
        var def = slot.definition;

        if (input.FireHeld)
        {
            if (def.useWindUp)
            {
                slot.windUpProgress = Mathf.Clamp01(
                    slot.windUpProgress + Time.deltaTime / def.windUpDuration
                );
            }
        }
        else
        {
            if (def.useWindUp)
            {
                slot.windUpProgress = def.windDownDuration > 0f
                    ? Mathf.Clamp01(slot.windUpProgress - Time.deltaTime / def.windDownDuration)
                    : 0f;
            }

            if (def.projectileMode == ProjectileMode.ParticleSystem)
                StopParticleEmitters(slot);
        }

        if (!input.FireHeld) return;
        if (!slot.HasAmmo)   return;

        float windUpScale = def.useWindUp
            ? def.windUpCurve.Evaluate(slot.windUpProgress)
            : 1f;

        if (windUpScale <= 0f)
        {
            if (def.projectileMode == ProjectileMode.ParticleSystem)
                StopParticleEmitters(slot);
            return;
        }

        if (def.projectileMode == ProjectileMode.ParticleSystem)
        {
            PlayParticleEmitters(slot);

            if (!slot.IsReady) return;

            slot.cooldownRemaining = (1f / def.fireRate) / Mathf.Max(0.001f, windUpScale);

            if (slot.currentAmmo > 0)
            {
                slot.currentAmmo--;
                if (slot.currentAmmo == 0)
                    OnAmmoDepletedForSlot?.Invoke(ActiveSlotIndex);
            }
            OnWeaponFired?.Invoke(ActiveSlotIndex, slot.currentAmmo);
        }
        else
        {
            if (!slot.IsReady) return;

            slot.cooldownRemaining = (1f / def.fireRate) / Mathf.Max(0.001f, windUpScale);

            FireAllMuzzles(slot);
        }
    }

    /// <summary>
    /// Missile: dumbfire or lock-on depending on useMissileLock.
    ///
    /// Lock state transitions:
    ///   Idle      → Scanning  : FireHeld begins while a target is in cone.
    ///   Scanning  → Locked    : Lock timer reaches lockAcquireTime.
    ///   Scanning  → Idle      : FireHeld released or target leaves cone.
    ///   Locked    → Committed : FirePressed while a ready slot has ammo.
    ///   Locked    → Idle      : FireHeld released.
    ///
    /// Committed is a one-frame broadcast state — the event fires, missile launches,
    /// and state resets to Idle in the same Update tick. It is never entered via the
    /// switch because the transition and reset happen together in the Locked case.
    /// OnMissileLockStateChanged fires for Committed explicitly before the reset
    /// so subscribers (UI, audio) reliably receive the "missile away" signal.
    /// </summary>
    private void TickMissile(WeaponSlot slot)
    {
        var def = slot.definition;

        if (!def.useMissileLock)
        {
            if (input.FirePressed && slot.IsReady && slot.HasAmmo)
                FireAllMuzzles(slot);
            return;
        }

        // Only scan for a target when actively looking — avoids OverlapSphere cost
        // every frame just because a missile weapon is equipped but not being used.
        bool targetInCone = (CurrentLockState == MissileLockState.Idle ||
                             CurrentLockState == MissileLockState.Scanning)
                            && ScanForLockTarget(def);

        switch (CurrentLockState)
        {
            case MissileLockState.Idle:
                if (input.FireHeld && targetInCone)
                    TransitionLockState(MissileLockState.Scanning);
                break;

            case MissileLockState.Scanning:
                if (!input.FireHeld || !targetInCone)
                {
                    ResetLockState();
                    break;
                }
                lockTimer   += Time.deltaTime;
                LockProgress = Mathf.Clamp01(lockTimer / def.lockAcquireTime);
                if (lockTimer >= def.lockAcquireTime)
                {
                    TransitionLockState(MissileLockState.Locked);
                    OnMissileLocked?.Invoke();
                }
                break;

            case MissileLockState.Locked:
                if (!input.FireHeld) { ResetLockState(); break; }
                if (input.FirePressed && slot.IsReady && slot.HasAmmo)
                {
                    // Transition to Committed, fire the event explicitly, then fire
                    // and reset — all in the same frame. Committed is intentionally a
                    // one-frame pulse: subscribers get the signal, but there's no
                    // separate tick needed for it.
                    TransitionLockState(MissileLockState.Committed);
                    OnMissileLockStateChanged?.Invoke(MissileLockState.Committed);
                    prevLockState = MissileLockState.Committed;
                    FireAllMuzzles(slot);
                    ResetLockState();
                }
                break;
        }

        if (CurrentLockState != prevLockState)
        {
            OnMissileLockStateChanged?.Invoke(CurrentLockState);
            prevLockState = CurrentLockState;
        }
    }

    private void TickMine(WeaponSlot slot)
    {
        if (!input.FirePressed) return;
        if (!slot.IsReady)      return;
        if (!slot.HasAmmo)      return;

        FireAllMuzzles(slot, applyVelocity: false);
    }

    // =========================================================================
    // Core fire dispatch
    // =========================================================================

    /// <summary>
    /// Routes a fire event based on projectileMode.
    /// Instantiated: spawns projectilePrefab at each muzzle in slot.muzzlePoints.
    /// ParticleSystem: calls Play() on each emitter in slot.particleEmitters.
    /// Ammo decrement and cooldown start after dispatch. Automatic manages its own cooldown.
    /// </summary>
    private void FireAllMuzzles(WeaponSlot slot, bool applyVelocity = true)
    {
        var def = slot.definition;

        if (def.projectileMode == ProjectileMode.ParticleSystem)
        {
            if (slot.particleEmitters == null || slot.particleEmitters.Count == 0)
            {
                Debug.LogWarning($"[HoverController_Weapons] Slot '{def.displayName}' is ParticleSystem mode " +
                                 "but slot has no particleEmitters assigned. Fire suppressed.", this);
                return;
            }

            PlayParticleEmitters(slot);

            if (drawDebug)
                foreach (var e in slot.particleEmitters)
                    if (e != null) Debug.DrawRay(e.transform.position, e.transform.forward * 3f, Color.yellow, 0.2f);
        }
        else
        {
            if (slot.muzzlePoints == null || slot.muzzlePoints.Count == 0)
            {
                Debug.LogWarning($"[HoverController_Weapons] Slot '{def.displayName}' has no muzzlePoints " +
                                 "assigned in the slot. Fire suppressed.", this);
                return;
            }

            if (def.projectilePrefab == null)
            {
                Debug.LogWarning($"[HoverController_Weapons] Slot '{def.displayName}' has no projectilePrefab " +
                                 "in the definition. Fire suppressed.", this);
                return;
            }

            foreach (var muzzle in slot.muzzlePoints)
            {
                if (muzzle == null) continue;
                var proj = Instantiate(def.projectilePrefab, muzzle.position, muzzle.rotation);
                proj.GetComponent<IProjectileDamageCarrier>()?.SetDamage(def.damage);
                if (drawDebug) Debug.DrawRay(muzzle.position, muzzle.forward * 3f, Color.red, 0.2f);
            }
        }

        if (slot.currentAmmo > 0)
        {
            slot.currentAmmo--;
            if (slot.currentAmmo == 0)
                OnAmmoDepletedForSlot?.Invoke(ActiveSlotIndex);
        }

        if (def.type != WeaponType.Automatic)
            slot.cooldownRemaining = 1f / def.fireRate;

        OnWeaponFired?.Invoke(ActiveSlotIndex, slot.currentAmmo);
    }

    // =========================================================================
    // Particle emitter helpers
    // =========================================================================

    private void PlayParticleEmitters(WeaponSlot slot)
    {
        foreach (var emitter in slot.particleEmitters)
        {
            if (emitter == null) continue;
            if (!emitter.isPlaying) emitter.Play();
        }
    }

    private void StopParticleEmitters(WeaponSlot slot)
    {
        foreach (var emitter in slot.particleEmitters)
        {
            if (emitter == null) continue;
            if (emitter.isPlaying) emitter.Stop();
        }
    }

    // =========================================================================
    // Missile lock helpers
    // =========================================================================

    private bool ScanForLockTarget(WeaponDefinition def)
    {
        Vector3 scanOrigin = transform.position + transform.forward * (def.lockRange * 0.5f);
        float   scanRadius = def.lockRange * Mathf.Tan(def.lockConeAngle * Mathf.Deg2Rad);

        // NonAlloc: writes into the pre-allocated buffer, returns hit count.
        // Zero heap allocation per call.
        int hitCount = Physics.OverlapSphereNonAlloc(scanOrigin, scanRadius, _lockScanBuffer, lockTargetLayers);

        Transform bestTarget = null;
        float     bestAngle  = def.lockConeAngle + 1f;

        for (int i = 0; i < hitCount; i++)
        {
            var hit = _lockScanBuffer[i];
            if (hit.transform.root == transform.root) continue;
            Vector3 toTarget = hit.transform.position - transform.position;
            if (toTarget.magnitude > def.lockRange) continue;
            float angle = Vector3.Angle(transform.forward, toTarget);
            if (angle < def.lockConeAngle && angle < bestAngle)
            {
                bestAngle  = angle;
                bestTarget = hit.transform;
            }
        }

        LockTarget = bestTarget;
        return bestTarget != null;
    }

    private void TransitionLockState(MissileLockState newState) => CurrentLockState = newState;

    private void ResetLockState()
    {
        CurrentLockState = MissileLockState.Idle;
        LockProgress     = 0f;
        lockTimer        = 0f;
        LockTarget       = null;
    }

    // =========================================================================
    // Public API
    // =========================================================================

    /// <summary>Refills ammo on a specific slot, or all slots if slotIndex is -1.</summary>
    public void RefillAmmo(int slotIndex = -1)
    {
        if (slotIndex == -1)
            foreach (var slot in weaponSlots) slot?.Initialize();
        else if (slotIndex >= 0 && slotIndex < weaponSlots.Count)
            weaponSlots[slotIndex]?.Initialize();
    }

    /// <summary>Forces the active slot to a specific index from external code.</summary>
    public void SetActiveSlot(int index)
    {
        if (index < 0 || index >= weaponSlots.Count) return;
        if (ActiveSlot?.definition?.type == WeaponType.Missile) ResetLockState();
        ActiveSlotIndex = index;
        aim?.NotifySlotChanged(ActiveSlot);
        OnWeaponSwitched?.Invoke(ActiveSlotIndex);
    }

    // =========================================================================
    // 🎨 Debug Gizmos
    // =========================================================================
#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (!drawDebug || !Application.isPlaying) return;

        var slot = ActiveSlot;
        if (slot?.definition == null) return;

        bool isParticleMode = slot.definition.projectileMode == ProjectileMode.ParticleSystem;
        Gizmos.color = isParticleMode ? Color.yellow : Color.red;

        if (isParticleMode)
        {
            foreach (var emitter in slot.particleEmitters)
            {
                if (emitter == null) continue;
                Gizmos.DrawWireSphere(emitter.transform.position, 0.1f);
                Gizmos.DrawRay(emitter.transform.position, emitter.transform.forward * 1f);
            }
        }
        else
        {
            foreach (var muzzle in slot.muzzlePoints)
            {
                if (muzzle == null) continue;
                Gizmos.DrawWireSphere(muzzle.position, 0.1f);
                Gizmos.DrawRay(muzzle.position, muzzle.forward * 1f);
            }
        }

        if (slot.definition.type == WeaponType.Missile && slot.definition.useMissileLock)
        {
            var def = slot.definition;
            Color coneColor = CurrentLockState switch
            {
                MissileLockState.Scanning => Color.yellow,
                MissileLockState.Locked   => Color.green,
                _                         => new Color(1f, 1f, 1f, 0.2f)
            };

            Gizmos.color = coneColor;
            Gizmos.DrawRay(transform.position, transform.forward * def.lockRange);

            if (LockTarget != null)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawLine(transform.position, LockTarget.position);
                Gizmos.DrawWireSphere(LockTarget.position, 0.5f);
            }

            UnityEditor.Handles.color = coneColor;
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 3.5f,
                $"LOCK [{CurrentLockState}] {LockProgress * 100f:F0}%"
            );
        }

        UnityEditor.Handles.color = Color.white;
        UnityEditor.Handles.Label(
            transform.position + Vector3.up * 4.2f,
            $"WEAPON [{ActiveSlotIndex}] {slot.definition.displayName} " +
            $"| AMMO {(slot.currentAmmo == -1 ? "\u221e" : slot.currentAmmo.ToString())} " +
            $"| CD {slot.cooldownRemaining:F2}s" +
            (energy.IsEmpFrozen ? " [EMP FROZEN]" : "")
        );
    }
#endif
}
