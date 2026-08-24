using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// HoverController_Weapons v1.1
///
/// v1.1: Weapon switching shuts down the weapon it switches away from. TickCycleWeapon
///       and SetActiveSlot were line-for-line copies and neither deactivated the
///       outgoing slot, so switching mid-burst on an Automatic left its emitter firing
///       permanently: Update only ticks the ACTIVE slot, which makes the
///       StopParticleEmitters call in TickAutomatic's idle branch unreachable the
///       instant a slot stops being active. HandleEmpFreeze had already worked out this
///       exact reasoning for the freeze lockout; nobody carried it across to switching.
///       Both paths now share SwitchToSlot, which stops the outgoing emitters, restores
///       their emission rate, clears wind-up charge, and drops any missile lock. Wind-up
///       is included because wind-down lives in the same unreachable branch, so a
///       minigun abandoned mid-spin used to keep its charge for free.
///
/// Manages a list of weapon slots, cycles the active slot on input, and dispatches
/// fire events to muzzle transforms or particle emitters.
///
/// Four firing behaviors driven by WeaponType:
///   SingleShot: fires once per FirePressed (Shotgun, Rail).
///   Automatic:  fires repeatedly while FireHeld, gated by fire rate (Minigun).
///   Missile:    Dumbfire, SoftHoming, or HardLock via MissileFireMode. See that enum for behavior.
///   Mine:       fires once per FirePressed; spawns at muzzle, no projectile velocity.
///
/// Two projectile modes via ProjectileMode:
///   Instantiated:    spawns a prefab at each muzzle.
///   ParticleSystem:  drives Play / Stop on pre-placed emitters per slot.
///
/// Missile lock state machine: Idle → Scanning → Locked → Committed → Idle.
/// Committed is a one-frame broadcast pulse that fires the event then resets.
///
/// Contracts:
///   Weapons do NOT consume energy. Ammo is the only resource managed here.
///   EMP freeze gates fire (read from HoverController_Energy.IsEmpFrozen). No
///   energy is ever spent on a fire event.
///   Owns no physics. Projectile behavior lives on the projectile prefab.
///   WeaponDefinition is a shared ScriptableObject. All vehicles can reference the
///   same asset; change damage or fire rate once and every vehicle updates.
///   Vehicle-specific scene references (muzzlePoints, particleEmitters, vfxMount)
///   live on WeaponSlot, not on WeaponDefinition.
/// </summary>
[RequireComponent(typeof(HoverController_Energy))]
public class HoverController_Weapons : MonoBehaviour
{
    // =========================================================================
    // 🎰 Weapon Slot
    // =========================================================================

    /// <summary>
    /// Per-vehicle runtime wrapper around a shared WeaponDefinition asset.
    /// Holds vehicle-specific scene references and runtime state.
    /// One slot per entry in the weaponSlots list.
    /// </summary>
    [Serializable]
    public class WeaponSlot
    {
        [Tooltip("Shared weapon definition asset. Drag a WeaponDefinition asset from the Project window. " +
                 "All vehicles using the same asset share its damage, fire rate, and tuning.")]
        public WeaponDefinition definition;

        [Tooltip("Spawn points for Instantiated projectiles. All fire simultaneously. " +
                 "Assign child transforms from this vehicle's mesh. Ignored in ParticleSystem mode.")]
        public List<Transform> muzzlePoints = new List<Transform>();

        [Tooltip("Pre-placed ParticleSystem emitters for this weapon on this vehicle. " +
                 "All start and stop together. Assign child ParticleSystems from this vehicle. " +
                 "Only used in ParticleSystem mode. Fire is suppressed if this list is empty in that mode.")]
        public List<ParticleSystem> particleEmitters = new List<ParticleSystem>();

        [Tooltip("Parent transform grouping all VFX for this weapon (e.g. VFX_MachineGuns). " +
                 "HoverController_Aim rotates this node to apply intentional aim direction; child emitters follow. " +
                 "Only needed for ParticleSystem mode. Leave null for Instantiated weapons.")]
        public Transform vfxMount;

        /// <summary>Current ammo remaining. -1 indicates unlimited (maxAmmo == 0).</summary>
        [HideInInspector] public int currentAmmo;

        /// <summary>Seconds until this slot can fire again.</summary>
        [HideInInspector] public float cooldownRemaining;

        /// <summary>Wind-up progress for Automatic weapons. Range 0..1.</summary>
        [HideInInspector] public float windUpProgress;

        /// <summary>
        /// Inspector-configured rateOverTime.constant per emitter, captured on Initialize.
        /// Used as the unscaled "full spin" rate that wind-up modulates against.
        /// Parallel list to particleEmitters.
        /// </summary>
        [HideInInspector] public List<float> baseEmitterRates = new List<float>();

        /// <summary>Initializes runtime state from the definition.</summary>
        public void Initialize()
        {
            if (definition == null) return;
            currentAmmo       = definition.combat.maxAmmo == 0 ? -1 : definition.combat.startingAmmo;
            cooldownRemaining = 0f;
            windUpProgress    = 0f;

            // The full-rate baseline comes from the definition, NOT from whatever the emitter
            // currently reads. Sampling the emitter created an ordering trap: ParticleWeaponCollision
            // writes the definition into the emitter in its own Awake, so whichever Awake ran second
            // decided whether wind-up scaled against the real rate or a stale authored one.
            // Reading the definition directly removes the ordering dependency entirely.
            baseEmitterRates.Clear();
            for (int i = 0; i < particleEmitters.Count; i++)
                baseEmitterRates.Add(definition.emitter.emissionRate);
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
    /// Only meaningful when the active slot is WeaponType.Missile with MissileFireMode.HardLock.
    /// </summary>
    public enum MissileLockState
    {
        /// <summary>No scan in progress.</summary>
        Idle,

        /// <summary>FireHeld is held and a valid target is in cone. Lock timer accumulating.</summary>
        Scanning,

        /// <summary>Lock confirmed. Waiting for FirePressed to commit.</summary>
        Locked,

        /// <summary>Missile committed and fired. Resets to Idle next frame.</summary>
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
    [Tooltip("Layers that missiles can lock onto. Set to enemy vehicle layers only. " +
             "Default ~0 will lock onto terrain and props, so always configure this.")]
    [SerializeField] private LayerMask lockTargetLayers = ~0;

    [Header("🛠 Debug")]
    [Tooltip("Draw muzzle points, emitter origins, and missile lock cone in the Scene view.")]
    [SerializeField] private bool drawDebug = true;

    [Tooltip("Optional global debug toggle. When assigned, overrides Draw Debug.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.IsEnabled(HoverDebugCategory.Weapons) : drawDebug;

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

    /// <summary>
    /// Current lock target. The one being acquired if a lock is in progress, otherwise the
    /// first committed target.
    ///
    /// Kept as a single Transform because the HUD, the debug gizmo and the SoftHoming path
    /// all read it and all of them only ever want "the thing we are pointed at". A volley
    /// reads <see cref="LockTargets"/> instead.
    /// </summary>
    public Transform LockTarget =>
        _lockCandidate != null ? _lockCandidate
                               : (_lockTargets.Count > 0 ? _lockTargets[0] : null);

    /// <summary>Every target committed by the current hold. One missile fires per entry.</summary>
    public IReadOnlyList<Transform> LockTargets => _lockTargets;

    /// <summary>How many locks are banked. Drives the volley size and the HUD readout.</summary>
    public int LockedCount => _lockTargets.Count;

    /// <summary>
    /// True when releasing right now would actually launch something. False while a hold is
    /// still below minLocksToFire, where releasing wastes the attempt entirely.
    ///
    /// Exposed for the HUD: "you have locks but not enough to fire" is a distinct state from
    /// "you are locked and ready", and without it the meter cannot tell the player which one
    /// they are in.
    /// </summary>
    public bool HasFiringSolution
    {
        get
        {
            var def = ActiveSlot?.definition;
            if (def == null) return false;
            int min = Mathf.Clamp(def.weaponLock.minLocksToFire, 1, Mathf.Max(1, def.weaponLock.maxLocks));
            return _lockTargets.Count >= min;
        }
    }

    // =========================================================================
    // Private runtime state
    // =========================================================================

    private IHoverInputProvider input;
    private HoverController_Energy energy;
    private HoverController_Aim aim; // optional. Null if not present on vehicle.

    private float lockTimer;
    private MissileLockState prevLockState;

    /// <summary>The target currently being acquired, before it is banked into _lockTargets.</summary>
    private Transform _lockCandidate;

    /// <summary>Committed locks, in acquisition order. One missile per entry on release.</summary>
    private readonly List<Transform> _lockTargets = new List<Transform>();

    /// <summary>
    /// Targets copied out at release, so a staggered volley survives ResetLockState clearing
    /// the live list. Reused rather than reallocated per volley.
    /// </summary>
    private readonly List<Transform> _volleyBuffer = new List<Transform>();

    /// <summary>The in-flight staggered launch, if one is running. Null when firing all at once.</summary>
    private Coroutine _volleyRoutine;

    // Pre-allocated buffer for missile lock scans. OverlapSphereNonAlloc writes
    // into this array, so there is no heap allocation per frame. Size 16 is
    // generous for a small vehicle roster; raise if lock targets exceed this count.
    private readonly Collider[] _lockScanBuffer = new Collider[16];

    // =========================================================================
    // Unity lifecycle
    // =========================================================================

    private void Awake()
    {
        input  = GetComponent<IHoverInputProvider>();
        energy = GetComponent<HoverController_Energy>();
        aim    = GetComponent<HoverController_Aim>(); // optional, no error if absent

        if (input == null)
            Debug.LogError("[HoverController_Weapons] No IHoverInputProvider found on this GameObject. " +
                           "Attach PlayerHoverInput or an AI implementation.", this);
        if (energy == null)
            Debug.LogError("[HoverController_Weapons] No HoverController_Energy found.", this);

        InitializeSlots();
        aim?.NotifySlotChanged(ActiveSlot);
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
    }

    // Snap all active firing/lock state to a cold-start condition on freeze entry.
    // Without this, a spinning minigun emitter would continue playing visually
    // because the early-out in Update prevents TickAutomatic from ever calling
    // StopParticleEmitters.
    private void HandleEmpFreeze(float _)
    {
        for (int i = 0, count = weaponSlots.Count; i < count; i++)
        {
            var slot = weaponSlots[i];
            if (slot == null) continue;
            StopParticleEmitters(slot);
            slot.windUpProgress = 0f;
        }

        ResetLockState();
    }

    private void Update()
    {
        if (input == null || energy == null)
            return;

        // EMP freeze: complete weapon lockout. Cooldowns pause, switching blocked,
        // firing blocked, missile lock progress frozen. HandleEmpFreeze (subscribed
        // to OnEmpFreezeApplied) has already stopped emitters and reset lock state.
        if (energy.IsEmpFrozen)
            return;

        TickCooldowns();
        TickCycleWeapon();

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
        _lockCandidate   = null;
        _lockTargets.Clear();
        lockTimer        = 0f;
    }

    // =========================================================================
    // Cooldown tick
    // =========================================================================

    private void TickCooldowns()
    {
        for (int i = 0, count = weaponSlots.Count; i < count; i++)
        {
            var slot = weaponSlots[i];
            if (slot != null && slot.cooldownRemaining > 0f)
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

        SwitchToSlot(candidate);
    }

    /// <summary>
    /// The single path for changing weapons. Both the input-driven cycle and the
    /// public setter route through here, because they were previously line-for-line
    /// copies of each other and a fix applied to one silently missed the other.
    ///
    /// Deactivating the OUTGOING slot is the load-bearing part, and it is the exact
    /// failure HandleEmpFreeze already documents one screen up: Update only ticks the
    /// ACTIVE slot, so the instant a slot stops being active, TickAutomatic can never
    /// run for it again and the StopParticleEmitters call in its idle branch becomes
    /// unreachable. Switching away from a firing Machine Gun or Chain Gun therefore
    /// left that emitter running for the rest of the match, one more orphan per
    /// switch, with no way to ever stop it. The freeze path worked out this reasoning
    /// for EMP and nobody carried it across to switching, which is the same early-out
    /// with a different trigger.
    ///
    /// windUpProgress is cleared on the same precedent. Wind-down also lives in that
    /// unreachable idle branch, so a minigun switched away from mid-spin kept its
    /// charge indefinitely and handed it straight back when reselected. Spinning up
    /// should have to be paid for again.
    ///
    /// The emission rate is restored to its baseline because the wind-down path
    /// leaves it scaled, and a slot abandoned mid-spool would otherwise be reselected
    /// still throttled down.
    ///
    /// The same-index early-out is a correctness guard, not an optimisation: without
    /// it, calling SetActiveSlot with the current index while firing would stop your
    /// own emitters mid-burst.
    /// </summary>
    private void SwitchToSlot(int index)
    {
        if (index < 0 || index >= weaponSlots.Count) return;
        if (index == ActiveSlotIndex)               return;

        var outgoing = ActiveSlot;
        if (outgoing != null)
        {
            if (outgoing.definition?.type == WeaponType.Missile)
                ResetLockState();

            StopParticleEmitters(outgoing);
            SetEmitterRateMultiplier(outgoing, 1f);
            outgoing.windUpProgress = 0f;
        }

        ActiveSlotIndex = index;
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
    /// ParticleSystem mode drives emitter Play / Stop directly.
    /// Instantiated mode spawns a projectile per fire event.
    /// </summary>
    private void TickAutomatic(WeaponSlot slot)
    {
        var def = slot.definition;

        if (input.FireHeld)
        {
            if (def.windUp.useWindUp)
            {
                slot.windUpProgress = Mathf.Clamp01(
                    slot.windUpProgress + Time.deltaTime / def.windUp.windUpDuration
                );
            }
        }
        else
        {
            if (def.windUp.useWindUp)
            {
                slot.windUpProgress = def.windUp.windDownDuration > 0f
                    ? Mathf.Clamp01(slot.windUpProgress - Time.deltaTime / def.windUp.windDownDuration)
                    : 0f;
            }

            // ParticleSystem + useWindUp: keep emitters playing at the decaying rate
            // while wind-down is in progress so the stream visibly spools down instead
            // of cutting off. No ammo decrement or fire events during this coast.
            // Stops once windUpProgress reaches 0, or immediately for non-windup weapons.
            bool visualWindDown = def.projectileMode == ProjectileMode.ParticleSystem
                                  && def.windUp.useWindUp
                                  && slot.windUpProgress > 0f;

            if (visualWindDown)
            {
                float windDownScale = def.windUp.windUpCurve.Evaluate(slot.windUpProgress);
                SetEmitterRateMultiplier(slot, windDownScale);
                PlayParticleEmitters(slot);
            }
            else if (def.projectileMode == ProjectileMode.ParticleSystem)
            {
                StopParticleEmitters(slot);
            }
        }

        if (!input.FireHeld) return;
        if (!slot.HasAmmo)   return;

        float windUpScale = def.windUp.useWindUp
            ? def.windUp.windUpCurve.Evaluate(slot.windUpProgress)
            : 1f;

        if (windUpScale <= 0f)
        {
            if (def.projectileMode == ProjectileMode.ParticleSystem)
                StopParticleEmitters(slot);
            return;
        }

        if (def.projectileMode == ProjectileMode.ParticleSystem)
        {
            // Scale emitter rateOverTime by the wind-up curve so the visual bullet
            // stream ramps with the spin-up, not just the ammo decrement cadence.
            if (def.windUp.useWindUp)
                SetEmitterRateMultiplier(slot, windUpScale);

            PlayParticleEmitters(slot);

            if (!slot.IsReady) return;

            slot.cooldownRemaining = (1f / def.combat.fireRate) / Mathf.Max(0.001f, windUpScale);

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

            slot.cooldownRemaining = (1f / def.combat.fireRate) / Mathf.Max(0.001f, windUpScale);

            FireAllMuzzles(slot);
        }
    }

    /// <summary>
    /// Missile dispatch. Routes to one of three sub-modes per WeaponDefinition.missileFireMode:
    ///   Dumbfire:   FirePressed fires immediately. No homing.
    ///   SoftHoming: FirePressed fires immediately. A single cone scan picks the best
    ///               in-cone target and passes it to the missile via IHomingTarget.
    ///               If no target is in cone, the missile fires straight.
    ///   HardLock:   FireHeld accumulates a lock; releasing FireHeld while Locked
    ///               launches the missile. Releasing while Scanning aborts.
    /// HardLock state machine lives in TickHardLockMissile.
    /// </summary>
    private void TickMissile(WeaponSlot slot)
    {
        var def = slot.definition;

        switch (def.weaponLock.missileFireMode)
        {
            case MissileFireMode.Dumbfire:
                if (input.FirePressed && slot.IsReady && slot.HasAmmo)
                    FireAllMuzzles(slot);
                return;

            case MissileFireMode.SoftHoming:
#if UNITY_EDITOR
                UpdateSoftHomingPreview(def);
#endif
                if (input.FirePressed && slot.IsReady && slot.HasAmmo)
                {
                    // Scan once at fire time. ScanForLockTarget assigns LockTarget
                    // (best in-cone target, or null if nothing qualifies).
                    // FireAllMuzzles reads LockTarget and forwards it via IHomingTarget.
                    ScanForLockTarget(def);
                    FireAllMuzzles(slot);
                    _lockCandidate = null; // clear so the gizmo doesn't persist
                }
                return;

            case MissileFireMode.HardLock:
                TickHardLockMissile(slot);
                return;
        }
    }

    /// <summary>
    /// HardLock state machine. Hold fire to scan and acquire; RELEASE fire to launch.
    ///
    /// Lock state transitions:
    ///   Idle      → Scanning  : FireHeld begins while a target is in cone. That target
    ///                           is COMMITTED at this moment and held for the whole lock.
    ///   Scanning  → Locked    : Lock timer reaches lockAcquireTime.
    ///   Scanning  → Idle      : FireHeld released, or the committed target stops being
    ///                           lockable (destroyed, disabled, out of range or cone).
    ///   Locked    → Committed : FireHeld released while a ready slot has ammo.
    ///   Locked    → Idle      : FireHeld released without ammo or while on cooldown,
    ///                           or the committed target stops being lockable.
    ///
    /// Committed is a one-frame broadcast pulse. The event fires, missile launches,
    /// and state resets to Idle in the same Update tick. It is never entered via
    /// the switch because the transition and reset happen together in the Locked
    /// case. OnMissileLockStateChanged fires for Committed explicitly before the
    /// reset so subscribers (UI, audio) reliably receive the "missile away" signal.
    ///
    /// The lock COMMITS to one target and does not re-pick.
    ///
    /// It used to re-scan every frame while Idle or Scanning, and ScanForLockTarget
    /// assigns LockTarget to whatever is currently closest to the nose. lockTimer kept
    /// accumulating across that, so a second vehicle crossing nearer your centreline
    /// partway through the acquire silently inherited the elapsed lock time: the HUD
    /// showed an unbroken SCANNING → LOCKED and the missile launched at something you
    /// had never tracked. Unreachable with one opponent, guaranteed with three to five.
    ///
    /// This is also the primitive multi-target lock needs. A cascade is "commit a
    /// target, keep holding, commit the next", so the committed target has to be a
    /// thing the state machine owns rather than something re-derived from cone
    /// geometry each frame. Extending this means promoting LockTarget to a list and
    /// looping Scanning back on itself with already-committed targets excluded from
    /// the scan; the validity rule and the release-to-fire trigger are unchanged.
    /// </summary>
    private void TickHardLockMissile(WeaponSlot slot)
    {
        var def      = slot.definition;
        int maxLocks = Mathf.Max(1, def.weaponLock.maxLocks);

        switch (CurrentLockState)
        {
            case MissileLockState.Idle:
                // The only place a target is chosen. Scanning past this point would
                // let a later contender inherit the lock.
                if (input.FireHeld && ScanForLockTarget(def))
                    TransitionLockState(MissileLockState.Scanning);
                break;

            case MissileLockState.Scanning:
                // Releasing part-way through a cascade goes through the same gate as a full
                // release: ReleaseVolley fires only if minLocksToFire is met, and otherwise
                // drops the whole hold. Letting a partial release always fire turned the
                // weapon into cheap guaranteed-homing spam, which is the Soft Homing
                // Missile's job and available here for a fraction of the commitment.
                if (!input.FireHeld)
                {
                    ReleaseVolley(slot);
                    break;
                }

                if (!IsStillLockable(_lockCandidate, def))
                {
                    // Losing the CANDIDATE does not discard locks already banked. Try to
                    // move on to somebody else; if there is nobody, hold what we have.
                    _lockCandidate = null;
                    lockTimer      = 0f;
                    if (!AcquireNextCandidate(def, maxLocks))
                    {
                        if (_lockTargets.Count > 0) TransitionLockState(MissileLockState.Locked);
                        else                        ResetLockState();
                    }
                    break;
                }

                lockTimer += Time.deltaTime;
                UpdateVolleyProgress(def, maxLocks);

                if (lockTimer >= def.weaponLock.lockAcquireTime)
                {
                    _lockTargets.Add(_lockCandidate);
                    _lockCandidate = null;
                    lockTimer      = 0f;

                    // Cascade: keep holding and the next lock starts immediately.
                    if (!AcquireNextCandidate(def, maxLocks))
                        TransitionLockState(MissileLockState.Locked);

                    UpdateVolleyProgress(def, maxLocks);
                }
                break;

            case MissileLockState.Locked:
                // A held lock still has to be maintained. Without this the target can
                // die or fly behind you and the missile is still handed its transform:
                // VehicleHealth disables the GameObject rather than destroying it, so
                // the reference stays non-null and the missile homes onto a corpse.
                // Prune rather than drop everything: one target of four escaping the cone
                // should cost you that missile, not the whole volley.
                PruneDeadLocks(def);
                if (_lockTargets.Count == 0)
                {
                    ResetLockState();
                    break;
                }

                // Release-to-fire: releasing FireHeld is the launch trigger.
                // If we can fire (ammo + cooldown ready), commit and launch.
                // Otherwise the lock just drops.
                if (!input.FireHeld) ReleaseVolley(slot);
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
    /// <param name="homingTargetOverride">
    /// Which target this particular missile chases. A volley fires one missile per committed
    /// lock, so the target cannot come from shared state — every missile in the salvo would
    /// then inherit whichever lock happened to be first.
    /// Null falls back to <see cref="LockTarget"/>, which is the single-missile and
    /// SoftHoming behaviour.
    /// </param>
    private void FireAllMuzzles(WeaponSlot slot, bool applyVelocity = true,
                                Transform homingTargetOverride = null)
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

            // Non-Automatic weapons (SingleShot, Missile, Mine) need a stop-then-play
            // to restart the burst from scratch. Without this, the isPlaying guard in
            // PlayParticleEmitters skips Play() while particles from the previous burst
            // are still alive, blocking subsequent shots.
            if (def.type != WeaponType.Automatic)
                StopParticleEmitters(slot);

            PlayParticleEmitters(slot);

            if (ShouldDrawDebug)
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

                // Definition first: a projectile that reads its tuning live needs the asset before
                // anything else touches it, and certainly before its first Start/FixedUpdate.
                proj.GetComponent<IProjectileDefinitionCarrier>()?.SetDefinition(def);

                // Who fired it. Without this the firer is just another collider in its own
                // blast: full enemy-strength splash knockback, and full self-damage the moment
                // combat.damage stops being 0. With it, the firer is a case that can be
                // authored (never self-damaged; shove scaled by impact.selfImpactScale).
                proj.GetComponent<IProjectileOwner>()?.SetOwner(transform);

                // Legacy per-value pushes, kept so any prefab that implements only these keeps
                // working. RocketProjectile no longer needs them.
                proj.GetComponent<IProjectileDamageCarrier>()?.SetDamage(def.combat.damage);
                proj.GetComponent<IProjectileImpactCarrier>()?
                    .SetImpact(def.impact.impactForce, def.impact.splashImpactForce, def.impact.destabilizeFraction);

                proj.GetComponent<IHomingTarget>()?.SetTarget(homingTargetOverride ?? LockTarget);
                if (ShouldDrawDebug) Debug.DrawRay(muzzle.position, muzzle.forward * 3f, Color.red, 0.2f);
            }
        }

        if (slot.currentAmmo > 0)
        {
            slot.currentAmmo--;
            if (slot.currentAmmo == 0)
                OnAmmoDepletedForSlot?.Invoke(ActiveSlotIndex);
        }

        if (def.type != WeaponType.Automatic)
            slot.cooldownRemaining = 1f / def.combat.fireRate;

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

    /// <summary>
    /// Writes baseRate * multiplier into each emitter's rateOverTime.constant so the
    /// wind-up curve modulates the visible bullet stream. baseRate is captured on
    /// Initialize from the inspector-configured value; the multiplier here is
    /// expected to be windUpScale (0..1).
    /// Avoids EmissionModule.rateOverTimeMultiplier, which overwrites the constant
    /// in Constant mode rather than scaling against the inspector value.
    /// </summary>
    private void SetEmitterRateMultiplier(WeaponSlot slot, float multiplier)
    {
        var emitters = slot.particleEmitters;
        var bases    = slot.baseEmitterRates;
        int count    = Mathf.Min(emitters.Count, bases.Count);

        for (int i = 0; i < count; i++)
        {
            var emitter = emitters[i];
            if (emitter == null) continue;
            var emission = emitter.emission;
            var rate     = emission.rateOverTime;
            rate.constant = bases[i] * multiplier;
            emission.rateOverTime = rate;
        }
    }

    // =========================================================================
    // Missile lock helpers
    // =========================================================================

    /// <summary>
    /// Progress across the WHOLE volley, not the lock currently being acquired.
    ///
    /// The HUD polls LockProgress straight into a fill bar, so per-lock progress made a
    /// four-lock weapon fill and reset the meter four times in under two seconds. That reads
    /// as a glitch rather than as charging, and it hides the only thing the player needs to
    /// know: how close they are to having a volley worth releasing.
    /// </summary>
    private void UpdateVolleyProgress(WeaponDefinition def, int maxLocks)
    {
        float partial = _lockCandidate != null && def.weaponLock.lockAcquireTime > 0f
            ? Mathf.Clamp01(lockTimer / def.weaponLock.lockAcquireTime)
            : 0f;

        LockProgress = Mathf.Clamp01((_lockTargets.Count + partial) / Mathf.Max(1, maxLocks));
    }

    private bool ScanForLockTarget(WeaponDefinition def)
    {
        _lockCandidate = TargetingScan.PickBestInCone(
            transform,
            def.weaponLock.lockRange,
            def.weaponLock.lockConeAngle,
            lockTargetLayers,
            _lockScanBuffer);
        return _lockCandidate != null;
    }

    /// <summary>
    /// Picks the next thing to lock in a cascade, or reports that the volley is full.
    ///
    /// Prefers somebody NEW: a fresh scan with the committed list excluded, so a volley
    /// spreads across a crowd rather than piling onto whoever is nearest the nose.
    ///
    /// If nothing new qualifies, `allowRepeatLocks` decides whether the weapon can stack
    /// another missile onto a target it already holds. That switch is what makes the
    /// weapon work at all in a one-on-one fight, where by definition there is never a
    /// second target — without it a four-missile volley fires exactly one missile and
    /// reads as broken rather than as "designed for crowds".
    /// </summary>
    private bool AcquireNextCandidate(WeaponDefinition def, int maxLocks)
    {
        if (_lockTargets.Count >= maxLocks) return false;

        _lockCandidate = TargetingScan.PickBestInCone(
            transform, def.weaponLock.lockRange, def.weaponLock.lockConeAngle,
            lockTargetLayers, _lockScanBuffer, _lockTargets);

        if (_lockCandidate == null && def.weaponLock.allowRepeatLocks)
            _lockCandidate = TargetingScan.PickBestInCone(
                transform, def.weaponLock.lockRange, def.weaponLock.lockConeAngle,
                lockTargetLayers, _lockScanBuffer);

        return _lockCandidate != null;
    }

    /// <summary>
    /// Drops committed targets that have stopped qualifying, without disturbing the rest.
    /// </summary>
    private void PruneDeadLocks(WeaponDefinition def)
    {
        for (int i = _lockTargets.Count - 1; i >= 0; i--)
            if (!IsStillLockable(_lockTargets[i], def))
                _lockTargets.RemoveAt(i);
    }

    /// <summary>
    /// Launches one missile per committed lock, each handed its own target.
    ///
    /// Ammo is spent per missile and the volley is TRUNCATED to what can be paid for, so a
    /// four-lock hold with two rounds left fires two. Firing all four would put currentAmmo
    /// negative, and `HasAmmo` tests `> 0`, so the weapon would then be permanently unable
    /// to fire with no visible reason. Unreachable today because maxAmmo is 0 and ammo never
    /// decrements (TODO 1.4), which is exactly the kind of latent trap that surfaces months
    /// later as "the hard lock stopped working".
    /// </summary>
    private void ReleaseVolley(WeaponSlot slot)
    {
        // Below the minimum the whole hold is wasted. Without this gate the weapon degenerates
        // into cheap homing spam: bank one lock, release, and you have a guaranteed-tracking
        // missile for a fraction of a second of holding — strictly better than the Soft Homing
        // Missile, which is the weapon that is supposed to own that role.
        int minLocks = Mathf.Clamp(slot.definition.weaponLock.minLocksToFire,
                                   1, Mathf.Max(1, slot.definition.weaponLock.maxLocks));

        if (slot.IsReady && slot.HasAmmo && _lockTargets.Count >= minLocks)
        {
            TransitionLockState(MissileLockState.Committed);
            OnMissileLockStateChanged?.Invoke(MissileLockState.Committed);
            prevLockState = MissileLockState.Committed;

            int volley = _lockTargets.Count;
            if (slot.currentAmmo > 0) volley = Mathf.Min(volley, slot.currentAmmo);

            // Copy the targets out. ResetLockState clears the live list on the next line, and a
            // staggered volley reads them over the following fraction of a second.
            _volleyBuffer.Clear();
            for (int i = 0; i < volley; i++) _volleyBuffer.Add(_lockTargets[i]);

            float interval = slot.definition.weaponLock.volleyLaunchInterval;
            if (interval <= 0f)
            {
                for (int i = 0; i < _volleyBuffer.Count; i++)
                    FireAllMuzzles(slot, applyVelocity: true, homingTargetOverride: _volleyBuffer[i]);
            }
            else
            {
                // A second release cannot overlap the first: the buffer is shared, so the old
                // cascade would start reading the new volley's targets partway through.
                if (_volleyRoutine != null) StopCoroutine(_volleyRoutine);
                _volleyRoutine = StartCoroutine(FireVolleyStaggered(slot, interval));
            }
        }
        ResetLockState();
    }

    /// <summary>
    /// Walks the volley out one missile at a time so the salvo reads as a cascade rather than
    /// a single thump. Delays the LAUNCH; the missiles fly normally once away, so later ones
    /// also arrive later.
    ///
    /// Aborts if the player switches weapons. The unfired missiles have not been paid for yet
    /// (ammo is spent per launch), and firing them anyway would raise OnWeaponFired carrying
    /// whatever slot index is active by then, which the HUD would read as the WRONG weapon's
    /// ammo changing.
    ///
    /// Times with an explicit accumulator rather than WaitForSeconds, which allocates on every
    /// yield. A five-missile cascade would otherwise leave five objects for the collector on
    /// each trigger pull, which is the sort of thing 4.4 exists to stop accumulating.
    /// </summary>
    private System.Collections.IEnumerator FireVolleyStaggered(WeaponSlot slot, float interval)
    {
        int firedFromSlot = ActiveSlotIndex;

        for (int i = 0; i < _volleyBuffer.Count; i++)
        {
            if (ActiveSlotIndex != firedFromSlot) break;
            if (!slot.HasAmmo)                    break;

            FireAllMuzzles(slot, applyVelocity: true, homingTargetOverride: _volleyBuffer[i]);

            if (i == _volleyBuffer.Count - 1) break;

            float waited = 0f;
            while (waited < interval)
            {
                waited += Time.deltaTime;
                yield return null;
            }
        }

        _volleyRoutine = null;
    }

    /// <summary>
    /// Whether the ALREADY COMMITTED lock target still qualifies. Deliberately not a
    /// scan: a scan answers "what is the best target right now", which is the wrong
    /// question once a target has been committed, and answering it every frame is the
    /// bug this replaced.
    ///
    /// Three ways a lock is lost, and each has bitten something in this project:
    ///   Destroyed  - Unity's overloaded == reports a destroyed object as null.
    ///   Disabled   - VehicleHealth DISABLES the GameObject on death rather than
    ///                destroying it, so the reference stays genuinely non-null and only
    ///                activeInHierarchy separates a live target from a corpse.
    ///   Escaped    - out of lockRange, or outside lockConeAngle of our nose. Same
    ///                geometry TargetingScan uses, so acquiring and holding agree.
    /// </summary>
    private bool IsStillLockable(Transform target, WeaponDefinition def)
    {
        if (target == null || !target.gameObject.activeInHierarchy)
            return false;

        Vector3 toTarget = target.position - transform.position;

        if (toTarget.sqrMagnitude > def.weaponLock.lockRange * def.weaponLock.lockRange)
            return false;

        return Vector3.Angle(transform.forward, toTarget) < def.weaponLock.lockConeAngle;
    }

    private void TransitionLockState(MissileLockState newState) => CurrentLockState = newState;

    private void ResetLockState()
    {
        CurrentLockState = MissileLockState.Idle;
        LockProgress     = 0f;
        lockTimer        = 0f;
        _lockCandidate   = null;
        _lockTargets.Clear();
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

    /// <summary>
    /// Forces the active slot to a specific index from external code. Shares
    /// SwitchToSlot with the input-driven cycle so the outgoing weapon is shut down
    /// identically no matter which path selected the new one.
    /// </summary>
    public void SetActiveSlot(int index) => SwitchToSlot(index);

    // =========================================================================
    // 🎨 Debug Gizmos
    // =========================================================================
#if UNITY_EDITOR
    // Editor-only buffer for the soft-homing preview scan.
    // Separate from _lockScanBuffer so the preview never touches live lock state.
    private readonly Collider[] _gizmoScanBuffer = new Collider[16];

    private Transform _previewTarget;
    private float     _previewNextScan;

    /// <summary>
    /// Keeps the soft-homing preview target fresh on the GAME tick, throttled.
    ///
    /// SoftHoming holds no lock state between shots (LockTarget is cleared immediately after
    /// firing), so the gizmo has nothing to display unless something scans for it. It used to do
    /// that scan inside OnDrawGizmos, which is wrong twice over: OnDrawGizmos runs on the editor's
    /// repaint schedule rather than the game's, and the scan is an OverlapSphere whose radius is
    /// lockRange x tan(lockConeAngle) -- 80m for this weapon, 115m on WD_HardLockMissile. That put
    /// a large physics query on every repaint of every visible view.
    ///
    /// Running it here instead drops it from once-per-repaint (60-120+/sec, doubled if both Scene
    /// and Game views are showing gizmos) to 10/sec, and only while a soft-homing missile is the
    /// active slot AND the weapons debug category is on. The preview is a targeting aid; a tenth
    /// of a second of staleness is invisible.
    /// </summary>
    private void UpdateSoftHomingPreview(WeaponDefinition def)
    {
        if (!ShouldDrawDebug) { _previewTarget = null; return; }
        if (Time.unscaledTime < _previewNextScan) return;

        _previewNextScan = Time.unscaledTime + 0.1f;
        _previewTarget = TargetingScan.PickBestInCone(
            transform,
            def.weaponLock.lockRange,
            def.weaponLock.lockConeAngle,
            lockTargetLayers,
            _gizmoScanBuffer);
    }

    private void OnDrawGizmos()
    {
        if (!ShouldDrawDebug || !Application.isPlaying) return;

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

        if (slot.definition.type == WeaponType.Missile && slot.definition.weaponLock.missileFireMode != MissileFireMode.Dumbfire)
        {
            var def = slot.definition;
            bool softHoming = def.weaponLock.missileFireMode == MissileFireMode.SoftHoming;

            // Read only. The scan that produces this runs on the game tick in
            // UpdateSoftHomingPreview, throttled to 10Hz; doing it here meant a
            // large OverlapSphere on every repaint.
            Transform previewTarget = softHoming ? _previewTarget : null;

            Color coneColor = softHoming
                ? (previewTarget != null ? Color.cyan : new Color(1f, 1f, 1f, 0.2f))
                : CurrentLockState switch
                {
                    MissileLockState.Scanning => Color.yellow,
                    MissileLockState.Locked   => Color.green,
                    _                         => new Color(1f, 1f, 1f, 0.2f)
                };

            DrawLockCone(def.weaponLock.lockRange, def.weaponLock.lockConeAngle, coneColor);

            if (LockTarget != null)
            {
                Gizmos.color = Color.green;
                Gizmos.DrawLine(transform.position, LockTarget.position);
                Gizmos.DrawWireSphere(LockTarget.position, 0.5f);
            }
            else if (previewTarget != null)
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawLine(transform.position, previewTarget.position);
                Gizmos.DrawWireSphere(previewTarget.position, 0.5f);
            }

            UnityEditor.Handles.color = coneColor;
            UnityEditor.Handles.Label(
                transform.position + Vector3.up * 3.5f,
                softHoming
                    ? $"TRACK [{(previewTarget != null ? previewTarget.root.name : "no target")}]"
                    : $"LOCK [{CurrentLockState}] {LockProgress * 100f:F0}%"
            );
        }

        UnityEditor.Handles.color = Color.white;
        UnityEditor.Handles.Label(
            transform.position + Vector3.up * 4.2f,
            $"WEAPON [{ActiveSlotIndex}] {slot.definition.displayName} " +
            $"| AMMO {(slot.currentAmmo == -1 ? "∞" : slot.currentAmmo.ToString())} " +
            $"| CD {slot.cooldownRemaining:F2}s" +
            (energy.IsEmpFrozen ? " [EMP FROZEN]" : "")
        );
    }

    /// <summary>
    /// Draws the lock acquisition cone: center ray plus four edge rays around
    /// the vehicle's up and right axes. Same visual language as the EMP cone.
    /// </summary>
    private void DrawLockCone(float range, float halfAngleDegrees, Color color)
    {
        Vector3 origin = transform.position;
        Vector3 fwd    = transform.forward;

        Gizmos.color = color;
        Gizmos.DrawLine(origin, origin + fwd * range);

        Quaternion lRot = Quaternion.AngleAxis(-halfAngleDegrees, transform.up);
        Quaternion rRot = Quaternion.AngleAxis( halfAngleDegrees, transform.up);
        Quaternion uRot = Quaternion.AngleAxis(-halfAngleDegrees, transform.right);
        Quaternion dRot = Quaternion.AngleAxis( halfAngleDegrees, transform.right);

        Gizmos.DrawLine(origin, origin + lRot * fwd * range);
        Gizmos.DrawLine(origin, origin + rRot * fwd * range);
        Gizmos.DrawLine(origin, origin + uRot * fwd * range);
        Gizmos.DrawLine(origin, origin + dRot * fwd * range);

        // Cone base circle so the width reads at a glance in the Game view.
        UnityEditor.Handles.color = color;
        UnityEditor.Handles.DrawWireDisc(
            origin + fwd * range,
            fwd,
            range * Mathf.Tan(halfAngleDegrees * Mathf.Deg2Rad));
    }
#endif
}
