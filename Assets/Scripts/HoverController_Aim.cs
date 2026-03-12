using UnityEngine;

/// <summary>
/// HoverController_Aim v1.0
/// ------------------------
/// Responsibilities:
///   • Computes an intentional world-space aim rotation each LateUpdate.
///   • Applies that rotation to the active weapon slot's vfxMount transform,
///     keeping particle emitters aimed correctly regardless of terrain-induced
///     vehicle pitch or roll.
///
/// Aim rotation is constructed from two components only:
///   • Yaw  — taken from the vehicle's world Y euler. Weapons always aim with heading.
///   • Pitch — derived from CameraLookY input, scaled by Propulsion.StrafePitchLimit
///             and blended by Propulsion.StrafeModeBlend. Zero in drive mode.
///
/// What it deliberately ignores:
///   • Vehicle X (pitch) from terrain reaction — Foundation's leveling corrections
///     should not affect bullet direction.
///   • Vehicle Z (roll) — never intentional, never communicated to weapons.
///
/// Design contract:
///   • Owns no physics. Rotation is applied to vfxMount only — a visual/VFX node.
///   • HoverController_Weapons notifies this module when the active slot changes
///     via NotifySlotChanged(WeaponSlot). Aim module caches the active vfxMount.
///   • Only the active slot's vfxMount is updated per frame. Inactive mounts
///     are left at their last orientation — they don't need updating while inactive.
///   • Instantiated-mode weapons (muzzle points) are not affected. Muzzle point
///     orientation is the designer's responsibility via transform placement.
/// </summary>
[DefaultExecutionOrder(-10)]
[RequireComponent(typeof(HoverController_Propulsion))]
[RequireComponent(typeof(HoverController_Weapons))]
public class HoverController_Aim : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // Inspector
    // -------------------------------------------------------------------------

    [Header("🕹 Input")]
    [Tooltip("MonoBehaviour implementing IHoverInputProvider. " +
             "Must be the same provider assigned to Propulsion and Weapons.")]
    [SerializeField] private MonoBehaviour inputProvider;

    [Header("🛠 Debug")]
    [Tooltip("Draw the computed aim direction as a ray in the Scene view.")]
    [SerializeField] private bool drawDebug = true;

    // -------------------------------------------------------------------------
    // Private
    // -------------------------------------------------------------------------

    private IHoverInputProvider    input;
    private HoverController_Propulsion propulsion;

    // The vfxMount of the currently active weapon slot. Null if no active slot
    // or active slot is Instantiated mode with no vfxMount assigned.
    private Transform activeMount;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------

    private void Awake()
    {
        propulsion = GetComponent<HoverController_Propulsion>();
        input      = inputProvider as IHoverInputProvider;

        if (input == null)
            Debug.LogError("[HoverController_Aim] inputProvider is null or missing " +
                           "IHoverInputProvider. Aim will not function.", this);
    }

    /// <summary>
    /// LateUpdate: runs after Propulsion FixedUpdate has applied pitch torque.
    /// Guarantees aim rotation is computed against the vehicle's settled orientation
    /// for this frame rather than fighting physics mid-step.
    /// </summary>
    private void LateUpdate()
    {
        if (input == null || propulsion == null || activeMount == null)
            return;

        Quaternion aimRotation = ComputeAimRotation();
        activeMount.rotation   = aimRotation;

        if (drawDebug)
            Debug.DrawRay(activeMount.position, aimRotation * Vector3.forward * 5f, Color.cyan);
    }

    // -------------------------------------------------------------------------
    // Aim computation
    // -------------------------------------------------------------------------

    /// <summary>
    /// Constructs a world-space aim rotation from vehicle yaw and intentional pitch only.
    ///
    /// Yaw: vehicle's current world Y euler angle. Always applied — weapons aim with heading.
    ///
    /// Pitch: CameraLookY input mapped to [-StrafePitchLimit, +StrafePitchLimit],
    ///        scaled by StrafeModeBlend so pitch authority fades in/out with strafe mode.
    ///        Stick up (+1) = nose up = negative pitch in Unity convention.
    ///        Zero in drive mode (StrafeModeBlend == 0).
    ///
    /// Roll: always zero. Never intentional, never communicated to weapons.
    /// </summary>
    private Quaternion ComputeAimRotation()
    {
        float yaw   = transform.eulerAngles.y;

        float aimY  = input.CameraLookY;
        if (Mathf.Abs(aimY) < 0.1f) aimY = 0f; // small deadzone, matches Propulsion

        float pitch = -aimY
                      * propulsion.StrafePitchLimit
                      * propulsion.StrafeModeBlend;

        return Quaternion.Euler(pitch, yaw, 0f);
    }

    // -------------------------------------------------------------------------
    // Public API — called by HoverController_Weapons
    // -------------------------------------------------------------------------

    /// <summary>
    /// Called by HoverController_Weapons when the active slot changes.
    /// Caches the new slot's vfxMount for per-frame rotation.
    /// Pass null to clear (e.g. when switching to an Instantiated-mode weapon).
    /// </summary>
    public void NotifySlotChanged(HoverController_Weapons.WeaponSlot slot)
    {
        activeMount = slot?.vfxMount;

        if (slot != null && slot.definition != null
            && slot.definition.projectileMode == ProjectileMode.ParticleSystem
            && activeMount == null)
        {
            Debug.LogWarning($"[HoverController_Aim] Active slot '{slot.definition.displayName}' " +
                             "is ParticleSystem mode but has no vfxMount assigned. " +
                             "Aim correction will not be applied.", this);
        }
    }
}
