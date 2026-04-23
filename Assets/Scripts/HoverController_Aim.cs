using UnityEngine;

/// <summary>
/// HoverController_Aim v1.2
/// ------------------------
/// Responsibilities:
///   • Computes an intentional world-space aim rotation each LateUpdate.
///   • Applies that rotation to the active weapon slot's vfxMount transform,
///     keeping particle emitters aimed correctly regardless of terrain-induced
///     vehicle pitch or roll.
///
/// Aim rotation is constructed from two components only:
///   • Yaw  — taken from the vehicle's world Y euler. Weapons always aim with heading.
///   • Pitch — FPS-style free aim. Right Stick Y accumulates as continuous aim angle,
///             clamped to [-StrafePitchLimit, +StrafePitchLimit]. Resets to 0 on strafe exit.
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
///   • Input is acquired via GetComponent<IHoverInputProvider>() — attach
///     PlayerHoverInput or any AI implementation to this same GameObject.
///
/// v1.3 changes:
///   • Pitch now reads actual vehicle pitch (from Rigidbody orientation) instead of
///     maintaining an independent accumulated angle. Eliminates divergence between
///     where the vehicle body points and where bullets go — the vehicle IS the turret.
///   • Removed aimSensitivity and currentPitch accumulator — no longer needed.
///     Propulsion.ApplyStrafePitch drives the physics pitch; Aim just reads the result.
///
/// v1.2 changes:
///   • Pitch now FPS-style free aim instead of spring-loaded.
///   • Right Stick Y input accumulates as continuous angle change (degrees/frame).
///   • Aim angle is clamped to [-StrafePitchLimit, +StrafePitchLimit].
///   • Aim resets to 0 (forward) when exiting strafe mode for clean re-entry.
///   • New parameter: aimSensitivity (degrees per unit stick input per second).
///
/// v1.1 changes:
///   • Removed [DefaultExecutionOrder(-10)] — LateUpdate naturally runs after all
///     Update and FixedUpdate calls, so no explicit ordering is needed.
///   • inputProvider serialized field removed. Input now acquired via GetComponent
///     in Awake, consistent with Propulsion and Weapons.
/// </summary>
[RequireComponent(typeof(HoverController_Propulsion))]
[RequireComponent(typeof(HoverController_Weapons))]
public class HoverController_Aim : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // Inspector
    // -------------------------------------------------------------------------

    [Header("🛠 Debug")]
    [Tooltip("Draw the computed aim direction as a ray in the Scene view.")]
    [SerializeField] private bool drawDebug = true;

    [Tooltip("Optional global debug toggle. When assigned, overrides drawDebug.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.enableDebugGizmos : drawDebug;

    // -------------------------------------------------------------------------
    // Private
    // -------------------------------------------------------------------------

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
    }

    /// <summary>
    /// LateUpdate: runs after Propulsion FixedUpdate has applied pitch torque.
    /// Reads the vehicle's actual settled orientation — no independent state.
    /// </summary>
    private void LateUpdate()
    {
        if (propulsion == null || activeMount == null)
            return;

        Quaternion aimRotation = ComputeAimRotation();
        activeMount.rotation   = aimRotation;

        if (ShouldDrawDebug)
            Debug.DrawRay(activeMount.position, aimRotation * Vector3.forward * 5f, Color.cyan);
    }

    // -------------------------------------------------------------------------
    // Aim computation
    // -------------------------------------------------------------------------

    /// <summary>
    /// Constructs a world-space aim rotation from the vehicle's actual orientation.
    ///
    /// Yaw:   vehicle's current world Y euler. Weapons always aim with heading.
    /// Pitch: vehicle's current world X euler — the actual physics pitch driven by
    ///        Propulsion.ApplyStrafePitch and Foundation leveling. No independent
    ///        accumulation — eliminates divergence between body and bullet direction.
    /// Roll:  always zero. Never intentional, never communicated to weapons.
    /// </summary>
    private Quaternion ComputeAimRotation()
    {
        float pitch = HoverMath.NormalizeAngle(transform.eulerAngles.x);
        float yaw   = transform.eulerAngles.y;
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
