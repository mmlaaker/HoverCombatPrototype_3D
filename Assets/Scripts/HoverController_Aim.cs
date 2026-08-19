using UnityEngine;

/// <summary>
/// HoverController_Aim v1.0
///
/// Computes an intentional world-space aim rotation each LateUpdate and applies it
/// to the active weapon slot's vfxMount transform. Keeps particle emitters aimed
/// correctly regardless of terrain-induced pitch or roll.
///
/// Aim rotation is built from two components only:
///   Yaw:   the vehicle's actual world Y. Weapons aim with heading.
///   Pitch: the vehicle's actual world X. The physics pitch driven by
///          Propulsion.ApplyStrafePitch and Foundation leveling. No independent
///          accumulation; the vehicle IS the turret, in both modes.
///
/// TERRAIN PITCH IS NOT REMOVED HERE, AND THIS HEADER USED TO CLAIM OTHERWISE.
/// It said the module deliberately ignored vehicle pitch from terrain reaction. It
/// never did and, reading the hull's total pitch, never could: it keeps no commanded
/// aim of its own to subtract. Corrected 2026-08-19 with TODO 0.29.
///
/// What actually happens is mode-dependent, and it is decided upstream in
/// HoverController_Foundation.ApplyLevelingTorque rather than here:
///   Drive mode: the hull follows the ground, so the guns follow the ground. Point
///          the nose at the target. This is intended, and it is why a craft on a
///          ramp shooting at a craft on the same ramp hits it.
///   Aim mode:  the hull levels to the horizon instead, so the terrain drops out of
///          the shot direction without this module doing anything about it.
/// Reading the hull's achieved pitch is therefore still correct. It is correct
/// BECAUSE the hull is the single definition of where the craft is pointing.
///
/// What it deliberately ignores:
///   Vehicle roll. Never intentional, never communicated to weapons.
///
/// Contracts:
///   Owns no physics. Rotation is applied to vfxMount only, a visual node.
///   HoverController_Weapons notifies this module on slot change via
///   NotifySlotChanged(WeaponSlot). Aim caches the new vfxMount.
///   Only the active slot's vfxMount is updated per frame.
///   Instantiated-mode weapons are not affected. Muzzle point orientation is the
///   designer's responsibility via transform placement.
///   LateUpdate naturally runs after all Update and FixedUpdate calls; no
///   DefaultExecutionOrder needed.
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

    [Tooltip("Optional global debug toggle. When assigned, overrides Draw Debug.")]
    [SerializeField] private HoverDebugSettings debugSettings;

    private bool ShouldDrawDebug => debugSettings != null ? debugSettings.IsEnabled(HoverDebugCategory.Weapons) : drawDebug;

    // -------------------------------------------------------------------------
    // Private
    // -------------------------------------------------------------------------

    private HoverController_Propulsion propulsion;
    private HoverController_Energy     energy;

    // The vfxMount of the currently active weapon slot. Null if no active slot or
    // active slot is Instantiated mode with no vfxMount assigned.
    private Transform activeMount;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------

    private void Awake()
    {
        propulsion = GetComponent<HoverController_Propulsion>();
        energy     = GetComponent<HoverController_Energy>();
    }

    /// <summary>
    /// LateUpdate runs after Propulsion FixedUpdate has applied pitch torque.
    /// Reads the vehicle's actual settled orientation, no independent state.
    /// </summary>
    private void LateUpdate()
    {
        if (propulsion == null || activeMount == null)
            return;

        // EMP freeze: vfxMount holds its last rotation through the freeze. Since
        // weapons can't fire while frozen, drift on the visual mount is harmless.
        if (energy != null && energy.IsEmpFrozen)
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
    /// Yaw:   vehicle's current world Y. Weapons aim with heading.
    /// Pitch: vehicle's current world X. The actual physics pitch.
    /// Roll:  always zero. Never intentional, never communicated to weapons.
    /// </summary>
    private Quaternion ComputeAimRotation()
    {
        float pitch = HoverMath.NormalizeAngle(transform.eulerAngles.x);
        float yaw   = transform.eulerAngles.y;
        return Quaternion.Euler(pitch, yaw, 0f);
    }

    // -------------------------------------------------------------------------
    // Public API (called by HoverController_Weapons)
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
