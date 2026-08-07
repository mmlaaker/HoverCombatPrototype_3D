using System;
using UnityEngine;

/// <summary>
/// HoverController_EMP v2.0
///
/// Single-shot soft-homing projectile ability. On activation, performs a
/// forward cone scan via TargetingScan to acquire the best in-cone target,
/// then instantiates an EmpProjectile prefab from a muzzle point and hands
/// it the target Transform plus the freeze duration. The projectile flies
/// itself, bends toward the target if any, and applies the freeze on hit.
///
/// Energy cost is paid once on activation, before the projectile is spawned.
/// If the scan returns no target, the projectile still fires — it just flies
/// straight, same fallback used by SoftHoming missiles.
///
/// Visual is whatever ParticleSystem the EmpProjectile prefab parents under
/// itself. This component does not own any scene VFX.
///
/// v2.0 changes (from v1.0 vehicle-emitter design):
///   • Replaced ParticleSystem-on-vehicle with instantiated EmpProjectile prefab.
///   • Added soft-homing target acquisition via TargetingScan.
///   • Removed burst coroutine; projectile now owns its own lifetime.
/// </summary>
[RequireComponent(typeof(HoverController_Energy))]
public class HoverController_EMP : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // 📦 Tuning Profile
    // -------------------------------------------------------------------------
    [Header("📦 Tuning")]
    [Tooltip("Vehicle tuning profile (shared SO). Energy cost, freeze duration, and scan parameters live here.")]
    [SerializeField] private VehicleTuningProfile profile;

    private EmpTuning S => profile.emp;

    // -------------------------------------------------------------------------
    // 🚀 Projectile
    // -------------------------------------------------------------------------
    [Header("🚀 Projectile")]
    [Tooltip("EmpProjectile prefab. Instantiated once per activation. Contains the visual ParticleSystem as a child.")]
    [SerializeField] private EmpProjectile empProjectilePrefab;

    [Tooltip("Muzzle point. Projectile spawns at this transform's position and forward direction. " +
             "Place at the front of the vehicle aligned with intended firing direction.")]
    [SerializeField] private Transform muzzlePoint;

    [Tooltip("Layers eligible for soft-homing acquisition. Set to enemy vehicle layers only.")]
    [SerializeField] private LayerMask targetLayers = ~0;

    // -------------------------------------------------------------------------
    // 📢 Events
    // -------------------------------------------------------------------------
    public event Action OnEmpFired;

    // -------------------------------------------------------------------------
    // 🛠 Debug
    // -------------------------------------------------------------------------
    [Header("🛠 Debug")]
    [SerializeField] private HoverDebugSettings debugSettings;

    // -------------------------------------------------------------------------
    // Runtime references
    // -------------------------------------------------------------------------
    private IHoverInputProvider input;
    private HoverController_Energy energy;

    // Pre-allocated overlap buffer for the homing scan. Matches the missile pattern.
    private readonly Collider[] _scanBuffer = new Collider[16];

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------
    private void Awake()
    {
        if (profile == null)
        {
            Debug.LogError(
                $"[EMP] '{name}': VehicleTuningProfile is not assigned. EMP disabled.",
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
                $"[EMP] '{name}': No IHoverInputProvider found. " +
                $"Attach PlayerHoverInput or AIHoverInput. EMP disabled.",
                this
            );
            enabled = false;
            return;
        }

        if (empProjectilePrefab == null)
        {
            Debug.LogError(
                $"[EMP] '{name}': empProjectilePrefab is not assigned. EMP disabled.",
                this
            );
            enabled = false;
            return;
        }

        if (muzzlePoint == null)
        {
            Debug.LogError(
                $"[EMP] '{name}': muzzlePoint is not assigned. EMP disabled.",
                this
            );
            enabled = false;
            return;
        }
    }

    private void Update()
    {
        if (input.EmpPressed)
            TryActivate();
    }

    // -------------------------------------------------------------------------
    // Public API
    // -------------------------------------------------------------------------

    /// <summary>
    /// Attempts to fire an EMP shot. Returns true on success.
    ///
    /// Returns false (no state change, no debit, no projectile) when:
    ///   EMP is disabled in tuning.
    ///   Energy is EMP-frozen.
    ///   Energy reserves are below empEnergyCost.
    ///
    /// On success: pays energy, performs a soft-homing cone scan, instantiates
    /// the projectile at the muzzle, and hands it the target (or null) and
    /// freeze duration. The projectile then flies itself.
    /// </summary>
    public bool TryActivate()
    {
        if (!S.enableEmp) return false;
        if (energy.IsEmpFrozen) return false;

        if (!energy.TryConsume(S.empEnergyCost))
            return false;

        Transform target = TargetingScan.PickBestInCone(
            transform,
            S.empScanRange,
            S.empScanConeAngle,
            targetLayers,
            _scanBuffer);

        var projectile = Instantiate(empProjectilePrefab, muzzlePoint.position, muzzlePoint.rotation);
        projectile.SetTarget(target);
        projectile.SetOwner(transform);
        projectile.SetFreezeDuration(S.empFreezeDuration);

        OnEmpFired?.Invoke();
        return true;
    }

    // -------------------------------------------------------------------------
    // 🎨 Debug Gizmos
    // -------------------------------------------------------------------------
#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (debugSettings != null && !debugSettings.enableDebugGizmos) return;
        if (profile == null) return;

        var s = profile.emp;

        // Draw the acquisition cone in the Scene view.
        Vector3 origin = transform.position;
        Vector3 fwd    = transform.forward;
        float   range  = s.empScanRange;
        float   angle  = s.empScanConeAngle;

        Gizmos.color = new Color(0.4f, 0.9f, 1f, 0.4f);
        Gizmos.DrawLine(origin, origin + fwd * range);

        Quaternion lRot = Quaternion.AngleAxis(-angle, transform.up);
        Quaternion rRot = Quaternion.AngleAxis( angle, transform.up);
        Quaternion uRot = Quaternion.AngleAxis(-angle, transform.right);
        Quaternion dRot = Quaternion.AngleAxis( angle, transform.right);

        Gizmos.DrawLine(origin, origin + lRot * fwd * range);
        Gizmos.DrawLine(origin, origin + rRot * fwd * range);
        Gizmos.DrawLine(origin, origin + uRot * fwd * range);
        Gizmos.DrawLine(origin, origin + dRot * fwd * range);

        if (muzzlePoint != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(muzzlePoint.position, 0.1f);
            Gizmos.DrawLine(muzzlePoint.position, muzzlePoint.position + muzzlePoint.forward * 1.5f);
        }
    }
#endif
}
