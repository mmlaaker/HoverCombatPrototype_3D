using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class HoverController_Foundation : MonoBehaviour
{
    // ------------------------------------------------------------
    // 🧩 Hover Points
    // ------------------------------------------------------------
    [Header("🧩 Hover Points")]
    [Tooltip("Only these transforms are used as hover ray origins (e.g., HoverPoint_FL/FR/RL/RR).")]
    [SerializeField] private Transform[] hoverPoints;

    [Tooltip("Auto-fills Hover Points from children whose name starts with this prefix.")]
    [SerializeField] private string hoverPointNamePrefix = "HoverPoint_";

    // ------------------------------------------------------------
    // 🚀 Hover Lift Settings
    // ------------------------------------------------------------
    [Header("🚀 Hover Lift Settings")]
    [Tooltip("Target hover height above the ground (in meters).")]
    [SerializeField] private float hoverHeight = 3f;

    [Tooltip("Spring strength controlling how strongly the vehicle maintains hover height.")]
    [SerializeField] private float liftStrength = 50000f;

    [Tooltip("Damping applied to hover spring movement. Higher = less bounce.")]
    [SerializeField] private float liftDamping = 5000f;

    [Tooltip("Maximum raycast distance for ground detection.")]
    [SerializeField] private float sensorRange = 5f;

    [Tooltip("Controls how strongly the craft aligns to the average ground normal (0–1).")]
    [Range(0f, 1f)]
    [SerializeField] private float selfLevelStrength = 0.5f;

    // ------------------------------------------------------------
    // 🧮 Slope Lift Compensation
    // ------------------------------------------------------------
    [Header("🧮 Slope Lift Compensation")]
    [Tooltip("If enabled, increases lift on steep slopes to prevent power loss.")]
    [SerializeField] private bool enableSlopeLiftCompensation = true;

    [Tooltip("Multiplier for lift boost on steep slopes (1.0 = none, 1.3 = subtle, 1.6 = strong).")]
    [Range(1f, 2f)]
    [SerializeField] private float slopeLiftMultiplier = 1.3f;

    // ------------------------------------------------------------
    // ⚙️ Angular Damping
    // ------------------------------------------------------------
    [Header("⚙️ Angular Damping")]
    [Tooltip("Damps pitch and roll oscillation without affecting yaw turning.")]
    [Range(0f, 10f)]
    [SerializeField] private float angularDampingMultiplier = 5f;

    // ------------------------------------------------------------
    // 🌍 Ground Interaction
    // ------------------------------------------------------------
    [Header("🌍 Ground Interaction")]
    [Tooltip("Layers considered as ground for hover detection.")]
    [SerializeField] private LayerMask groundLayers = ~0;

    [Tooltip("Draws debug rays in the scene view.")]
    [SerializeField] private bool drawDebugRays = true;

    // ------------------------------------------------------------
    // Runtime
    // ------------------------------------------------------------
    private Rigidbody rb;

    public bool IsHoverGrounded { get; private set; }

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();

        if (hoverPoints == null || hoverPoints.Length == 0)
        {
            Debug.LogWarning(
                $"[Foundation] No hoverPoints assigned on '{name}'. " +
                $"Assign HoverPoint transforms (e.g., HoverPoint_FL/FR/RL/RR). Hover will be disabled."
            );
        }
    }

    private void FixedUpdate()
    {
        ApplyHoverForces();
        ApplyTorqueDamping();
    }

#if UNITY_EDITOR
    private void OnValidate()
    {
        // Auto-fill hover points if missing or contains nulls.
        bool needsFill = hoverPoints == null || hoverPoints.Length == 0;
        if (!needsFill)
        {
            for (int i = 0; i < hoverPoints.Length; i++)
            {
                if (hoverPoints[i] == null) { needsFill = true; break; }
            }
        }

        if (!needsFill) return;

        Transform[] all = GetComponentsInChildren<Transform>(true);
        List<Transform> found = new List<Transform>();

        foreach (Transform t in all)
        {
            if (t == transform) continue;

            if (!string.IsNullOrEmpty(hoverPointNamePrefix) &&
                t.name.StartsWith(hoverPointNamePrefix))
            {
                found.Add(t);
            }
        }

        hoverPoints = found.ToArray();
    }
#endif

    // ------------------------------------------------------------
    // 🧠 Core Hover Logic
    // ------------------------------------------------------------
    private void ApplyHoverForces()
    {
        IsHoverGrounded = false;

        if (hoverPoints == null || hoverPoints.Length == 0)
            return;

        Vector3 normalSum = Vector3.zero;

        foreach (Transform point in hoverPoints)
        {
            if (point == null) continue;

            Vector3 rayDir = -point.up;

            if (Physics.Raycast(point.position, rayDir, out RaycastHit hit, sensorRange, groundLayers))
            {
                IsHoverGrounded = true;
                normalSum += hit.normal;

                float compression = hoverHeight - hit.distance;
                float velocityAlongNormal = Vector3.Dot(rb.GetPointVelocity(point.position), hit.normal);

                float springForce = (compression * liftStrength) - (velocityAlongNormal * liftDamping);

                if (enableSlopeLiftCompensation)
                {
                    float slopeFactor = Mathf.Clamp01(1f - hit.normal.y);
                    springForce *= Mathf.Lerp(1f, slopeLiftMultiplier, slopeFactor);
                }

                springForce = Mathf.Max(springForce, 0f);
                rb.AddForceAtPosition(hit.normal * springForce, point.position, ForceMode.Force);

                if (drawDebugRays)
                {
                    Debug.DrawRay(point.position, rayDir * hit.distance, Color.green);
                }
            }
            else
            {
                if (drawDebugRays)
                {
                    Debug.DrawRay(point.position, rayDir * sensorRange, Color.red);
                }
            }
        }

        if (IsHoverGrounded && normalSum != Vector3.zero)
        {
            ApplyLeveling(normalSum.normalized);
        }
    }

    private void ApplyLeveling(Vector3 groundNormal)
    {
        Quaternion currentRot = transform.rotation;
        Quaternion targetRot = Quaternion.FromToRotation(transform.up, groundNormal) * currentRot;

        // Intentionally uses MoveRotation (stabilizing / arcade-friendly leveling).
        rb.MoveRotation(Quaternion.Slerp(currentRot, targetRot, selfLevelStrength));
    }

    // ------------------------------------------------------------
    // ⚙️ Pitch/Roll Damping
    // ------------------------------------------------------------
    private void ApplyTorqueDamping()
    {
        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);

        // Exponential-ish damping (stable across framerates).
        float dt = Time.fixedDeltaTime;
        float damp = 1f / (1f + angularDampingMultiplier * dt);

        localAngVel.x *= damp;
        localAngVel.z *= damp;

        rb.angularVelocity = transform.TransformDirection(localAngVel);
    }
}
