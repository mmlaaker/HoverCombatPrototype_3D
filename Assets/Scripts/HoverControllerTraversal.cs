using UnityEngine;

/// <summary>
/// HoverController_Traversal v2.0 (Designer / Tech-Art Edition)
/// ------------------------------------------------------------
/// Responsible for detecting ground under hover points,
/// calculating averaged ground normals, and providing
/// stable slope data to the Foundation and Propulsion systems.
///
/// • Adds normal smoothing to reduce ramp wobble
/// • Groups inspector fields with clear tooltips
/// • Compatible with Foundation v3.1 and Propulsion v5.4
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class HoverController_Traversal : MonoBehaviour
{
    // ------------------------------------------------------------
    // 🌍 Ground Sampling
    // ------------------------------------------------------------
    [Header("🌍 Ground Sampling")]
    [Tooltip("Hover point transforms used to sample the ground surface. If empty, will auto-detect children named 'HoverPoint'.")]
    public Transform[] hoverPoints;

    [Tooltip("Maximum raycast distance below each hover point (should be ≥ Foundation.SensorRange).")]
    public float rayLength = 6f;

    [Tooltip("Physics layers considered valid ground surfaces.")]
    public LayerMask groundLayers = -1;

    // ------------------------------------------------------------
    // 🔧 Surface Smoothing
    // ------------------------------------------------------------
    [Header("🔧 Surface Smoothing")]
    [Tooltip("How quickly the averaged ground normal blends toward new terrain angles. Higher = more responsive, lower = smoother.")]
    [Range(1f, 30f)] public float normalSmoothSpeed = 10f;

    [Tooltip("How strongly ground height changes are blended over time. Helps stabilize hover height on uneven terrain.")]
    [Range(1f, 30f)] public float heightSmoothSpeed = 10f;

    // ------------------------------------------------------------
    // 🧭 Debug Visualization
    // ------------------------------------------------------------
    [Header("🧭 Debug Visualization")]
    [Tooltip("Draws the ground-sampling rays and averaged normal in the Scene view.")]
    public bool drawDebugRays = false;

    // ------------------------------------------------------------
    // Runtime Data (read-only to others)
    // ------------------------------------------------------------
    public bool IsGrounded { get; private set; }
    public Vector3 GroundNormal { get; private set; } = Vector3.up;
    public float AverageGroundHeight { get; private set; }

    // Private state
    private Rigidbody rb;
    private Vector3 targetNormal = Vector3.up;
    private float targetHeight = 0f;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        // Auto-detect hover points if none assigned
        if (hoverPoints == null || hoverPoints.Length == 0)
        {
            var points = new System.Collections.Generic.List<Transform>();
            foreach (Transform child in transform)
            {
                if (child.name.ToLower().Contains("hoverpoint"))
                    points.Add(child);
            }
            hoverPoints = points.ToArray();
            if (hoverPoints.Length == 0)
                Debug.LogWarning("[Traversal] No hover points assigned or found. Using transform position as fallback.");
        }
    }

    void FixedUpdate()
    {
        SampleGround();
        SmoothResults();
    }

    // ------------------------------------------------------------
    // Ground Sampling
    // ------------------------------------------------------------
    private void SampleGround()
    {
        if (hoverPoints == null || hoverPoints.Length == 0)
        {
            // fallback single ray
            if (Physics.Raycast(transform.position, Vector3.down, out RaycastHit hit, rayLength, groundLayers))
            {
                targetNormal = hit.normal;
                targetHeight = hit.distance;
                IsGrounded = true;
            }
            else
            {
                targetNormal = Vector3.up;
                targetHeight = rayLength;
                IsGrounded = false;
            }
            return;
        }

        Vector3 normalSum = Vector3.zero;
        float heightSum = 0f;
        int hitCount = 0;

        foreach (Transform p in hoverPoints)
        {
            if (Physics.Raycast(p.position, Vector3.down, out RaycastHit hit, rayLength, groundLayers))
            {
                normalSum += hit.normal;
                heightSum += hit.distance;
                hitCount++;

                if (drawDebugRays)
                    Debug.DrawLine(p.position, hit.point, Color.green);
            }
            else if (drawDebugRays)
            {
                Debug.DrawLine(p.position, p.position + Vector3.down * rayLength, Color.red);
            }
        }

        if (hitCount > 0)
        {
            targetNormal = (normalSum / hitCount).normalized;
            targetHeight = heightSum / hitCount;
            IsGrounded = true;
        }
        else
        {
            targetNormal = Vector3.up;
            targetHeight = rayLength;
            IsGrounded = false;
        }
    }

    // ------------------------------------------------------------
    // Temporal Smoothing
    // ------------------------------------------------------------
    private void SmoothResults()
    {
        GroundNormal = Vector3.Slerp(GroundNormal, targetNormal, Time.fixedDeltaTime * normalSmoothSpeed);
        AverageGroundHeight = Mathf.Lerp(AverageGroundHeight, targetHeight, Time.fixedDeltaTime * heightSmoothSpeed);

        if (drawDebugRays)
        {
            Vector3 origin = transform.position;
            Debug.DrawRay(origin, GroundNormal * 2f, Color.cyan);
        }
    }
}
