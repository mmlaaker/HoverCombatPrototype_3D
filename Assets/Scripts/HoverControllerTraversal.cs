using UnityEngine;

/// <summary>
/// HoverController_Traversal v2.1 (Designer / Tech-Art Edition)
/// ------------------------------------------------------------
/// Handles terrain sampling and slope blending for the hover system.
/// Feeds averaged ground height and normals to HoverController_Foundation.
/// 
/// • Clean Designer/Tech-Art layout
/// • Slope and height smoothing for stability
/// • Auto-detects hover points in prefab hierarchy
/// • Visual debug lines for sampling and averaged normal
/// </summary>
[DisallowMultipleComponent]
public class HoverController_Traversal : MonoBehaviour
{
    // ------------------------------------------------------------
    // 🌍 Ground Sampling
    // ------------------------------------------------------------
    [Header("🌍 Ground Sampling")]
    [Tooltip("Hover points used to sample terrain. If left empty, automatically detects child objects named 'HoverPoint'.")]
    public Transform[] hoverPoints;

    [Tooltip("Maximum ray length cast downward from each hover point. Should be slightly greater than Foundation.SensorRange.")]
    [Min(1f)] public float rayLength = 6f;

    [Tooltip("Physics layers considered valid ground surfaces.")]
    public LayerMask groundLayers = ~0;

    // ------------------------------------------------------------
    // 🧩 Surface Smoothing
    // ------------------------------------------------------------
    [Header("🧩 Surface Smoothing")]
    [Tooltip("How quickly averaged ground normals adapt to changing slopes. Lower = smoother, higher = more responsive.")]
    [Range(1f, 30f)] public float normalSmoothSpeed = 8f;

    [Tooltip("How quickly averaged ground height adapts to changes in terrain height.")]
    [Range(1f, 30f)] public float heightSmoothSpeed = 12f;

    // ------------------------------------------------------------
    // 🧭 Debug Visualization
    // ------------------------------------------------------------
    [Header("🧭 Debug Visualization")]
    [Tooltip("Draws sampling rays and averaged ground normal in Scene view.")]
    public bool drawDebugRays = false;

    [Tooltip("Color for hit rays.")]
    public Color hitRayColor = Color.green;
    [Tooltip("Color for miss rays.")]
    public Color missRayColor = Color.red;
    [Tooltip("Color for averaged normal vector.")]
    public Color normalColor = Color.cyan;

    // ------------------------------------------------------------
    // Runtime Output (read-only to other systems)
    // ------------------------------------------------------------
    public bool IsGrounded { get; private set; }
    public Vector3 GroundNormal { get; private set; } = Vector3.up;
    public float AverageGroundHeight { get; private set; }

    // Private state
    private Vector3 targetNormal = Vector3.up;
    private float targetHeight;
    private Rigidbody rb;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();

        // Auto-detect hover points if none assigned
        if (hoverPoints == null || hoverPoints.Length == 0)
        {
            var points = new System.Collections.Generic.List<Transform>();
            foreach (Transform child in GetComponentsInChildren<Transform>())
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
            // fallback single ray from transform
            if (Physics.Raycast(transform.position, Vector3.down, out RaycastHit hit, rayLength, groundLayers))
            {
                targetNormal = hit.normal;
                targetHeight = hit.distance;
                IsGrounded = true;

                if (drawDebugRays)
                    Debug.DrawLine(transform.position, hit.point, hitRayColor);
            }
            else
            {
                targetNormal = Vector3.up;
                targetHeight = rayLength;
                IsGrounded = false;

                if (drawDebugRays)
                    Debug.DrawLine(transform.position, transform.position + Vector3.down * rayLength, missRayColor);
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
                    Debug.DrawLine(p.position, hit.point, hitRayColor);
            }
            else if (drawDebugRays)
            {
                Debug.DrawLine(p.position, p.position + Vector3.down * rayLength, missRayColor);
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
            Debug.DrawRay(transform.position, GroundNormal * 2f, normalColor);
    }
}
