using UnityEngine;

/// <summary>
/// HoverController_Traversal v1.0
/// --------------------------------
/// Bridges hover physics (Foundation) and motion (Propulsion).
/// - Smooths averaged ground normals over time
/// - Maintains grounded continuity with hysteresis
/// - Optionally predicts upcoming slopes with a forward anticipatory ray
/// - Exposes clean traversal data for propulsion and AI
/// </summary>
[RequireComponent(typeof(HoverController_Foundation))]
public class HoverController_Traversal : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Source Foundation providing hover state.")]
    public HoverController_Foundation foundation;

    [Header("Normal Smoothing")]
    [Tooltip("Blend speed for ground normal smoothing.")]
    [Range(1f, 20f)] public float normalBlendSpeed = 8f;

    [Tooltip("If true, start from current up vector on first contact to avoid snap.")]
    public bool initializeFromUp = true;

    [Header("Grounded Continuity")]
    [Tooltip("Seconds to remain 'grounded' after last contact loss.")]
    [Range(0f, 1f)] public float groundedHysteresis = 0.25f;

    [Header("Slope Anticipation")]
    [Tooltip("If >0, samples a forward point at this multiple of hoverHeight for predictive normal blending.")]
    [Range(0f, 2f)] public float forwardSampleDistance = 0.75f;

    [Tooltip("Weight applied to anticipatory normal blending (0–1).")]
    [Range(0f, 1f)] public float forwardNormalWeight = 0.5f;

    [Header("Debug")]
    public bool drawDebug = true;

    // --- Runtime state ---
    private Vector3 smoothedNormal = Vector3.up;
    private bool wasGrounded;
    private float lastGroundTime;
    private Rigidbody rb;

    // --- Public read-only interface ---
    public Vector3 GroundNormal => smoothedNormal;
    public bool IsGrounded => wasGrounded;

    void Awake()
    {
        if (!foundation) foundation = GetComponent<HoverController_Foundation>();
        rb = GetComponent<Rigidbody>();
        smoothedNormal = transform.up;
    }

    void FixedUpdate()
    {
        var fState = foundation.GetState();

        // Predictive slope sampling (optional)
        Vector3 predictedNormal = fState.avgNormal;
        if (forwardSampleDistance > 0f && fState.anyContact)
        {
            Vector3 origin = rb.worldCenterOfMass + transform.forward * (foundation.hoverHeight * forwardSampleDistance);
            if (Physics.Raycast(origin, -transform.up, out RaycastHit hit,
                foundation.rayLength, foundation.groundMask, QueryTriggerInteraction.Ignore))
            {
                predictedNormal = Vector3.Slerp(predictedNormal, hit.normal, forwardNormalWeight);
            }
        }

        // Normal smoothing
        if (fState.anyContact)
        {
            if (initializeFromUp && smoothedNormal == Vector3.up)
                smoothedNormal = predictedNormal;

            smoothedNormal = Vector3.Slerp(smoothedNormal, predictedNormal, Time.fixedDeltaTime * normalBlendSpeed);
        }

        // Grounded continuity
        if (fState.anyContact)
            lastGroundTime = Time.time;

        wasGrounded = fState.anyContact || (Time.time - lastGroundTime < groundedHysteresis);
    }

#if UNITY_EDITOR
    void OnDrawGizmosSelected()
    {
        if (!drawDebug) return;
        if (!Application.isPlaying) return;

        Gizmos.color = wasGrounded ? Color.green : Color.red;
        Vector3 origin = rb ? rb.worldCenterOfMass : transform.position;
        Gizmos.DrawLine(origin, origin + smoothedNormal * foundation.hoverHeight);
        Gizmos.DrawWireSphere(origin + smoothedNormal * foundation.hoverHeight, 0.1f);
    }
#endif
}
