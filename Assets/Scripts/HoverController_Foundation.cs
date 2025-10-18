using UnityEngine;

/// <summary>
/// HoverController Foundation v2.6 (restored robust unstick behavior)
/// - Mass-normalized hover physics
/// - Self-leveling torque
/// - Independent unstick controls for bias, recovery, and penetration
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class HoverController_Foundation : MonoBehaviour
{
    [Header("Hover Settings")]
    public float hoverHeight = 2.0f;
    public float springStrength = 25000f;
    public float damping = 500f;
    public float contactRadius = 0.25f;
    public float rayLength = 5f;
    public LayerMask groundMask = Physics.DefaultRaycastLayers;

    [Header("Leveling Torque")]
    public bool enableLeveling = true;
    [Range(0f, 1f)] public float torqueFactor = 0.3f;

    [Header("Hover Points")]
    public Transform[] hoverPoints;

    [Header("Recovery / Unstick")]
    [Tooltip("Maximum positional step used to break welded contacts (m).")]
    public float maxUnstickStep = 0.05f;

    [Tooltip("Guaranteed upward bias each frame when grounded (m).")]
    public float minLiftBias = 0.3f;   // ⬅ stronger, proven to work

    [Tooltip("Upward force range in g for recovery lift.")]
    public Vector2 recoveryGRange = new Vector2(0.35f, 3f);

    [Tooltip("Velocity along normal below which recovery activates (m/s).")]
    public float recoverNormalSpeed = 3f;

    [Tooltip("Maximum upward speed after recovery (m/s).")]
    public float maxSeparationSpeed = 5f;

    // --- state ---
    private Rigidbody rb;
    private Collider[] myColliders;
    private readonly Collider[] overlapBuffer = new Collider[16];

    private bool isCollidedGrounded;
    private Vector3 collidedAvgNormal = Vector3.up;
    private int collidedCount = 0;

    public struct HoverPhysicsState
    {
        public Vector3 avgNormal;
        public Vector3 avgPoint;
        public bool anyContact;
        public float avgCompression;
        public Vector3 velocity;

        // Legacy aliases
        public Vector3 averagePoint => avgPoint;
        public Vector3 averageNormal => avgNormal;
    }
    private HoverPhysicsState state;
    public HoverPhysicsState GetState() => state;

    public bool IsHoverGrounded => state.anyContact;
    public bool IsCollidedGrounded => isCollidedGrounded;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        myColliders = GetComponentsInChildren<Collider>();
        if (myColliders.Length == 0)
            Debug.LogWarning("[HoverFoundation] No colliders found on craft.");
    }

    void FixedUpdate()
    {
        PerformHover();
        PerformRecovery();
    }

    void PerformHover()
    {
        if (hoverPoints == null || hoverPoints.Length == 0) return;

        Vector3 totalForce = Vector3.zero;
        Vector3 weightedNormal = Vector3.zero;
        float totalCompression = 0f, totalWeight = 0f;
        bool anyContact = false;

        foreach (var p in hoverPoints)
        {
            if (!p) continue;

            if (Physics.SphereCast(p.position, contactRadius, -transform.up,
                out RaycastHit hit, rayLength, groundMask, QueryTriggerInteraction.Ignore))
            {
                anyContact = true;
                float distance = hit.distance;
                float compression = Mathf.Clamp01((hoverHeight - distance) / hoverHeight);
                float spring = springStrength * compression;
                float vel = Vector3.Dot(rb.GetPointVelocity(p.position), transform.up);
                float damp = damping * -vel;

                totalForce += transform.up * (spring + damp);
                weightedNormal += hit.normal * compression;
                totalCompression += compression;
                totalWeight += compression;
            }
        }

        if (anyContact)
        {
            rb.AddForce(totalForce / hoverPoints.Length, ForceMode.Force);

            if (enableLeveling)
            {
                Vector3 avgNormal = (totalWeight > 0f)
                    ? (weightedNormal / totalWeight).normalized
                    : transform.up;
                Vector3 axis = Vector3.Cross(transform.up, avgNormal);
                if (axis.sqrMagnitude > 1e-6f)
                {
                    Vector3 tq = axis.normalized * (rb.mass * Physics.gravity.magnitude * torqueFactor);
                    rb.AddTorque(tq, ForceMode.Force);
                }
                state.avgNormal = avgNormal;
            }

            state.anyContact = true;
            state.avgCompression = totalCompression / hoverPoints.Length;
        }
        else state.anyContact = false;

        state.velocity = rb.linearVelocity;
    }

    void PerformRecovery()
    {
        // --- Bias lift (core unstick) ---
        if (isCollidedGrounded && !IsHoverGrounded)
            rb.position += Vector3.up * minLiftBias;

        // --- Force-based recovery ---
        if (!IsHoverGrounded && isCollidedGrounded)
        {
            Vector3 n = (collidedCount > 0 && collidedAvgNormal.sqrMagnitude > 1e-6f)
                        ? (collidedAvgNormal / collidedCount).normalized
                        : transform.up;

            float vN = Vector3.Dot(rb.linearVelocity, n);
            bool nearlyStill = Mathf.Abs(vN) < recoverNormalSpeed;
            bool notRising = vN <= 0f;

            if (nearlyStill && notRising)
            {
                if (Physics.Raycast(rb.worldCenterOfMass, -n, out RaycastHit downHit,
                    hoverHeight * 1.5f, groundMask))
                {
                    float depth = Mathf.Clamp01((hoverHeight - downHit.distance) / hoverHeight);
                    float gScale = Mathf.Lerp(recoveryGRange.x, recoveryGRange.y, depth);
                    Vector3 lift = n * (rb.mass * Physics.gravity.magnitude * gScale);
                    rb.AddForce(lift, ForceMode.Force);
                }

                vN = Vector3.Dot(rb.linearVelocity, n);
                if (vN > maxSeparationSpeed)
                {
                    Vector3 vt = rb.linearVelocity - vN * n;
                    rb.linearVelocity = vt + n * maxSeparationSpeed;
                }
            }
        }

        // --- ComputePenetration micro-step (rare) ---
        if (!IsHoverGrounded && isCollidedGrounded && maxUnstickStep > 0f)
        {
            Vector3 sumDir = Vector3.zero;
            float sumDist = 0f;
            int hitCount = Physics.OverlapSphereNonAlloc(
                rb.worldCenterOfMass, rayLength, overlapBuffer, groundMask, QueryTriggerInteraction.Ignore);

            for (int i = 0; i < hitCount; i++)
            {
                var gcol = overlapBuffer[i];
                if (!gcol || gcol.attachedRigidbody == rb) continue;

                foreach (var mine in myColliders)
                {
                    if (!mine || !mine.enabled) continue;
                    Vector3 dir; float dist;
                    if (Physics.ComputePenetration(
                        mine, mine.transform.position, mine.transform.rotation,
                        gcol, gcol.transform.position, gcol.transform.rotation,
                        out dir, out dist))
                    {
                        if (dist > 0f)
                        {
                            sumDir += dir;
                            sumDist += dist;
                        }
                    }
                }
            }

            if (sumDist > 0f)
            {
                Vector3 dir = sumDir.sqrMagnitude > 1e-6f ? sumDir.normalized : transform.up;
                float step = Mathf.Clamp(sumDist * 0.6f, 0.01f, maxUnstickStep);
                rb.MovePosition(rb.position + dir * step);
                rb.WakeUp();
            }
        }

        // --- Reset cache ---
        isCollidedGrounded = false;
        collidedAvgNormal = Vector3.zero;
        collidedCount = 0;
    }

    void OnCollisionStay(Collision c)
    {
        if (((1 << c.gameObject.layer) & groundMask) == 0) return;
        isCollidedGrounded = true;
        for (int i = 0; i < c.contactCount; i++)
        {
            collidedAvgNormal += c.GetContact(i).normal;
            collidedCount++;
        }
    }

    void OnCollisionExit(Collision c)
    {
        if (((1 << c.gameObject.layer) & groundMask) == 0) return;
    }

#if UNITY_EDITOR
    void OnDrawGizmosSelected()
    {
        if (hoverPoints == null) return;
        Gizmos.color = Color.cyan;
        foreach (var p in hoverPoints)
        {
            if (!p) continue;
            Gizmos.DrawWireSphere(p.position, contactRadius);
            Gizmos.DrawLine(p.position, p.position - transform.up * rayLength);
        }
    }
#endif
}
