using UnityEngine;

/// <summary>
/// Physics-based hovercraft controller with:
/// - Stable hover and self-leveling
/// - Instant spawn-height alignment (no bounce)
/// - Physically grounded thrust & turning (mass-scaled)
/// - Dynamic drift damping + velocity alignment assist
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class HoverController : MonoBehaviour
{
    [Header("Hover Settings")]
    public Transform[] hoverPoints;
    public float hoverHeight = 3f;
    public float hoverForce = 300f;
    public float hoverDamp = 50f;

    [Header("Movement Settings")]
    public float thrustForce = 150f;
    public float turnTorque = 35f;
    public float maxSpeed = 100f;

    [Header("Thrust Refinement")]
    public float driftDamp = 1.5f;
    public float verticalDamp = 5.0f;

    [Header("Turn Assist Settings")]
    [Tooltip("Multiplier for drift damping while turning.")]
    public float driftBoostFactor = 3f;
    [Tooltip("How quickly velocity re-aligns with facing direction (2–4 is typical).")]
    public float alignStrength = 2.5f;

    [Header("Drag Settings")]
    public float linearDrag = 0.1f;
    public float angularDrag = 0.2f;

    [Header("Center of Mass")]
    public Transform centerOfMassTransform;

    [Header("Leveling Settings")]
    public float levelingStrength = 130f;
    [Range(0f, 1f)] public float angularDampFactor = 0.8f;

    private Rigidbody rb;
    private float[] lastLiftForces;
    private bool initialized;

    // ------------------------------------------------------------------------

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = true;
        rb.linearDamping = linearDrag;
        rb.angularDamping = angularDrag;

        if (!centerOfMassTransform)
        {
            centerOfMassTransform = transform.Find("CenterOfMass");
            if (!centerOfMassTransform)
            {
                Debug.LogError($"{name}: Missing CenterOfMass child transform!");
                enabled = false;
                return;
            }
        }

        rb.centerOfMass = transform.InverseTransformPoint(centerOfMassTransform.position);
        AdjustSpawnHeightInstantly();

        if (hoverPoints != null)
            lastLiftForces = new float[hoverPoints.Length];

        initialized = true;
    }

    private void FixedUpdate()
    {
        if (!initialized) return;

        ApplyHoverForces();
        ApplyMovementForces();
        ApplyLevelingTorque();
    }

    // ------------------------------------------------------------------------
    // INSTANT SPAWN HEIGHT ALIGNMENT
    // ------------------------------------------------------------------------

    private void AdjustSpawnHeightInstantly()
    {
        float totalGroundHeight = 0f;
        int hitCount = 0;

        foreach (var point in hoverPoints)
        {
            if (point && Physics.Raycast(point.position + Vector3.up * 10f, Vector3.down, out RaycastHit hit, 100f))
            {
                totalGroundHeight += hit.point.y;
                hitCount++;
            }
        }

        if (hitCount > 0)
        {
            float avgGroundY = totalGroundHeight / hitCount;
            float targetY = avgGroundY + hoverHeight;
            Vector3 pos = transform.position;
            pos.y = targetY;
            transform.position = pos;
        }
    }

    // ------------------------------------------------------------------------
    // HOVER LIFT
    // ------------------------------------------------------------------------

    private void ApplyHoverForces()
    {
        if (hoverPoints == null || hoverPoints.Length == 0) return;

        for (int i = 0; i < hoverPoints.Length; i++)
        {
            var point = hoverPoints[i];
            if (!point) continue;

            if (Physics.Raycast(point.position, -Vector3.up, out RaycastHit hit, hoverHeight * 3f))
            {
                float heightError = hoverHeight - hit.distance;
                float upwardSpeed = rb.GetPointVelocity(point.position).y;
                float lift = Mathf.Clamp((heightError * hoverForce) - (upwardSpeed * hoverDamp),
                                         -hoverForce, hoverForce);

                rb.AddForceAtPosition(Vector3.up * lift, point.position, ForceMode.Force);
                lastLiftForces[i] = lift;
            }
            else
            {
                lastLiftForces[i] = 0f;
            }
        }
    }

    // ------------------------------------------------------------------------
    // MOVEMENT (MASS-SCALED + HYBRID TURN ASSIST)
    // ------------------------------------------------------------------------

    private void ApplyMovementForces()
    {
        float forwardInput = 0f;
        float turnInput = 0f;

#if ENABLE_INPUT_SYSTEM
        if (UnityEngine.InputSystem.Keyboard.current != null)
        {
            var kb = UnityEngine.InputSystem.Keyboard.current;
            if (kb.wKey.isPressed) forwardInput += 1f;
            if (kb.sKey.isPressed) forwardInput -= 1f;
            if (kb.aKey.isPressed) turnInput -= 1f;
            if (kb.dKey.isPressed) turnInput += 1f;
        }
#else
        forwardInput = Input.GetAxis("Vertical");
        turnInput = Input.GetAxis("Horizontal");
#endif

        // --- Determine hover-plane direction ---
        Vector3 planeNormal = GetAverageGroundNormal();
        Vector3 thrustDir = Vector3.ProjectOnPlane(transform.forward, planeNormal).normalized;

        // --- Apply thrust scaled by mass ---
        Vector3 currentVel = rb.linearVelocity;
        float speed = Vector3.Dot(currentVel, thrustDir);

        if (Mathf.Abs(forwardInput) > 0.01f && speed < maxSpeed)
            rb.AddForce(thrustDir * forwardInput * thrustForce * rb.mass, ForceMode.Force);

        // --- Apply yaw torque scaled by mass ---
        Vector3 yawAxis = transform.up;
        if (Mathf.Abs(turnInput) > 0.01f)
            rb.AddTorque(yawAxis * turnInput * turnTorque * rb.mass * 0.5f, ForceMode.Force);
        else
        {
            // Light yaw damping when no input
            Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);
            localAngVel.y *= 0.9f;
            rb.angularVelocity = transform.TransformDirection(localAngVel);
        }

        // --- Drift damping (reduce sideways slide) ---
        Vector3 forwardVel = Vector3.Project(currentVel, transform.forward);
        Vector3 lateralVel = currentVel - forwardVel;

        // Boost damping when steering for sharper turn response
        float dynamicDrift = driftDamp * (Mathf.Abs(turnInput) > 0.01f ? driftBoostFactor : 1f);
        rb.AddForce(-lateralVel * dynamicDrift, ForceMode.Acceleration);

        // --- Vertical damping (stay near plane) ---
        Vector3 verticalVel = Vector3.Project(currentVel, planeNormal);
        rb.AddForce(-verticalVel * verticalDamp, ForceMode.Acceleration);

        // --- Velocity alignment assist (rotates velocity toward facing dir) ---
        if (rb.linearVelocity.sqrMagnitude > 1f)
        {
            Vector3 desiredVel = transform.forward * Vector3.Dot(rb.linearVelocity, transform.forward);
            rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, desiredVel, Time.fixedDeltaTime * alignStrength);
        }
    }

    private Vector3 GetAverageGroundNormal()
    {
        if (hoverPoints == null || hoverPoints.Length == 0)
            return Vector3.up;

        Vector3 avg = Vector3.zero;
        int hits = 0;

        foreach (var point in hoverPoints)
        {
            if (point && Physics.Raycast(point.position, -Vector3.up, out RaycastHit hit, hoverHeight * 3f))
            {
                avg += hit.normal;
                hits++;
            }
        }

        return hits > 0 ? (avg / hits).normalized : Vector3.up;
    }

    // ------------------------------------------------------------------------
    // LEVELING / STABILIZATION
    // ------------------------------------------------------------------------

    private void ApplyLevelingTorque()
    {
        Vector3 localUp = transform.up;
        Vector3 desiredUp = Vector3.up;

        Vector3 correctionAxis = Vector3.Cross(localUp, desiredUp);
        float tiltAngle = Vector3.Angle(localUp, desiredUp);

        if (tiltAngle > 0.01f)
            rb.AddTorque(correctionAxis * (tiltAngle * levelingStrength), ForceMode.Acceleration);

        // Dampen pitch and roll
        Vector3 localAngular = transform.InverseTransformDirection(rb.angularVelocity);
        localAngular.x *= angularDampFactor;
        localAngular.z *= angularDampFactor;
        rb.angularVelocity = transform.TransformDirection(localAngular);
    }

    // ------------------------------------------------------------------------
    // GIZMOS
    // ------------------------------------------------------------------------

    private void OnDrawGizmos()
    {
        if (!centerOfMassTransform || hoverPoints == null) return;

        Vector3 comWorld = centerOfMassTransform.position;
        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(comWorld, 0.15f);
        Gizmos.DrawWireSphere(comWorld, 0.3f);

        DrawHoverGizmos(comWorld);
    }

    private void DrawHoverGizmos(Vector3 centerOfMass)
    {
        if (hoverPoints == null) return;

        for (int i = 0; i < hoverPoints.Length; i++)
        {
            var p = hoverPoints[i];
            if (!p) continue;

            bool hit = Physics.Raycast(p.position, -Vector3.up, out RaycastHit hitInfo, hoverHeight * 3f);
            Gizmos.color = hit ? Color.green : Color.red;
            Gizmos.DrawLine(p.position, p.position - Vector3.up * hoverHeight);

            Vector3 contact = hit ? hitInfo.point : p.position - Vector3.up * hoverHeight;
            Gizmos.DrawWireSphere(contact, 0.07f);

            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(centerOfMass, p.position);

            if (Application.isPlaying && i < lastLiftForces.Length)
            {
                float lift = lastLiftForces[i];
                if (Mathf.Abs(lift) > 0.01f)
                {
                    Gizmos.color = Color.magenta;
                    Vector3 dir = Vector3.up * Mathf.Sign(lift);
                    float scale = Mathf.Clamp(Mathf.Abs(lift) / hoverForce, 0.2f, 1.0f);
                    Gizmos.DrawLine(p.position, p.position + dir * scale);
                }
            }
        }
    }
}
