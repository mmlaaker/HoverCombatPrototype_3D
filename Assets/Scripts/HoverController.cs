using UnityEngine;
using System.Linq;

/// <summary>
/// Hovercraft Controller (Stable + Realistic Fall v3)
/// - Smooth spring-based hover physics
/// - Distance-based lift cutoff to prevent floating above pits
/// - Natural gravity blending
/// - Visual pitch tilt for uphill/downhill motion
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class HoverController : MonoBehaviour
{
    // ============================================================
    // --- Hover Settings ---
    // ============================================================

    [Header("Hover Settings")]
    [Tooltip("Transforms where hover forces apply. Place these roughly under the vehicle corners.")]
    public Transform[] hoverPoints;

    [Tooltip("Ideal distance from the ground to maintain.")]
    public float hoverHeight = 6f;

    [Tooltip("Upward acceleration toward hoverHeight. Higher = stiffer hover.")]
    public float hoverForce = 5f;

    [Tooltip("Resists vertical oscillation. Higher = smoother hover, slower fall.")]
    public float hoverDamp = 1.0f;

    [Tooltip("Small upward bias applied when near the ground to keep hover stable.")]
    public float preloadBias = 0.1f;


    // ============================================================
    // --- Movement ---
    // ============================================================

    [Header("Movement Settings")]
    [Tooltip("Forward/backward acceleration.")]
    public float thrustAccel = 100f;

    [Tooltip("Turning torque strength.")]
    public float turnTorque = 10f;

    [Tooltip("Maximum forward speed.")]
    public float maxSpeed = 100f;


    // ============================================================
    // --- Stability & Refinement ---
    // ============================================================

    [Header("Stability Settings")]
    [Tooltip("How much the craft resists sliding sideways.")]
    public float driftDamp = 0.6f;

    [Tooltip("Multiplier to drift damping during active turns.")]
    public float driftBoostFactor = 3f;

    [Tooltip("Resists vertical velocity drift (helps steady altitude).")]
    public float verticalDamp = 3f;

    [Tooltip("Strength of velocity realignment toward facing direction.")]
    public float alignStrength = 5f;


    // ============================================================
    // --- Gravity Blending & Airtime ---
    // ============================================================

    [Header("Air Control")]
    [Tooltip("Fraction of hover points that must contact ground to be 'grounded'. Higher = heavier feel.")]
    [Range(0f, 1f)] public float minGroundedRatio = 0.65f;

    [Tooltip("Speed at which hover/gravity blend transitions. Higher = snappier reaction when losing contact.")]
    public float groundedBlendSpeed = 8f;

    [Tooltip("Reduces angular drag when fully airborne for freer rotation.")]
    public float airAngularDrag = 0.05f;


    // ============================================================
    // --- Drag & Leveling ---
    // ============================================================

    [Header("Drag & Leveling")]
    [Tooltip("Base linear drag (applied constantly).")]
    public float linearDrag = 1f;

    [Tooltip("Base angular drag when grounded.")]
    public float groundAngularDrag = 0.2f;

    [Tooltip("How strongly the craft self-levels to the ground normal.")]
    public float levelingStrength = 10f;

    [Tooltip("How much pitch/roll damping is applied. 1 = no damping.")]
    [Range(0f, 1f)] public float angularDampFactor = 0.8f;


    // ============================================================
    // --- Visual Tilt ---
    // ============================================================

    [Header("Visual Tilt (Pitch Response)")]
    [Tooltip("Multiplier for pitch tilt when ascending or descending.")]
    public float pitchResponse = 0.5f;

    [Tooltip("Maximum visual pitch angle (degrees).")]
    public float maxPitchAngle = 10f;

    [Tooltip("Smoothing speed for pitch reaction.")]
    public float tiltSmooth = 2f;


    // ============================================================
    // --- Center of Mass ---
    // ============================================================

    [Header("Center of Mass")]
    [Tooltip("Optional Transform to override Rigidbody center of mass.")]
    public Transform centerOfMassTransform;


    // ============================================================
    // --- Internal State ---
    // ============================================================

    private Rigidbody rb;
    private Vector3 smoothedGroundNormal = Vector3.up;
    private float groundedBlend = 1f;
    private bool initialized;

    private float visualPitchOffset = 0f; // visual pitch tracking


    // ============================================================
    // --- Unity Lifecycle ---
    // ============================================================

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = false;
        rb.linearDamping = linearDrag;
        rb.angularDamping = groundAngularDrag;

        if (centerOfMassTransform)
            rb.centerOfMass = transform.InverseTransformPoint(centerOfMassTransform.position);

        if (hoverPoints != null)
            hoverPoints = hoverPoints.Where(p => p != null).ToArray();

        AdjustSpawnHeightInstantly();
        initialized = true;
    }

    void FixedUpdate()
    {
        if (!initialized || hoverPoints == null || hoverPoints.Length == 0)
            return;

        ApplyHoverForces();
        ApplyMovementForces();
        ApplyLevelingTorque();
        ApplyVisualTilt();
    }


    // ============================================================
    // --- Hover Physics ---
    // ============================================================

    private void ApplyHoverForces()
    {
        int contactCount = 0;
        float avgDistance = 0f;
        RaycastHit hit;

        foreach (var point in hoverPoints)
        {
            if (Physics.Raycast(point.position, Vector3.down, out hit, hoverHeight * 6f))
            {
                // Ignore faraway hits (prevents floating above valleys)
                if (hit.distance > hoverHeight * 2.5f)
                    continue;

                contactCount++;
                avgDistance += hit.distance;

                float heightError = hoverHeight - hit.distance;
                float normalized = Mathf.Clamp01((heightError + preloadBias) / hoverHeight);
                float upwardSpeed = rb.GetPointVelocity(point.position).y;

                // Clamp damping to avoid violent oscillations
                float damping = Mathf.Clamp(upwardSpeed * hoverDamp, -hoverForce * 1.5f, hoverForce * 1.5f);

                // Distance-based lift falloff (prevents hovering too high)
                float distFactor = Mathf.Clamp01(1f - (hit.distance / (hoverHeight * 2.5f)));

                // Soft spring curve + controlled damping
                float liftAccel = (Mathf.Pow(normalized, 2.2f) * hoverForce * distFactor) - damping;

                rb.AddForceAtPosition(hit.normal * liftAccel * groundedBlend, point.position, ForceMode.Acceleration);
            }
        }

        // --- Compute grounded ratio and unified blend ---
        float contactRatio = (float)contactCount / hoverPoints.Length;
        float rawBlend = Mathf.Clamp01((contactRatio - minGroundedRatio) / (1f - minGroundedRatio));
        float targetBlend = Mathf.Pow(rawBlend, 2f); // faster falloff when losing contact

        // --- If completely airborne: apply gravity directly ---
        if (contactCount == 0)
        {
            groundedBlend = 0f;
            rb.AddForce(Physics.gravity, ForceMode.Acceleration);
            rb.AddForce(Vector3.down * 2f, ForceMode.Acceleration); // faster descent
            rb.angularDamping = airAngularDrag;
            return;
        }

        // --- Smoothly blend between hover and gravity ---
        groundedBlend = SmoothStepTowards(groundedBlend, targetBlend, groundedBlendSpeed);
        rb.AddForce(Physics.gravity * (1f - groundedBlend), ForceMode.Acceleration);

        // Adjust angular drag between air and ground
        rb.angularDamping = Mathf.Lerp(airAngularDrag, groundAngularDrag, groundedBlend);

        // --- Gentle correction for undershoot below hover height ---
        if (contactCount > 0 && groundedBlend > 0.5f)
        {
            float avgError = hoverHeight - (avgDistance / contactCount);
            if (avgError > 0.15f)
                rb.AddForce(Vector3.up * avgError * hoverForce * 0.1f, ForceMode.Acceleration);
        }
    }


    // ============================================================
    // --- Movement Forces ---
    // ============================================================

    private void ApplyMovementForces()
    {
        (float forwardInput, float turnInput) = HandleInput();

        Vector3 planeNormal = GetSmoothedGroundNormal();
        Vector3 thrustDir = Vector3.ProjectOnPlane(transform.forward, planeNormal).normalized;
        Vector3 currentVel = rb.linearVelocity;
        float speed = Vector3.Dot(currentVel, thrustDir);

        // --- Thrust ---
        if (Mathf.Abs(forwardInput) > 0.01f && speed < maxSpeed)
            rb.AddForce(thrustDir * forwardInput * thrustAccel, ForceMode.Acceleration);

        // --- Turning ---
        if (Mathf.Abs(turnInput) > 0.01f)
            rb.AddTorque(transform.up * turnInput * turnTorque, ForceMode.Acceleration);
        else
        {
            Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);
            localAngVel.y *= 0.9f;
            rb.angularVelocity = transform.TransformDirection(localAngVel);
        }

        // --- Lateral and vertical damping ---
        Vector3 projectedVel = Vector3.ProjectOnPlane(currentVel, planeNormal);
        Vector3 lateralVel = projectedVel - Vector3.Project(projectedVel, transform.forward);
        Vector3 verticalVel = currentVel - projectedVel;

        float dynamicDrift = driftDamp * (Mathf.Abs(turnInput) > 0.01f ? driftBoostFactor : 1f);
        Vector3 damping = (-lateralVel * dynamicDrift) - (verticalVel * verticalDamp);
        rb.AddForce(damping, ForceMode.Acceleration);

        // --- Velocity alignment ---
        if (rb.linearVelocity.sqrMagnitude > 1f)
        {
            Vector3 desiredVel = transform.forward * Vector3.Dot(rb.linearVelocity, transform.forward);
            rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, desiredVel, Time.fixedDeltaTime * alignStrength);
        }
    }


    // ============================================================
    // --- Leveling & Pitch Tilt ---
    // ============================================================

    private void ApplyLevelingTorque()
    {
        Vector3 localUp = transform.up;
        Vector3 desiredUp = smoothedGroundNormal;
        Vector3 correctionAxis = Vector3.Cross(localUp, desiredUp);

        rb.AddTorque(correctionAxis * levelingStrength, ForceMode.Acceleration);

        Vector3 localAngular = transform.InverseTransformDirection(rb.angularVelocity);
        localAngular.x *= angularDampFactor;
        localAngular.z *= angularDampFactor;
        rb.angularVelocity = transform.TransformDirection(localAngular);
    }

    private void ApplyVisualTilt()
    {
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
        float targetPitch = Mathf.Clamp(-localVel.y * pitchResponse, -maxPitchAngle, maxPitchAngle);
        visualPitchOffset = Mathf.Lerp(visualPitchOffset, targetPitch, Time.deltaTime * tiltSmooth);

        Quaternion tiltRot = Quaternion.Euler(visualPitchOffset, transform.localEulerAngles.y, transform.localEulerAngles.z);
        transform.localRotation = Quaternion.Slerp(transform.localRotation, tiltRot, Time.deltaTime * tiltSmooth);
    }


    // ============================================================
    // --- Helpers ---
    // ============================================================

    private Vector3 GetSmoothedGroundNormal()
    {
        if (hoverPoints == null || hoverPoints.Length == 0)
            return smoothedGroundNormal;

        Vector3 avg = Vector3.zero;
        int hits = 0;
        RaycastHit hit;

        foreach (var point in hoverPoints)
        {
            if (Physics.Raycast(point.position, Vector3.down, out hit, hoverHeight * 6f))
            {
                avg += hit.normal;
                hits++;
            }
        }

        Vector3 targetNormal = hits > 0 ? (avg / hits).normalized : Vector3.up;
        smoothedGroundNormal = Vector3.Slerp(smoothedGroundNormal, targetNormal, Time.fixedDeltaTime * 8f);
        return smoothedGroundNormal;
    }

    private (float forward, float turn) HandleInput()
    {
        float forward = 0f;
        float turn = 0f;

#if ENABLE_INPUT_SYSTEM
        if (UnityEngine.InputSystem.Keyboard.current != null)
        {
            var kb = UnityEngine.InputSystem.Keyboard.current;
            if (kb.wKey.isPressed) forward += 1f;
            if (kb.sKey.isPressed) forward -= 1f;
            if (kb.aKey.isPressed) turn -= 1f;
            if (kb.dKey.isPressed) turn += 1f;
        }
#else
        forward = Input.GetAxis("Vertical");
        turn = Input.GetAxis("Horizontal");
#endif
        return (forward, turn);
    }

    private void AdjustSpawnHeightInstantly()
    {
        if (hoverPoints == null || hoverPoints.Length == 0) return;

        float totalGroundHeight = 0f;
        int hitCount = 0;
        RaycastHit hit;

        foreach (var point in hoverPoints)
        {
            if (Physics.Raycast(point.position + Vector3.up * 10f, Vector3.down, out hit, 100f))
            {
                totalGroundHeight += hit.point.y;
                hitCount++;
            }
        }

        if (hitCount > 0)
        {
            float avgGroundY = totalGroundHeight / hitCount;
            Vector3 pos = transform.position;
            pos.y = avgGroundY + hoverHeight;
            transform.position = pos;
        }
    }

    private float SmoothStepTowards(float current, float target, float rate)
    {
        return Mathf.Lerp(current, target, 1f - Mathf.Exp(-rate * Time.fixedDeltaTime));
    }


#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (hoverPoints == null) return;

        if (centerOfMassTransform)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(centerOfMassTransform.position, 0.15f);
            Gizmos.DrawWireSphere(centerOfMassTransform.position, 0.3f);
        }

        RaycastHit hit;
        foreach (var p in hoverPoints)
        {
            if (!p) continue;
            bool grounded = Physics.Raycast(p.position, Vector3.down, out hit, hoverHeight * 6f);
            Gizmos.color = grounded && hit.distance <= hoverHeight * 2.5f ? Color.green : Color.red;
            Gizmos.DrawLine(p.position, p.position - Vector3.up * hoverHeight);
            Vector3 contact = grounded ? hit.point : p.position - Vector3.up * hoverHeight;
            Gizmos.DrawWireSphere(contact, 0.07f);
        }
    }
#endif
}
