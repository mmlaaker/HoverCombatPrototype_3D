using UnityEngine;
using System.Linq;

[RequireComponent(typeof(Rigidbody))]
public class HoverController : MonoBehaviour
{
    // ============================================================
    // --- Hover Configuration (Weighted Sedan Stable) ---
    // ============================================================

    [Header("Hover Settings")]
    public Transform[] hoverPoints;
    public float hoverHeight = 1.8f;
    public float hoverForce = 280f;
    public float hoverDamp = 12f; // slightly higher damping
    public float preloadBias = 0.2f;

    [Header("Movement Settings")]
    public float thrustAccel = 100f;
    public float turnTorque = 10f;
    public float maxSpeed = 200f;

    [Header("Stability Settings")]
    public float driftDamp = 0.6f;
    public float driftBoostFactor = 3f;
    public float verticalDamp = 7f;
    public float alignStrength = 5f;

    [Header("Air Control")]
    [Range(0f, 1f)] public float minGroundedRatio = 0.2f;
    public float groundedBlendSpeed = 15f;
    public float airAngularDrag = 0.05f;

    [Header("Drag & Leveling")]
    public float linearDrag = 1f;
    public float groundAngularDrag = 0.2f;
    public float levelingStrength = 5.5f;  // lowered from 6.5
    [Range(0f, 1f)] public float angularDampFactor = 0.75f;

    [Header("Visual Tilt (Pitch Response)")]
    public float pitchResponse = 1.2f;
    public float maxPitchAngle = 12f;
    public float tiltSmooth = 2f;

    [Header("Center of Mass")]
    public Transform centerOfMassTransform;

    [Header("Airborne Drop Recovery")]
    public float groundMemoryDuration = 0.15f;
    [Range(0f, 1f)] public float airborneDescentGain = 0.3f;
    public float airborneGravityBoost = 1.5f;
    public float noContactBlendFalloff = 10f;

    // ============================================================
    // --- Internal State ---
    // ============================================================

    private Rigidbody rb;
    private Vector3 smoothedGroundNormal = Vector3.up;
    private float groundedBlend = 1f;
    private float visualPitchOffset = 0f;
    private bool initialized;

    // Hover-point smoothing
    private float[] smoothedHeightErrors = new float[8];
    private const float heightSmoothFactor = 0.3f;

    // Drop-recovery memory
    private float lastKnownGroundY = 0f;
    private float groundMemoryTimer = 0f;

    // ============================================================
    // --- Unity Lifecycle ---
    // ============================================================

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = false;
        rb.linearDamping = linearDrag;
        rb.angularDamping = groundAngularDrag;

        if (centerOfMassTransform != null)
            rb.centerOfMass = transform.InverseTransformPoint(centerOfMassTransform.position);
        else
            rb.centerOfMass = new Vector3(0f, -0.2f, -0.3f);

        if (hoverPoints != null)
            hoverPoints = hoverPoints.Where(p => p != null).ToArray();

        AdjustSpawnHeightInstantly();
        initialized = true;
    }

    private void FixedUpdate()
    {
        if (!initialized || hoverPoints == null || hoverPoints.Length == 0) return;

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
        if (smoothedHeightErrors.Length != hoverPoints.Length)
            smoothedHeightErrors = new float[hoverPoints.Length];

        int contactCount = 0;
        float avgDistance = 0f;
        float totalGroundY = 0f;
        float avgHeightError = 0f;
        RaycastHit hit;

        for (int i = 0; i < hoverPoints.Length; i++)
        {
            var point = hoverPoints[i];
            if (!point) continue;

            if (Physics.Raycast(point.position, Vector3.down, out hit, hoverHeight * 6f))
            {
                if (hit.distance > hoverHeight * 3.5f)
                    continue;

                contactCount++;
                avgDistance += hit.distance;
                totalGroundY += hit.point.y;

                // Smoothed height error
                float heightError = hoverHeight - hit.distance;
                smoothedHeightErrors[i] = Mathf.Lerp(smoothedHeightErrors[i], heightError, heightSmoothFactor);
                float smoothedError = smoothedHeightErrors[i];

                float normalized = Mathf.Clamp01((smoothedError + preloadBias) / hoverHeight);
                float upwardSpeed = rb.GetPointVelocity(point.position).y;

                // --- Variable damping: stronger near zero velocity ---
                float dampingScale = Mathf.Lerp(1.5f, 0.8f, Mathf.Clamp01(Mathf.Abs(upwardSpeed) / 5f));
                float damping = Mathf.Clamp(upwardSpeed * hoverDamp * dampingScale, -hoverForce, hoverForce);

                float distanceFalloff = Mathf.Clamp01(1f - (hit.distance / (hoverHeight * 3.5f)));

                // --- Front-point bias ---
                float frontBias = 1f;
                if (i <= 1) frontBias = 0.8f;

                float liftAccel = Mathf.Clamp(
                    ((Mathf.Pow(normalized, 2.0f) * hoverForce * distanceFalloff * frontBias) - damping),
                    -hoverForce, hoverForce);

                rb.AddForceAtPosition(hit.normal * liftAccel * groundedBlend, point.position, ForceMode.Acceleration);
                avgHeightError += heightError;
            }
        }

        // Shared body-lift bias (halved intensity)
        if (contactCount > 0)
        {
            avgHeightError /= contactCount;
            rb.AddForce(Vector3.up * avgHeightError * hoverForce * 0.025f, ForceMode.Acceleration);
        }

        // --- Grounded blend with soft ramp-in ---
        float contactRatio = (float)contactCount / hoverPoints.Length;
        float rawBlend = Mathf.Clamp01((contactRatio - minGroundedRatio) / (1f - minGroundedRatio));
        float targetBlend = Mathf.Pow(rawBlend, 2f);

        float blendLerpSpeed = groundedBlend < targetBlend ? groundedBlendSpeed * 0.25f : groundedBlendSpeed;
        groundedBlend = Mathf.Lerp(groundedBlend, targetBlend, 1f - Mathf.Exp(-blendLerpSpeed * Time.fixedDeltaTime));

        // --- Airborne logic ---
        if (contactCount == 0)
        {
            rb.AddForce(Physics.gravity * (1f + airborneGravityBoost * (1f - groundedBlend)), ForceMode.Acceleration);

            if (groundMemoryTimer > 0f)
            {
                groundMemoryTimer -= Time.fixedDeltaTime;
                float desiredY = lastKnownGroundY + hoverHeight;
                float heightError = Mathf.Clamp(desiredY - transform.position.y, -hoverHeight * 2f, hoverHeight * 2f);
                rb.AddForce(Vector3.up * heightError * hoverForce * airborneDescentGain, ForceMode.Acceleration);
            }

            rb.angularDamping = airAngularDrag;
            return;
        }

        // Contacts found
        lastKnownGroundY = totalGroundY / contactCount;
        groundMemoryTimer = groundMemoryDuration;

        rb.AddForce(Physics.gravity * (1f - groundedBlend), ForceMode.Acceleration);
        rb.angularDamping = Mathf.Lerp(airAngularDrag, groundAngularDrag, groundedBlend);

        // Gentle global correction
        if (groundedBlend > 0.5f)
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

        if (Mathf.Abs(forwardInput) > 0.01f && speed < maxSpeed)
            rb.AddForce(thrustDir * forwardInput * thrustAccel, ForceMode.Acceleration);

        if (Mathf.Abs(turnInput) > 0.01f)
            rb.AddTorque(transform.up * turnInput * turnTorque, ForceMode.Acceleration);
        else
        {
            Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);
            localAngVel.y *= 0.9f;
            rb.angularVelocity = transform.TransformDirection(localAngVel);
        }

        Vector3 projectedVel = Vector3.ProjectOnPlane(currentVel, planeNormal);
        Vector3 lateralVel = projectedVel - Vector3.Project(projectedVel, transform.forward);
        Vector3 verticalVel = currentVel - projectedVel;

        float dynamicDrift = driftDamp * (Mathf.Abs(turnInput) > 0.01f ? driftBoostFactor : 1f);
        Vector3 damping = (-lateralVel * dynamicDrift) - (verticalVel * verticalDamp);
        rb.AddForce(damping, ForceMode.Acceleration);

        if (rb.linearVelocity.sqrMagnitude > 1f)
        {
            Vector3 desiredVel = transform.forward * Vector3.Dot(rb.linearVelocity, transform.forward);
            rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, desiredVel, Time.fixedDeltaTime * alignStrength);
        }
    }

    // ============================================================
    // --- Leveling & Tilt ---
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
    // --- Utility ---
    // ============================================================

    private Vector3 GetSmoothedGroundNormal()
    {
        if (hoverPoints == null || hoverPoints.Length == 0) return smoothedGroundNormal;

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

        // Slower smoothing to reduce normal “whip”
        smoothedGroundNormal = Vector3.Slerp(smoothedGroundNormal, targetNormal, Time.fixedDeltaTime * 4f);
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
        for (int i = 0; i < hoverPoints.Length; i++)
        {
            var p = hoverPoints[i];
            if (!p) continue;
            bool grounded = Physics.Raycast(p.position, Vector3.down, out hit, hoverHeight * 6f);
            Gizmos.color = grounded && hit.distance <= hoverHeight * 3.5f ? Color.green : Color.red;
            Gizmos.DrawLine(p.position, p.position - Vector3.up * hoverHeight);
            Vector3 contact = grounded ? hit.point : p.position - Vector3.up * hoverHeight;
            Gizmos.DrawWireSphere(contact, 0.07f);
        }
    }
#endif
}
