using UnityEngine;
using System.Linq;

/*
=====================================================================
 HOVER CONTROLLER (With Custom Editor System Overview)
=====================================================================

Core hovercraft controller featuring physics-based lift, thrust,
steering, damping, and terrain-aligned orientation.

System documentation is shown in the Unity Inspector through
HoverControllerEditor.cs (scrollable, read-only section at top).

=====================================================================
*/

[RequireComponent(typeof(Rigidbody))]
public class HoverController : MonoBehaviour
{
    // ============================================================
    // --- System Overview (Used by Custom Editor Only) ---
    // ============================================================

    private const string systemOverview =
        "<b>VERTICAL STABILITY (Hover Height Control)</b>\n" +
        "â€¢ <b>hoverHeight</b> â€“ Target ride height above ground.\n" +
        "â€¢ <b>hoverForce / hoverDamp / preloadBias / verticalDamp</b> â€“ Control lift strength, smoothing, and rebound.\n" +
        "  â†‘ Higher hoverForce = stiffer lift, more reactive.\n" +
        "  â†‘ Higher hoverDamp = smoother, slower altitude correction.\n\n" +
        "<b>LATERAL CONTROL (Side Slip & Steering)</b>\n" +
        "â€¢ <b>driftDamp / driftBoostFactor / alignStrength</b> â€“ Control side friction and how tightly velocity follows facing.\n\n" +
        "<b>GROUND TRANSITION (Hover vs. Air)</b>\n" +
        "â€¢ <b>minGroundedRatio / groundedBlendSpeed</b> â€“ Control how easily the craft transitions between hover and fall.\n" +
        "  â†‘ Higher ratio = drops off sooner on crests.\n" +
        "  â†‘ Faster blend = snappier lift recovery.\n\n" +
        "<b>LEVELING & ORIENTATION</b>\n" +
        "â€¢ <b>levelingStrength / angularDampFactor</b> â€“ How strongly the craft stays upright and smooths rotation.\n\n" +
        "<b>VISUAL TILT (Aesthetic Response)</b>\n" +
        "â€¢ <b>pitchResponse / maxPitchAngle / tiltSmooth</b> â€“ Cosmetic tilt when climbing or descending.\n\n" +
        "<b>RECOMMENDED TUNING ORDER:</b>\n" +
        "1. Movement (thrustAccel, turnTorque, maxSpeed)\n" +
        "2. Hover Core (hoverHeight, hoverForce, hoverDamp)\n" +
        "3. Vertical Stability (verticalDamp, preloadBias)\n" +
        "4. Lateral Control (driftDamp, alignStrength, driftBoostFactor)\n" +
        "5. Ground Transition (minGroundedRatio, groundedBlendSpeed)\n" +
        "6. Leveling (levelingStrength, angularDampFactor)\n" +
        "7. Final Polish (linearDrag, visual tilt, airAngularDrag)\n";

    public string GetOverviewText() => systemOverview;


    // ============================================================
    // --- Hover Settings ---
    // ============================================================

    [Header("Hover Settings")]
    [Tooltip("Transforms where hover forces are applied. Place these roughly under each corner of the vehicle.")]
    public Transform[] hoverPoints;

    [Tooltip("Target hover height above ground.")]
    public float hoverHeight = 6f;

    [Tooltip("Upward force used to maintain hover height.")]
    public float hoverForce = 5f;

    [Tooltip("Damping applied to reduce vertical oscillation.")]
    public float hoverDamp = 1.0f;

    [Tooltip("Small upward bias near ground to prevent bottoming out.")]
    public float preloadBias = 0.1f;


    // ============================================================
    // --- Movement ---
    // ============================================================

    [Header("Movement Settings")]
    [Tooltip("Forward/backward acceleration.")]
    public float thrustAccel = 100f;

    [Tooltip("Yaw torque strength.")]
    public float turnTorque = 10f;

    [Tooltip("Maximum forward speed before thrust stops adding velocity.")]
    public float maxSpeed = 100f;


    // ============================================================
    // --- Stability & Refinement ---
    // ============================================================

    [Header("Stability Settings")]
    [Tooltip("Resistance against sideways sliding. Higher = more grip.")]
    public float driftDamp = 0.6f;

    [Tooltip("Multiplier to drift damping during turns.")]
    public float driftBoostFactor = 3f;

    [Tooltip("Additional damping on vertical velocity to prevent bobbing.")]
    public float verticalDamp = 3f;

    [Tooltip("Realignment speed of velocity toward facing direction.")]
    public float alignStrength = 5f;


    // ============================================================
    // --- Gravity Blending & Airtime ---
    // ============================================================

    [Header("Air Control")]
    [Range(0f, 1f)]
    [Tooltip("Fraction of hover points that must detect ground to be considered 'grounded'.")]
    public float minGroundedRatio = 0.65f;

    [Tooltip("How quickly lift and gravity transition when leaving or regaining ground contact.")]
    public float groundedBlendSpeed = 8f;

    [Tooltip("Angular drag used while airborne.")]
    public float airAngularDrag = 0.05f;


    // ============================================================
    // --- Drag & Leveling ---
    // ============================================================

    [Header("Drag & Leveling")]
    [Tooltip("Base linear drag applied constantly.")]
    public float linearDrag = 1f;

    [Tooltip("Angular drag applied when grounded.")]
    public float groundAngularDrag = 0.2f;

    [Tooltip("Torque strength aligning craft 'up' to the terrain normal.")]
    public float levelingStrength = 10f;

    [Range(0f, 1f)]
    [Tooltip("Pitch/roll damping scale. 1 = no damping.")]
    public float angularDampFactor = 0.8f;


    // ============================================================
    // --- Visual Tilt ---
    // ============================================================

    [Header("Visual Tilt (Pitch Response)")]
    [Tooltip("Pitch tilt strength based on vertical velocity.")]
    public float pitchResponse = 0.5f;

    [Tooltip("Maximum pitch tilt in degrees.")]
    public float maxPitchAngle = 10f;

    [Tooltip("Smooth speed of visual tilt transitions.")]
    public float tiltSmooth = 2f;


    // ============================================================
    // --- Center of Mass ---
    // ============================================================

    [Header("Center of Mass")]
    [Tooltip("Optional Transform for custom center of mass (low = stable, high = twitchy).")]
    public Transform centerOfMassTransform;


    // ============================================================
    // --- Internal State ---
    // ============================================================

    private Rigidbody rb;
    private Vector3 smoothedGroundNormal = Vector3.up;
    private float groundedBlend = 1f;
    private bool initialized;
    private float visualPitchOffset = 0f;


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
                if (hit.distance > hoverHeight * 2.5f)
                    continue;

                contactCount++;
                avgDistance += hit.distance;

                float heightError = hoverHeight - hit.distance;
                float normalized = Mathf.Clamp01((heightError + preloadBias) / hoverHeight);
                float upwardSpeed = rb.GetPointVelocity(point.position).y;

                float damping = Mathf.Clamp(upwardSpeed * hoverDamp, -hoverForce * 1.5f, hoverForce * 1.5f);
                float distFactor = Mathf.Clamp01(1f - (hit.distance / (hoverHeight * 2.5f)));

                float liftAccel = (Mathf.Pow(normalized, 2.2f) * hoverForce * distFactor) - damping;
                rb.AddForceAtPosition(hit.normal * liftAccel * groundedBlend, point.position, ForceMode.Acceleration);
            }
        }

        float contactRatio = (float)contactCount / hoverPoints.Length;
        float rawBlend = Mathf.Clamp01((contactRatio - minGroundedRatio) / (1f - minGroundedRatio));
        float targetBlend = Mathf.Pow(rawBlend, 2f);

        if (contactCount == 0)
        {
            groundedBlend = 0f;
            rb.AddForce(Physics.gravity, ForceMode.Acceleration);
            rb.AddForce(Vector3.down * 2f, ForceMode.Acceleration);
            rb.angularDamping = airAngularDrag;
            return;
        }

        groundedBlend = SmoothStepTowards(groundedBlend, targetBlend, groundedBlendSpeed);
        rb.AddForce(Physics.gravity * (1f - groundedBlend), ForceMode.Acceleration);
        rb.angularDamping = Mathf.Lerp(airAngularDrag, groundAngularDrag, groundedBlend);

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
    // --- Leveling & Visual Tilt ---
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
