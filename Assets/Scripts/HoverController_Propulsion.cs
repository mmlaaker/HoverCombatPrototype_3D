using UnityEngine;

/// <summary>
/// HoverController_Propulsion v4.1 (Hybrid + Fixed Arcade Edition)
/// --------------------------------------------------------------
/// Three feel modes:
///   • Physical – realistic, mass-scaled physics
///   • Hybrid   – physical base + impulse/torque boost (futuristic weight)
///   • Arcade   – kinematic velocity interpolation (responsive & consistent)
///
/// Works with:
///   - HoverController_Foundation  (hover physics)
///   - HoverController_Traversal   (slope normal & grounded continuity)
/// </summary>
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(HoverController_Foundation))]
public class HoverController_Propulsion : MonoBehaviour
{
    public enum PropulsionMode { Physical, Hybrid, Arcade }

    [Header("Input Source")]
    [Tooltip("Optional input provider (player, AI, or network). Can be null for pure-physics tests.")]
    public MonoBehaviour inputProvider;

    private IHoverInputProvider input;
    private Rigidbody rb;
    private HoverController_Foundation foundation;
    private HoverController_Traversal traversal;

    [Header("Mode")]
    [Tooltip("Physical = mass scaled, Hybrid = powered feel, Arcade = mass agnostic.")]
    public PropulsionMode mode = PropulsionMode.Hybrid;

    [Header("Movement Settings")]
    [Tooltip("Base forward thrust (N).")]
    public float thrustForce = 20000f;

    [Tooltip("Turning torque (N·m).")]
    public float turnTorque = 8000f;

    [Tooltip("Boost multiplier applied while boosting.")]
    public float boostMultiplier = 1.5f;

    [Header("Speed Control")]
    [Tooltip("Approximate max speed (m/s) where thrust fades to zero.")]
    public float maxSpeed = 40f;

    [Tooltip("Sharpness of thrust fade. Higher = more abrupt cutoff.")]
    [Range(0.5f, 5f)] public float speedFalloffSharpness = 2f;

    [Header("Turn Control")]
    [Tooltip("Approximate max yaw rate (deg/s) before torque fades.")]
    public float maxTurnRate = 100f;

    [Tooltip("How aggressively turn torque fades as yaw speed increases.")]
    [Range(0.1f, 5f)] public float turnFalloffSharpness = 2f;

    [Header("Hybrid Boost Curves")]
    [Tooltip("Extra thrust multiplier at low speed (Hybrid mode only).")]
    public float lowSpeedImpulseBoost = 1.8f;

    [Tooltip("Yaw-rate threshold (deg/s) for initial torque boost (Hybrid mode only).")]
    public float yawImpulseThreshold = 60f;

    [Header("Drag Settings")]
    [Tooltip("Linear drag applied while hovering (acts like air resistance).")]
    public float moveDrag = 0.8f;

    [Tooltip("Angular drag applied while hovering (stabilizes spin).")]
    public float turnDrag = 1.2f;

    [Header("Gravity Profile")]
    [Tooltip("Scales gravity for this vehicle (1 = normal, 3 = heavy).")]
    [Range(0.5f, 5f)] public float gravityScale = 2.5f;

    [Tooltip("If true, heavier gravity only applies when airborne (not grounded).")]
    public bool applyHeavyGravityOnlyWhenAirborne = true;

    [Header("Grounded Friction")]
    [Tooltip("Multiplier for damping sideways slide. Higher = less ice.")]
    [Range(0f, 10f)] public float lateralFriction = 4.0f;

    [Tooltip("Multiplier for damping residual rotation when grounded.")]
    [Range(0f, 10f)] public float rotationalFriction = 4.0f;

    [Header("Debug")]
    public bool drawDebugVectors = true;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        foundation = GetComponent<HoverController_Foundation>();
        traversal = GetComponent<HoverController_Traversal>();

        if (inputProvider is IHoverInputProvider provider)
            input = provider;
    }

    void FixedUpdate()
    {
        bool grounded = foundation.IsHoverGrounded || foundation.IsCollidedGrounded;
        ApplyGravity(grounded);

        if (input != null)
            ApplyMovement(input.Throttle, input.Turn, input.Boost, grounded);

        ApplyDrag();
        ApplyGroundedFriction(grounded);
    }

    // --------------------------------------------------
    // PHYSICS LAYERS
    // --------------------------------------------------

    private void ApplyGravity(bool grounded)
    {
        if (!applyHeavyGravityOnlyWhenAirborne || !grounded)
        {
            Vector3 scaledGravity = Physics.gravity * rb.mass * (gravityScale - 1f);
            rb.AddForce(scaledGravity, ForceMode.Force);
        }
    }

    private void ApplyMovement(float throttle, float turn, bool boost, bool grounded)
    {
        float boostMult = boost ? boostMultiplier : 1f;

        // --- Input shaping for smoother low-end response ---
        float curvedThrottle = Mathf.Sign(throttle) * Mathf.Pow(Mathf.Abs(throttle), 1.2f);
        float curvedTurn = Mathf.Sign(turn) * Mathf.Pow(Mathf.Abs(turn), 1.1f);

        // --- Slope-aware forward direction ---
        Vector3 forwardDir = transform.forward;
        if (grounded && traversal != null)
        {
            Vector3 tangent = Vector3.Cross(transform.right, traversal.GroundNormal);
            if (tangent.sqrMagnitude > 1e-6f)
                forwardDir = tangent.normalized;
        }

        // --- Common speed metrics ---
        float speed = rb.linearVelocity.magnitude;
        float speedNorm = Mathf.Clamp01(speed / maxSpeed);
        float speedFactor = 1f - Mathf.Pow(speedNorm, speedFalloffSharpness);

        // =====================================
        //   PHYSICAL / HYBRID MODES
        // =====================================
        if (mode != PropulsionMode.Arcade)
        {
            ForceMode forceMode = ForceMode.Force;

            // --- Forward / Reverse ---
            if (Mathf.Abs(curvedThrottle) > 0.01f)
            {
                float impulseBoost = 1f;
                if (mode == PropulsionMode.Hybrid)
                    impulseBoost = Mathf.Lerp(lowSpeedImpulseBoost, 1f, Mathf.Pow(speedNorm, 2f));

                float totalThrust = thrustForce * curvedThrottle * boostMult * speedFactor * impulseBoost;
                rb.AddForce(forwardDir * totalThrust, forceMode);
            }

            // --- Turning ---
            if (Mathf.Abs(curvedTurn) > 0.01f)
            {
                float yawSpeed = Mathf.Abs(Vector3.Dot(rb.angularVelocity, Vector3.up)) * Mathf.Rad2Deg;
                float turnFactor = 1f - Mathf.Pow(Mathf.Clamp01(yawSpeed / maxTurnRate), turnFalloffSharpness);

                float torqueBoost = 1f;
                if (mode == PropulsionMode.Hybrid && yawSpeed < yawImpulseThreshold)
                    torqueBoost = Mathf.Lerp(1.5f, 1f, yawSpeed / yawImpulseThreshold);

                float totalTorque = turnTorque * curvedTurn * turnFactor * torqueBoost * speedFactor;
                rb.AddTorque(Vector3.up * totalTorque, forceMode);
            }

            return; // Done for physical/hybrid
        }

        // =====================================
        //   ARCADE MODE (kinematic-style)
        // =====================================
        Vector3 desiredVelocity = forwardDir * (curvedThrottle * maxSpeed * boostMult);
        float accelRate = moveDrag * 2f + 3f;  // tune this for snappiness
        rb.linearVelocity = Vector3.Lerp(rb.linearVelocity, desiredVelocity, Time.fixedDeltaTime * accelRate);

        // Yaw control via angular velocity
        float targetYawRate = curvedTurn * maxTurnRate * Mathf.Deg2Rad;
        Vector3 angVel = rb.angularVelocity;
        float turnRate = turnDrag * 2f + 2f; // tune for steering twitch
        angVel.y = Mathf.Lerp(angVel.y, targetYawRate, Time.fixedDeltaTime * turnRate);
        rb.angularVelocity = angVel;
    }

    private void ApplyDrag()
    {
        Vector3 dragForce = -rb.linearVelocity * moveDrag;
        rb.AddForce(dragForce, ForceMode.Force);

        Vector3 angDragForce = -rb.angularVelocity * turnDrag;
        rb.AddTorque(angDragForce, ForceMode.Force);
    }

    private void ApplyGroundedFriction(bool grounded)
    {
        if (!grounded) return;

        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);
        localVel.x = Mathf.Lerp(localVel.x, 0f, Time.fixedDeltaTime * lateralFriction);
        localVel.y = Mathf.Lerp(localVel.y, 0f, Time.fixedDeltaTime * lateralFriction * 0.5f);
        rb.linearVelocity = transform.TransformDirection(localVel);

        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);
        localAngVel.x = Mathf.Lerp(localAngVel.x, 0f, Time.fixedDeltaTime * rotationalFriction);
        localAngVel.z = Mathf.Lerp(localAngVel.z, 0f, Time.fixedDeltaTime * rotationalFriction);
        rb.angularVelocity = transform.TransformDirection(localAngVel);
    }

#if UNITY_EDITOR
    void OnDrawGizmosSelected()
    {
        if (!drawDebugVectors) return;

        Vector3 origin = transform.position;
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(origin, origin + transform.forward * 2f);
        Gizmos.DrawSphere(origin + transform.forward * 2f, 0.05f);

        if (Application.isPlaying && traversal != null)
        {
            // Ground normal (cyan)
            Gizmos.color = Color.cyan;
            Gizmos.DrawLine(origin, origin + traversal.GroundNormal * 2f);
            Gizmos.DrawSphere(origin + traversal.GroundNormal * 2f, 0.05f);

            // Projected tangent (green)
            Vector3 tangent = Vector3.Cross(transform.right, traversal.GroundNormal).normalized;
            Gizmos.color = Color.green;
            Gizmos.DrawLine(origin, origin + tangent * 2f);
            Gizmos.DrawSphere(origin + tangent * 2f, 0.05f);
        }
    }
#endif
}
