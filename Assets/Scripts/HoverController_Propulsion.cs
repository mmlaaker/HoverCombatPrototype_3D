using UnityEngine;

/// <summary>
/// Handles propulsion, steering, and physics "feel" for a hover vehicle.
/// Works in tandem with:
///   - HoverController_Foundation  (physics base)
///   - HoverController_Traversal   (optional slope & continuity filter)
/// Input is provided via an IHoverInputProvider (player, AI, etc.).
/// </summary>
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(HoverController_Foundation))]
public class HoverController_Propulsion : MonoBehaviour
{
    [Header("Input Source")]
    [Tooltip("Optional input provider (player, AI, or network). Can be null for pure-physics tests.")]
    public MonoBehaviour inputProvider;

    private IHoverInputProvider input;
    private Rigidbody rb;
    private HoverController_Foundation foundation;
    private HoverController_Traversal traversal; // optional slope-aware helper

    [Header("Movement Settings")]
    [Tooltip("Maximum forward thrust force (N).")]
    public float thrustForce = 10000f;

    [Tooltip("Maximum turning torque (N·m).")]
    public float turnTorque = 5000f;

    [Tooltip("Additional force multiplier when boosting.")]
    public float boostMultiplier = 1.5f;

    [Header("Drag Settings")]
    [Tooltip("Linear drag applied while hovering (acts like air resistance).")]
    public float moveDrag = 1.0f;

    [Tooltip("Angular drag applied while hovering (stabilizes spin).")]
    public float turnDrag = 2.0f;

    [Header("Gravity Profile")]
    [Tooltip("Scales gravity for this vehicle (1 = normal, 3 = heavy).")]
    [Range(0.5f, 5f)] public float gravityScale = 2.5f;

    [Tooltip("If true, heavier gravity only applies when airborne (not grounded).")]
    public bool applyHeavyGravityOnlyWhenAirborne = true;

    [Header("Grounded Friction")]
    [Tooltip("Multiplier for damping sideways slide. Higher = less ice.")]
    [Range(0f, 10f)] public float lateralFriction = 4.0f;

    [Tooltip("Multiplier for damping residual rotation when grounded.")]
    [Range(0f, 10f)] public float rotationalFriction = 3.0f;

    [Header("Debug")]
    public bool drawDebugVectors = true;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        foundation = GetComponent<HoverController_Foundation>();
        traversal = GetComponent<HoverController_Traversal>(); // optional

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

        // ---- Slope-aware thrust direction ----
        Vector3 forwardDir = transform.forward;

        if (grounded && traversal != null)
        {
            // Correct cross order: right × normal = forward-aligned tangent
            Vector3 tangent = Vector3.Cross(transform.right, traversal.GroundNormal);
            if (tangent.sqrMagnitude > 1e-6f)
                forwardDir = tangent.normalized;
        }

        // Forward/backward thrust
        if (Mathf.Abs(throttle) > 0.01f)
        {
            Vector3 force = forwardDir * (thrustForce * throttle * boostMult);
            rb.AddForce(force, ForceMode.Force);
        }

        // Turning torque (yaw)
        if (Mathf.Abs(turn) > 0.01f)
        {
            Vector3 torque = Vector3.up * (turnTorque * turn);
            rb.AddTorque(torque, ForceMode.Force);
        }
    }

    private void ApplyDrag()
    {
        // Isotropic linear drag (slows general motion)
        Vector3 dragForce = -rb.linearVelocity * moveDrag;
        rb.AddForce(dragForce, ForceMode.Force);

        // Angular drag for rotational damping
        Vector3 angDragForce = -rb.angularVelocity * turnDrag;
        rb.AddTorque(angDragForce, ForceMode.Force);
    }

    private void ApplyGroundedFriction(bool grounded)
    {
        if (!grounded) return;

        // Transform velocity into local space
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity);

        // Suppress sideways and vertical motion (hover friction)
        localVel.x = Mathf.Lerp(localVel.x, 0f, Time.fixedDeltaTime * lateralFriction);
        localVel.y = Mathf.Lerp(localVel.y, 0f, Time.fixedDeltaTime * lateralFriction * 0.5f);

        // Convert back to world space
        rb.linearVelocity = transform.TransformDirection(localVel);

        // Dampen unwanted angular velocity (roll/pitch drift)
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
