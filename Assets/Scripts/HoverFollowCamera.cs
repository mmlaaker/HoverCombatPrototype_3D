using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Smoothed follow camera for ballistic hovercraft motion.
/// - Position & rotation smoothing
/// - Velocity-based look-ahead
/// - Independent vertical smoothing
/// - Optional mouse orbit
/// </summary>
public class HoverFollowCamera : MonoBehaviour
{
    [Header("Target")]
    public Transform target;

    [Header("Offsets")]
    public Vector3 offset = new Vector3(0f, 3f, -8f);
    [Tooltip("How much the camera looks ahead in movement direction.")]
    public float lookAheadDistance = 5f;

    [Header("Smoothing")]
    public float positionSmooth = 6f;
    public float rotationSmooth = 5f;
    [Tooltip("Vertical smoothing (lower = more floaty).")]
    public float verticalSmooth = 3f;

    [Header("Mouse Control")]
    public bool enableMouseOrbit = true;
    public float mouseSensitivity = 2f;
    public float minPitch = -15f;
    public float maxPitch = 60f;

    private Vector3 velocity;
    private Vector3 smoothedPos;
    private float yaw;
    private float pitch = 15f;
    private Rigidbody targetRb;

    void Start()
    {
        if (!target)
        {
            Debug.LogError("HoverFollowCamera missing target!");
            enabled = false;
            return;
        }

        targetRb = target.GetComponent<Rigidbody>();
        smoothedPos = target.position + target.TransformVector(offset);
        Vector3 euler = transform.eulerAngles;
        yaw = euler.y;
        pitch = euler.x;
    }

    void LateUpdate()
    {
        if (!target) return;

        HandleMouseInput();
        UpdateCamera();
    }

    // ----------------------------------------------------------
    private void HandleMouseInput()
    {
#if ENABLE_INPUT_SYSTEM
        if (!enableMouseOrbit || Mouse.current == null)
            return;

        Vector2 delta = Mouse.current.delta.ReadValue() * mouseSensitivity;
        yaw += delta.x;
        pitch -= delta.y;
        pitch = Mathf.Clamp(pitch, minPitch, maxPitch);
#else
        if (!enableMouseOrbit) return;

        yaw += Input.GetAxis("Mouse X") * mouseSensitivity;
        pitch -= Input.GetAxis("Mouse Y") * mouseSensitivity;
        pitch = Mathf.Clamp(pitch, minPitch, maxPitch);
#endif
    }

    // ----------------------------------------------------------
    private void UpdateCamera()
    {
        // --- Target base position ---
        Vector3 baseTargetPos = target.TransformPoint(offset);

        // --- Velocity-based look-ahead ---
        Vector3 velocityDir = targetRb ? targetRb.linearVelocity.normalized : target.forward;
        Vector3 lookAhead = velocityDir * lookAheadDistance;
        Vector3 desiredPos = baseTargetPos + lookAhead;

        // --- Smooth position ---
        Vector3 horizTarget = new Vector3(desiredPos.x, smoothedPos.y, desiredPos.z);
        smoothedPos = Vector3.Lerp(smoothedPos, horizTarget, Time.deltaTime * positionSmooth);

        // Separate vertical smoothing (gentler on hops)
        smoothedPos.y = Mathf.Lerp(smoothedPos.y, desiredPos.y, Time.deltaTime * verticalSmooth);

        // --- Compute rotation ---
        Quaternion targetRot;

        if (enableMouseOrbit)
        {
            Quaternion orbitRot = Quaternion.Euler(pitch, yaw, 0f);
            targetRot = orbitRot;
        }
        else
        {
            Vector3 dirToTarget = (target.position - smoothedPos).normalized;
            targetRot = Quaternion.LookRotation(dirToTarget, Vector3.up);
        }

        // --- Apply smoothed rotation ---
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, Time.deltaTime * rotationSmooth);
        transform.position = smoothedPos;
    }
}
