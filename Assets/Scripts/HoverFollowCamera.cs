using UnityEngine;
using UnityEngine.InputSystem;

public class HoverFollowCamera : MonoBehaviour
{
    [Header("Target")]
    public Transform target;
    private Rigidbody targetRb;

    [Header("Offsets")]
    public Vector3 offset = new Vector3(0f, 3f, -8f);
    public float lookAheadDistance = 5f;
    public float lookAheadThreshold = 1f;

    [Header("Smoothing")]
    public float positionSmoothTime = 0.2f;
    public float rotationSmooth = 6f;

    [Header("Ground Avoidance")]
    public float groundRayLength = 10f;
    public float minHeightAboveGround = 1.0f;

    [Header("Mouse Orbit")]
    public bool enableMouseOrbit = true;
    public float mouseSensitivity = 2f;
    public float minPitch = -10f;
    public float maxPitch = 65f;

    private Vector3 currentVelocity;
    private float yaw;
    private float pitch = 15f;

    void Start()
    {
        if (!target)
        {
            Debug.LogError("HoverFollowCamera missing target!");
            enabled = false;
            return;
        }

        targetRb = target.GetComponent<Rigidbody>();
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

    void HandleMouseInput()
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

    void UpdateCamera()
    {
        // Compute desired offset and lookahead
        Vector3 basePos = target.TransformPoint(offset);

        Vector3 velocity = targetRb ? targetRb.linearVelocity : Vector3.zero;
        Vector3 lookAhead = Vector3.zero;
        if (velocity.sqrMagnitude > lookAheadThreshold * lookAheadThreshold)
            lookAhead = velocity.normalized * lookAheadDistance;

        Vector3 desiredPos = basePos + lookAhead;

        // Smooth damp position (prevents jitter when still)
        Vector3 smoothedPos = Vector3.SmoothDamp(transform.position, desiredPos, ref currentVelocity, positionSmoothTime);

        // --- Ground avoidance ---
        if (Physics.Raycast(smoothedPos + Vector3.up * 2f, Vector3.down, out RaycastHit hit, groundRayLength))
        {
            float groundY = hit.point.y + minHeightAboveGround;
            if (smoothedPos.y < groundY)
                smoothedPos.y = Mathf.Lerp(smoothedPos.y, groundY, Time.deltaTime * 10f);
        }

        // --- Rotation ---
        Quaternion targetRot;
        if (enableMouseOrbit)
        {
            Quaternion orbitRot = Quaternion.Euler(pitch, yaw, 0f);
            targetRot = orbitRot;
        }
        else
        {
            Vector3 dir = (target.position - smoothedPos).normalized;
            targetRot = Quaternion.LookRotation(dir, Vector3.up);
        }

        transform.position = smoothedPos;
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, Time.deltaTime * rotationSmooth);
    }
}
