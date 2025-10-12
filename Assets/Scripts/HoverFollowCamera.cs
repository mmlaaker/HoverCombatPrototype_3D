using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

/// <summary>
/// Smooth third-person follow camera for HoverController testing.
/// Compatible with Unity's new Input System.
/// Now includes yaw/pitch clamping to prevent excessive orbit.
/// </summary>
public class HoverFollowCamera : MonoBehaviour
{
    [Header("Target Settings")]
    public Transform target;
    public Vector3 offset = new Vector3(0f, 4f, -10f);

    [Header("Follow Settings")]
    public float positionSmoothTime = 0.2f;
    public float rotationSmoothTime = 5f;

    [Header("Dynamic Behavior")]
    public float velocityLag = 0.1f;
    public float turnBankAmount = 3f;

    [Header("Mouse Control")]
    public bool enableMouseControl = true;
    public float mouseSensitivity = 3f;

    [Tooltip("How quickly camera returns to default offset when not orbiting.")]
    public float returnToFollowSpeed = 1.5f;

    [Header("Angle Limits")]
    [Tooltip("Maximum upward pitch (degrees).")]
    public float pitchUpLimit = 60f;
    [Tooltip("Maximum downward pitch (degrees).")]
    public float pitchDownLimit = 20f;
    [Tooltip("Yaw limit left/right (degrees from behind the craft).")]
    public float yawLimit = 80f;

    private Vector3 velocitySmoothDamp;
    private Rigidbody targetRb;

    private float yaw;
    private float pitch;
    private bool isOrbiting;

    void Start()
    {
        if (target != null)
        {
            targetRb = target.GetComponent<Rigidbody>();
            Vector3 relativePos = target.InverseTransformPoint(transform.position);
            Vector3 dir = relativePos.normalized;
            yaw = Mathf.Atan2(dir.x, -dir.z) * Mathf.Rad2Deg;
            pitch = Mathf.Asin(dir.y) * Mathf.Rad2Deg;
        }
    }

    void LateUpdate()
    {
        if (!target) return;

        HandleMouseInput();
        UpdateCameraPositionAndRotation();
    }

    // ------------------------------------------------------------------------
    // INPUT
    // ------------------------------------------------------------------------
    void HandleMouseInput()
    {
        if (!enableMouseControl) return;

#if ENABLE_INPUT_SYSTEM
        if (Mouse.current != null)
        {
            if (Mouse.current.rightButton.isPressed)
            {
                isOrbiting = true;

                Vector2 delta = Mouse.current.delta.ReadValue();
                yaw += delta.x * mouseSensitivity * Time.deltaTime * 60f;
                pitch -= delta.y * mouseSensitivity * Time.deltaTime * 60f;

                // Clamp pitch (hard limit)
                pitch = Mathf.Clamp(pitch, -pitchDownLimit, pitchUpLimit);

                // Clamp yaw (soft limit)
                yaw = Mathf.Clamp(yaw, -yawLimit, yawLimit);
            }
            else if (isOrbiting)
            {
                yaw = Mathf.Lerp(yaw, 0f, Time.deltaTime * returnToFollowSpeed);
                pitch = Mathf.Lerp(pitch, 0f, Time.deltaTime * returnToFollowSpeed);

                if (Mathf.Abs(yaw) < 0.1f && Mathf.Abs(pitch) < 0.1f)
                    isOrbiting = false;
            }
        }
#else
        if (Input.GetMouseButton(1))
        {
            isOrbiting = true;

            float mouseX = Input.GetAxis("Mouse X") * mouseSensitivity;
            float mouseY = Input.GetAxis("Mouse Y") * mouseSensitivity;

            yaw += mouseX;
            pitch -= mouseY;

            pitch = Mathf.Clamp(pitch, -pitchDownLimit, pitchUpLimit);
            yaw = Mathf.Clamp(yaw, -yawLimit, yawLimit);
        }
        else if (isOrbiting)
        {
            yaw = Mathf.Lerp(yaw, 0f, Time.deltaTime * returnToFollowSpeed);
            pitch = Mathf.Lerp(pitch, 0f, Time.deltaTime * returnToFollowSpeed);

            if (Mathf.Abs(yaw) < 0.1f && Mathf.Abs(pitch) < 0.1f)
                isOrbiting = false;
        }
#endif
    }

    // ------------------------------------------------------------------------
    // CAMERA LOGIC
    // ------------------------------------------------------------------------
    void UpdateCameraPositionAndRotation()
    {
        Vector3 dynamicOffset = offset;

        if (targetRb)
        {
            Vector3 localVel = target.InverseTransformDirection(targetRb.linearVelocity);
            dynamicOffset += target.TransformDirection(-localVel * velocityLag);
        }

        if (enableMouseControl)
        {
            Quaternion orbitRot = Quaternion.Euler(pitch, yaw, 0f);
            dynamicOffset = orbitRot * offset;
        }

        Vector3 desiredPosition = target.TransformPoint(dynamicOffset);

        transform.position = Vector3.SmoothDamp(transform.position, desiredPosition,
                                                ref velocitySmoothDamp, positionSmoothTime);

        Quaternion lookRot = Quaternion.LookRotation(target.position - transform.position, Vector3.up);

        if (targetRb)
        {
            float yawRate = targetRb.angularVelocity.y;
            lookRot *= Quaternion.Euler(0f, 0f, -yawRate * turnBankAmount);
        }

        transform.rotation = Quaternion.Slerp(transform.rotation, lookRot, rotationSmoothTime * Time.deltaTime);
    }

#if UNITY_EDITOR
    private void OnDrawGizmosSelected()
    {
        if (target)
        {
            Gizmos.color = Color.cyan;
            Vector3 desiredPos = target.TransformPoint(offset);
            Gizmos.DrawLine(target.position, desiredPos);
            Gizmos.DrawWireSphere(desiredPos, 0.25f);
        }
    }
#endif
}
