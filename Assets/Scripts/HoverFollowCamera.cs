using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// HoverFollowCamera v2.1
/// ----------------------------------------
/// A testing follow camera for hovercraft vehicles.
/// Orbits via right mouse button hold. Follows with smoothing and lookahead.
/// Ground avoidance runs on the desired position before smoothing to prevent clipping.
/// When not orbiting, yaw/pitch spring back to the vehicle's forward facing direction.
///
/// Intended as a testing camera only — replace with Cinemachine for production.
///
/// Setup:
/// 1. Attach to a camera GameObject
/// 2. Assign the vehicle Transform to Target
/// </summary>
public class HoverFollowCamera : MonoBehaviour
{
    [Header("Target")]
    [Tooltip("The vehicle transform to follow.")]
    public Transform target;
    private Rigidbody _targetRb;

    [Header("Offset")]
    [Tooltip("Camera offset in target local space.")]
    public Vector3 offset = new Vector3(0f, 3f, -8f);

    [Header("Lookahead")]
    [Tooltip("How far ahead of the vehicle the camera looks based on velocity.")]
    public float lookAheadDistance = 5f;
    [Tooltip("Minimum speed (m/s) before lookahead activates.")]
    public float lookAheadThreshold = 1f;

    [Header("Smoothing")]
    [Tooltip("Position smooth damp time. Lower = snappier.")]
    public float positionSmoothTime = 0.2f;
    [Tooltip("Rotation lerp speed.")]
    public float rotationSmooth = 6f;

    [Header("Ground Avoidance")]
    [Tooltip("Ray length for ground detection below the camera.")]
    public float groundRayLength = 10f;
    [Tooltip("Minimum height the camera is allowed above detected ground.")]
    public float minHeightAboveGround = 1.5f;

    [Header("Mouse Orbit")]
    [Tooltip("Hold right mouse button to orbit.")]
    public float mouseSensitivity = 2f;
    [Tooltip("Minimum pitch angle (degrees).")]
    public float minPitch = -10f;
    [Tooltip("Maximum pitch angle (degrees).")]
    public float maxPitch = 65f;

    [Header("Auto Recenter")]
    [Tooltip("How quickly yaw and pitch spring back to vehicle forward when not orbiting. Higher = snappier.")]
    public float recenterSpeed = 3f;
    [Tooltip("Neutral pitch to return to when not orbiting (degrees).")]
    public float neutralPitch = 15f;

    // ── Private State ────────────────────────────────────────────────────

    private Vector3 _smoothVelocity;
    private float _yaw;
    private float _pitch = 15f;
    private bool _isOrbiting;

    // ── Unity Lifecycle ──────────────────────────────────────────────────

    private void Start()
    {
        if (!target)
        {
            Debug.LogError("[HoverFollowCamera] No target assigned.", this);
            enabled = false;
            return;
        }

        _targetRb = target.GetComponent<Rigidbody>();

        // Initialise yaw to vehicle forward, pitch to neutral so camera doesn't snap on Start
        _yaw   = target.eulerAngles.y;
        _pitch = neutralPitch;
    }

    private void LateUpdate()
    {
        if (!target) return;

        HandleMouseOrbit();
        UpdateCamera();
    }

    // ── Input ────────────────────────────────────────────────────────────

    private void HandleMouseOrbit()
    {
        if (Mouse.current == null) return;

        _isOrbiting = Mouse.current.rightButton.isPressed;

        if (_isOrbiting)
        {
            Vector2 delta = Mouse.current.delta.ReadValue() * mouseSensitivity;
            _yaw   += delta.x;
            _pitch -= delta.y;
            _pitch  = Mathf.Clamp(_pitch, minPitch, maxPitch);
        }
        else
        {
            // Spring yaw back to vehicle's current world-space forward yaw.
            // Use LerpAngle to correctly handle the 0/360 wrap boundary.
            float targetYaw = target.eulerAngles.y;
            _yaw   = Mathf.LerpAngle(_yaw,   targetYaw,    Time.deltaTime * recenterSpeed);
            _pitch = Mathf.LerpAngle(_pitch, neutralPitch, Time.deltaTime * recenterSpeed);
        }
    }

    // ── Camera Update ────────────────────────────────────────────────────

    private void UpdateCamera()
    {
        // ── Desired position ─────────────────────────────────────────────
        // Rotate the local offset by current yaw/pitch orbit angles,
        // then place it relative to the target. This ensures the camera
        // always orbits around the vehicle regardless of world position.
        Quaternion orbitRotation  = Quaternion.Euler(_pitch, _yaw, 0f);
        Vector3    rotatedOffset  = orbitRotation * offset;
        Vector3    desiredPos     = target.position + rotatedOffset;

        // ── Lookahead ────────────────────────────────────────────────────
        // Offset the look target forward in world space based on velocity.
        // Applied to the look target, not the camera position, so it doesn't
        // fight the orbit offset direction.
        Vector3 velocity  = _targetRb ? _targetRb.linearVelocity : Vector3.zero;
        Vector3 lookPoint = target.position;
        if (velocity.sqrMagnitude > lookAheadThreshold * lookAheadThreshold)
            lookPoint += velocity.normalized * lookAheadDistance;

        // ── Ground avoidance ─────────────────────────────────────────────
        // Run on desiredPos BEFORE smoothing. This prevents the camera from
        // ever targeting a below-ground position, eliminating the clipping
        // that occurs when correction runs after the camera is already placed.
        desiredPos = AvoidGround(desiredPos);

        // ── Smooth damp position ─────────────────────────────────────────
        Vector3 smoothedPos = Vector3.SmoothDamp(
            transform.position, desiredPos, ref _smoothVelocity, positionSmoothTime);

        // ── Rotation ─────────────────────────────────────────────────────
        // Always look at the vehicle (+ lookahead offset).
        // This fixes the original bug where orbit rotation didn't track target position.
        Quaternion desiredRot = Quaternion.LookRotation(
            (lookPoint - smoothedPos).normalized, Vector3.up);

        transform.position = smoothedPos;
        transform.rotation = Quaternion.Slerp(
            transform.rotation, desiredRot, Time.deltaTime * rotationSmooth);
    }

    // ── Helpers ───────────────────────────────────────────────────────────

    /// <summary>
    /// Raises the candidate position above ground if it would clip.
    /// Runs on the desired position before smoothing.
    /// </summary>
    private Vector3 AvoidGround(Vector3 candidatePos)
    {
        // Cast from slightly above candidate to handle cases where candidate
        // is already below ground surface
        Vector3 rayOrigin = candidatePos + Vector3.up * 2f;

        if (Physics.Raycast(rayOrigin, Vector3.down, out RaycastHit hit, groundRayLength + 2f))
        {
            float minY = hit.point.y + minHeightAboveGround;
            if (candidatePos.y < minY)
                candidatePos.y = minY;
        }

        return candidatePos;
    }
}
