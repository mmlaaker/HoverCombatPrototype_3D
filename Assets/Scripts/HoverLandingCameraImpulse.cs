using Unity.Cinemachine;
using UnityEngine;

/// <summary>
/// Fires a Cinemachine impulse camera punch on hard landings, scaled by
/// severity and aimed along the impact axis. Lives on the vehicle root next
/// to the controllers.
///
/// The impulse shape and duration are authored on the CinemachineImpulseSource
/// component; CinemachineImpulseListener components on the vcams pick it up
/// post-body, so HoverCameraController's per-frame FollowOffset writes are
/// never fought.
/// </summary>
[RequireComponent(typeof(CinemachineImpulseSource))]
public class HoverLandingCameraImpulse : MonoBehaviour
{
    // -------------------------------------------------------------------------
    // Tuning
    // -------------------------------------------------------------------------
    [Header("💥 Impulse")]
    [Tooltip("Impulse velocity magnitude at severity 1. The impulse shape and duration " +
             "are authored on the CinemachineImpulseSource. Try 1.5 to 3.")]
    [Min(0f)]
    [SerializeField] private float maxImpulseVelocity = 2f;

    [Header("🛠 Debug")]
    [Tooltip("Severity used by the F9 test hotkey and the 'Test Hard Landing Impulse' " +
             "context-menu item. 0..1, same scale Foundation passes on a real landing. " +
             "Editor only; F9 fires the impulse while in play mode.")]
    [Range(0f, 1f)]
    [SerializeField] private float testSeverity = 1f;

    // -------------------------------------------------------------------------
    // Runtime
    // -------------------------------------------------------------------------
    private HoverController_Foundation foundation;
    private CinemachineImpulseSource   impulseSource;

    // -------------------------------------------------------------------------
    // Unity lifecycle
    // -------------------------------------------------------------------------
    private void Awake()
    {
        foundation    = GetComponent<HoverController_Foundation>();
        impulseSource = GetComponent<CinemachineImpulseSource>();
    }

    private void OnEnable()
    {
        if (foundation != null)
            foundation.OnHardLanding += HandleHardLanding;
    }

    private void OnDisable()
    {
        if (foundation != null)
            foundation.OnHardLanding -= HandleHardLanding;
    }

    // -------------------------------------------------------------------------
    // Hard landing handler
    // -------------------------------------------------------------------------
    /// <summary>Called by Foundation.OnHardLanding. Punch along the impact axis, scaled by severity.</summary>
    private void HandleHardLanding(float severity)
    {
        impulseSource.GenerateImpulseWithVelocity(ImpactDirection * (maxImpulseVelocity * severity));
    }

    /// <summary>
    /// Direction the camera is thrown on impact: into the ground, along the
    /// negated ground normal. This is the same axis Foundation measures impact
    /// speed on, so a landing on a bank punches along the bank instead of
    /// world-down. Foundation assigns the normal on the grounded edge one line
    /// before it fires OnHardLanding, so the read here is the landing frame's.
    ///
    /// Falls back to world-down without a Foundation; Foundation itself reports
    /// Vector3.up while airborne, so the two agree in the air anyway.
    /// </summary>
    private Vector3 ImpactDirection =>
        foundation != null ? -foundation.AverageGroundNormal : Vector3.down;

    /// <summary>
    /// Play-mode test without a real landing: press F9 with the Game view focused.
    /// Runs the exact handler a real landing would, at testSeverity. Preferable to
    /// the context-menu item below: menu navigation stalls the player loop, and the
    /// post-menu frame's deltaTime can step past the entire 0.25s impulse envelope,
    /// so the shake resolves without ever rendering.
    /// </summary>
#if UNITY_EDITOR
    private void Update()
    {
        var kb = UnityEngine.InputSystem.Keyboard.current;
        if (kb != null && kb.f9Key.wasPressedThisFrame)
        {
            Debug.Log($"[LandingImpulse] F9 test impulse fired at severity {testSeverity:F2}.", this);
            HandleHardLanding(testSeverity);
        }
    }
#endif

    [ContextMenu("Test Hard Landing Impulse")]
    private void TestImpulse()
    {
        if (!Application.isPlaying)
        {
            Debug.LogWarning("[LandingImpulse] Enter play mode to test the impulse.", this);
            return;
        }
        HandleHardLanding(testSeverity);
    }
}
