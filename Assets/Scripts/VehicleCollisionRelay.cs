using UnityEngine;

/// <summary>
/// Forwards the vehicle's collision impacts to HoverCameraImpulseRouter, which
/// lives on the camera rig and therefore cannot receive them itself.
///
/// WHY THIS EXISTS AT ALL. Unity delivers OnCollisionEnter to the GameObject that
/// owns the Rigidbody, and to nothing else. It does not propagate to children, and
/// there is no subscribe-to-collisions API. So a component that needs impacts must
/// physically sit on the vehicle root. When the impulse router moved onto the
/// Camera object to group everything camera-related in one place, crash detection
/// was the one thing that could not come with it.
///
/// This is deliberately the whole file. It holds no tuning, makes no decisions and
/// filters nothing: severity, the floor-angle test, the cooldown and the direction
/// all stay in the router, where the rest of the crash logic already lives. A relay
/// that started making judgement calls would be a second place to look when the
/// shake misbehaves, which is exactly what the router was built to avoid.
///
/// Foundation also has an OnCollisionStay on this same GameObject, for contact
/// tracking. The two do not interact: Stay tracks a state, Enter is an edge.
/// </summary>
public class VehicleCollisionRelay : MonoBehaviour
{
    [Tooltip("The camera's impulse router. Lives on the Camera object under " +
             "HoverCameraController, so it cannot see this vehicle's collisions without " +
             "this relay. Unassigned means crash shake is silently off, which is why " +
             "Awake complains about it.")]
    [SerializeField] private HoverCameraImpulseRouter impulseRouter;

    private void Awake()
    {
        if (impulseRouter == null)
            Debug.LogWarning("[CollisionRelay] Impulse Router unassigned. Crashes will not shake " +
                             "the camera. Everything else about the vehicle is unaffected.", this);
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (impulseRouter != null)
            impulseRouter.ReportCollision(collision);
    }
}
