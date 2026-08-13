using UnityEngine;

/// <summary>
/// VehicleLayerAssigner v1.0
/// ---------------------------------
/// Attach to the root of any vehicle prefab alongside the input provider.
/// At Awake, detects whether the vehicle is player-driven or AI-driven and
/// assigns the entire hierarchy to the correct layer.
///
/// This allows a single shared prefab to split into PlayerVehicle / AIVehicle
/// layers at runtime, enabling per-layer collision filtering (e.g. particle
/// weapons that only hit the opposing team).
///
/// Requires layers "PlayerVehicle" and "AIVehicle" to exist in the Tag Manager.
/// DefaultExecutionOrder(-20) makes this the earliest thing on the vehicle. It is
/// the lowest order in the project: VehicleHealth is -10 and everything else is
/// the default 0, so the layer is settled before any Awake that reads it. The one
/// remaining consumer that depends on that ordering is ParticleWeaponCollision,
/// which strips the firer's own layer from the emitter collision mask.
/// VehicleHUD used to be the second consumer, stripping this layer from a reticle
/// aim raycast. That raycast was deleted 2026-08-13 when the crosshair stopped
/// following hit depth, so the HUD no longer reads the layer at all.
/// </summary>
[DefaultExecutionOrder(-20)]
public class VehicleLayerAssigner : MonoBehaviour
{
    private void Awake()
    {
        int layer;

        if (GetComponent<PlayerHoverInput>() != null)
            layer = LayerMask.NameToLayer("PlayerVehicle");
        else if (GetComponent<AIHoverInput>() != null)
            layer = LayerMask.NameToLayer("AIVehicle");
        else
        {
            Debug.LogWarning("[VehicleLayerAssigner] No recognised input provider found. " +
                             "Layer unchanged.", this);
            return;
        }

        if (layer < 0)
        {
            Debug.LogError("[VehicleLayerAssigner] Required layer not found in Tag Manager. " +
                           "Add 'PlayerVehicle' and 'AIVehicle' layers.", this);
            return;
        }

        SetLayerRecursive(gameObject, layer);
    }

    private static void SetLayerRecursive(GameObject go, int layer)
    {
        go.layer = layer;
        foreach (Transform child in go.transform)
            SetLayerRecursive(child.gameObject, layer);
    }
}
