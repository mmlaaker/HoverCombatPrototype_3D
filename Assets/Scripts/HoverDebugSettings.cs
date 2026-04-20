using UnityEngine;

/// <summary>
/// HoverDebugSettings v1.0
/// -----------------------
/// Global debug toggle for all hover vehicle gizmos and debug rays.
/// Create one asset via Assets > Create > Hover > Debug Settings.
/// Assign it to each vehicle's debug settings field — all vehicles
/// read from the same asset, so one toggle controls everything.
///
/// When no asset is assigned, scripts fall back to their per-instance
/// drawDebug / drawDebugRays fields for backward compatibility.
/// </summary>
[CreateAssetMenu(fileName = "HoverDebugSettings", menuName = "Hover/Debug Settings")]
public class HoverDebugSettings : ScriptableObject
{
    [Tooltip("Master toggle for all hover vehicle debug gizmos and rays. " +
             "Disable to suppress all debug visuals across all vehicles at once.")]
    public bool enableDebugGizmos = true;
}
