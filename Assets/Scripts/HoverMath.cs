using UnityEngine;

/// <summary>
/// Shared math utilities for hover controller scripts.
/// </summary>
public static class HoverMath
{
    /// <summary>
    /// Normalizes a Unity euler angle from 0..360 to -180..180.
    /// Required because Unity returns 350 for -10 degrees.
    /// </summary>
    public static float NormalizeAngle(float angle)
    {
        if (angle > 180f) angle -= 360f;
        return angle;
    }
}
