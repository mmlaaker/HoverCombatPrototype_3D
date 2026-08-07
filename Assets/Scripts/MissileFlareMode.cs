/// <summary>
/// Which way a missile swings before it turns onto its target.
///
/// The flare is expressed as a roll angle around the launch axis, so every option here is
/// really just "which direction is outward for this shot". Alternate and Random exist because a
/// fixed direction reads as a scripted animation once you have seen it twice; varying it per
/// shot is what makes a salvo look like a salvo.
/// </summary>
public enum MissileFlareMode
{
    /// <summary>Left, then right, then left. Best for volleys: they split around the target.</summary>
    Alternate,

    /// <summary>Random roll around the launch axis, so shots fan out in a cone.</summary>
    Random,

    /// <summary>Always out to the launcher's left.</summary>
    Left,

    /// <summary>Always out to the launcher's right.</summary>
    Right,

    /// <summary>Always up. Reads as a lofted arc that drops onto the target.</summary>
    Up,

    /// <summary>Always down. Hugs terrain on the way in.</summary>
    Down,
}
