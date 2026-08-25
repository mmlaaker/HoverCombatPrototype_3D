/// <summary>
/// Which way a missile's flare swings before it curls back onto the target.
///
/// All of these are a ROLL ANGLE around one shared frame, and that frame is built from WORLD
/// up, not from the launching craft. That matters more than it sounds: a hover craft is banked
/// most of the time it is shooting, and in a chassis-relative frame "left" quietly becomes
/// "down-left" the moment you fire mid-turn, so the same authored setting produces a different
/// arc on every shot. Referencing the world instead means what you author is what you see.
///
/// The angles, measured from straight up: Loft 0, Right +90, Left -90.
///
/// History worth keeping: there used to be a chassis-relative Up and a world-relative Loft
/// sitting side by side, which were indistinguishable in level flight and diverged only
/// mid-bank — a distinction nobody could see when testing and everybody would trip over in
/// play. There was also a Down, which aimed at a point offset x range BELOW the target: tens
/// of metres underground at any normal range, so the missile flew into the road and exploded.
/// Its summary claimed it "hugs terrain". Both are gone.
/// </summary>
public enum MissileFlareMode
{
    /// <summary>Left, then right, then left. Best for volleys: they split around the target.</summary>
    Alternate,

    /// <summary>
    /// A random angle anywhere across the upper half — full left, through straight up, to full
    /// right, and every diagonal between. Continuous, not a pick between the other entries in
    /// this list, so no two shots take quite the same path.
    ///
    /// Deliberately never rolls below horizontal. The lower half is not an arc, it is a hole in
    /// the ground: the aim point ends up under the map and the missile detonates on the road.
    /// </summary>
    Random,

    /// <summary>Always out to the world left, level with the horizon.</summary>
    Left,

    /// <summary>Always out to the world right, level with the horizon.</summary>
    Right,

    /// <summary>
    /// Climbs away, tips over, and drops onto the target from above — missiles you see raining
    /// down rather than snaking across. Uses world up, so the arc stays vertical no matter how
    /// hard the launching craft was banked.
    /// </summary>
    Loft,
}
