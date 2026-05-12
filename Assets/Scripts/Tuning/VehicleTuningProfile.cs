using UnityEngine;

/// <summary>
/// VehicleTuningProfile v1.0
///
/// Shared per-archetype tuning asset for the four hover controllers
/// (Foundation, Propulsion, Energy, VehicleHealth). One asset = one
/// vehicle archetype's handling profile. Scene references and runtime
/// state stay on the MonoBehaviours.
///
/// Create via: Assets > Create > Hover > Vehicle Tuning Profile.
/// Suggested location: Assets/Data/Tuning/VTP_*.asset.
/// </summary>
[CreateAssetMenu(fileName = "VTP_NewVehicle", menuName = "Hover/Vehicle Tuning Profile")]
public class VehicleTuningProfile : ScriptableObject
{
    public FoundationTuning foundation = new FoundationTuning();
    public PropulsionTuning propulsion = new PropulsionTuning();
    public EnergyTuning     energy     = new EnergyTuning();
    public HealthTuning     health     = new HealthTuning();
    public ShieldTuning     shield     = new ShieldTuning();
    public EmpTuning        emp        = new EmpTuning();
}
