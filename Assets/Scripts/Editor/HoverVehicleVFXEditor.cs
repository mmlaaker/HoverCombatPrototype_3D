using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(HoverVehicleVFX))]
public class HoverVehicleVFXEditor : Editor
{
    private static readonly string[] tierLabels = { "Idle", "Acceleration", "Boost" };

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        EditorGUILayout.Space(8);
        EditorGUILayout.LabelField("Preview Tier", EditorStyles.boldLabel);

        var vfx = (HoverVehicleVFX)target;

        EditorGUILayout.BeginHorizontal();
        for (int i = 0; i < tierLabels.Length; i++)
        {
            if (GUILayout.Button(tierLabels[i]))
                vfx.EditorPreviewTier(i);
        }
        EditorGUILayout.EndHorizontal();
    }
}
