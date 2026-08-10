using System;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Inspector for HoverCameraController. Adds the definitive state preview and
/// the framing verdict on top of the normal fields.
///
/// Why this exists: you cannot drag a slider while both hands are on a
/// controller, so any framing that only appears while an input is held was
/// previously tuned by guessing between drives. Pick a state here, tune with
/// the mouse, watch the Game view.
///
/// The verdict is the other half. A screenshot tells you the framing looks
/// wrong; the readout tells you by how many degrees and in which direction, so
/// a fix can be aimed rather than hunted.
///
/// Deliberate deviation from VehicleTuningProfileEditor: that editor walks each
/// nested section's children by hand (Copy / GetEndProperty / NextVisible)
/// because it interleaves derived readouts after specific fields. Nothing here
/// needs interleaving, so the fields are drawn by a plain top-level iteration,
/// which cannot drop a field added to a Camera*Tuning class later either. The
/// free-heading convention is unaffected: [Header] attributes inside the
/// sections still draw with their fields.
/// </summary>
[CustomEditor(typeof(HoverCameraController))]
public class HoverCameraControllerEditor : Editor
{
    /// <summary>Drawn by hand inside the preview block, so skipped in the field pass.</summary>
    private static readonly string[] HandledManually = { "previewInEditMode", "previewState" };

    private static bool _sweepExpanded;

    private SerializedProperty _previewEnabled;
    private SerializedProperty _previewState;

    private void OnEnable()
    {
        _previewEnabled = serializedObject.FindProperty("previewInEditMode");
        _previewState   = serializedObject.FindProperty("previewState");
    }

    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        DrawPreviewBlock();

        EditorGUILayout.Space(6);

        SerializedProperty it = serializedObject.GetIterator();
        bool enterChildren = true;

        while (it.NextVisible(enterChildren))
        {
            enterChildren = false;

            if (it.propertyPath == "m_Script")
            {
                using (new EditorGUI.DisabledScope(true))
                    EditorGUILayout.PropertyField(it);
                continue;
            }

            if (Array.IndexOf(HandledManually, it.propertyPath) >= 0) continue;

            EditorGUILayout.PropertyField(it, true);
        }

        serializedObject.ApplyModifiedProperties();
    }

    // ── Preview block ─────────────────────────────────────────────────────

    private void DrawPreviewBlock()
    {
        EditorGUILayout.LabelField("Definitive State Preview", EditorStyles.boldLabel);

        if (Application.isPlaying)
        {
            EditorGUILayout.HelpBox(
                "Inert during play. The live camera solves from real input, and this pane would " +
                "only be able to lie about it. Stop the game to preview states.",
                MessageType.None);
            return;
        }

        // Multi-select would have to pick one component's tuning to measure and
        // then report it as though it described all of them.
        if (targets.Length > 1)
        {
            EditorGUILayout.HelpBox("Select a single camera controller to preview states.",
                                    MessageType.None);
            return;
        }

        EditorGUILayout.PropertyField(_previewEnabled);

        if (!_previewEnabled.boolValue)
        {
            EditorGUILayout.HelpBox(
                "Preview is off, so the vcams are left alone and you can compose on them directly, " +
                "then adopt the result with the Pull Framing From VCams context menu.",
                MessageType.None);
            return;
        }

        DrawStateSelector();

        var cam = (HoverCameraController)target;
        DrawVerdict(cam, (CameraPreviewState)_previewState.enumValueIndex);
        DrawSweep(cam);
    }

    private void DrawStateSelector()
    {
        var    states = HoverCameraController.AllPreviewStates;
        string[] names = new string[states.Length];

        for (int i = 0; i < states.Length; i++)
            names[i] = ObjectNames.NicifyVariableName(states[i].ToString());

        EditorGUI.BeginChangeCheck();
        int picked = GUILayout.SelectionGrid(_previewState.enumValueIndex, names, 3);

        if (EditorGUI.EndChangeCheck())
        {
            _previewState.enumValueIndex = picked;
            serializedObject.ApplyModifiedProperties();
            RequestViewRefresh();
        }
    }

    private static void DrawVerdict(HoverCameraController cam, CameraPreviewState state)
    {
        HoverCameraController.FramingVerdict v = cam.EvaluateState(state, MainGameViewAspect());

        DerivedBox(
            "Camera offset",  v.followOffset.ToString("F2"),
            "Look point",     v.targetOffset.ToString("F2"),
            "Field of view",  $"{v.fov:F1} deg vertical",
            "Elevation",      $"{v.elevation:F1} deg   (neutral {cam.NeutralElevation:F1})",
            "Distance",       $"{v.orbitRadius:F2} m",
            "Craft off axis", $"{Mathf.Abs(v.verticalOffAxis):F1} deg " +
                              (v.verticalOffAxis >= 0f ? "below" : "above") +
                              $", {Mathf.Abs(v.horizontalOffAxis):F1} deg " +
                              (v.horizontalOffAxis >= 0f ? "right" : "left"),
            "Framing guard",  v.guardScale >= 0.999f
                                ? "not needed"
                                : $"ACTIVE, look-ahead cut to {v.guardScale * 100f:F0}% " +
                                  $"({v.guardRemoved:F2} m removed)");

        float over = -Mathf.Min(v.verticalMargin, v.horizontalMargin);
        string axis = v.verticalMargin < v.horizontalMargin ? "top or bottom" : "sides";

        if (v.Fits)
        {
            string note = v.guardScale >= 0.999f
                ? $"Whole craft in frame, {Mathf.Min(v.verticalMargin, v.horizontalMargin):F1} deg to spare."
                : $"Whole craft in frame, but only because the guard cut look-ahead to " +
                  $"{v.guardScale * 100f:F0}%. Your authored look-ahead does not fit this state. " +
                  "Lower Pitch Up Range, weaken the look-ahead, or boom out with Pitch Up Distance " +
                  "Gain if you want the full amount back.";

            EditorGUILayout.HelpBox(note, MessageType.Info);
            return;
        }

        // Cropped but centred is ordinary framing, not a fault. Saying otherwise
        // would flag nearly every state and train you to ignore this box.
        if (v.CentreVisible)
        {
            EditorGUILayout.HelpBox(
                $"Craft is centred, with {over:F1} deg clipped at the {axis}. Normal at close " +
                "range, and expected in strafe where the camera pulls in. Only a problem if you " +
                "can see it.",
                MessageType.None);
            return;
        }

        EditorGUILayout.HelpBox(
            $"Craft is LEAVING THE FRAME. Its centre sits {Mathf.Abs(v.verticalOffAxis):F1} deg " +
            (v.verticalOffAxis >= 0f ? "below" : "above") +
            $" the look axis against a {v.halfFovV:F1} deg half-frame.\n\n" +
            "The craft is at the centre of the orbit, so the angle down to it is always exactly " +
            "the elevation angle, and distance cannot change that. Only tipping the look axis buys " +
            "it back, so Pitch Up Range and Pitch Look Ahead are competing for one budget. Lower " +
            "the ceiling, strengthen the look-ahead, or boom out with Pitch Up Distance Gain.",
            MessageType.Warning);
    }

    private static void DrawSweep(HoverCameraController cam)
    {
        _sweepExpanded = EditorGUILayout.Foldout(_sweepExpanded, "All states", true);

        if (!_sweepExpanded) return;

        float aspect = MainGameViewAspect();

        using (new EditorGUI.DisabledScope(true))
        using (new EditorGUILayout.VerticalScope(EditorStyles.helpBox))
        {
            foreach (CameraPreviewState s in HoverCameraController.AllPreviewStates)
            {
                HoverCameraController.FramingVerdict v = cam.EvaluateState(s, aspect);
                float margin = Mathf.Min(v.verticalMargin, v.horizontalMargin);

                string guard   = v.guardScale >= 0.999f ? "" : $"  [guard {v.guardScale * 100f:F0}%]";
                string verdict = (v.Fits          ? $"clear, {margin:F1} deg spare"
                               :  v.CentreVisible ? $"centred, {-margin:F1} deg clipped"
                                                  : $"LEAVING FRAME, centre {Mathf.Abs(v.verticalOffAxis):F1} " +
                                                    $"vs {v.halfFovV:F1} deg") + guard;

                EditorGUILayout.LabelField(ObjectNames.NicifyVariableName(s.ToString()), verdict);
            }
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────

    /// <summary>
    /// Aspect of whatever the Game view is currently rendering, for the
    /// horizontal half of the fit test. Read off the live camera rather than a
    /// hardcoded 16:9, so the verdict matches the picture on screen. Falls back
    /// only when there is no camera to ask.
    /// </summary>
    private static float MainGameViewAspect()
    {
        Camera cam = Camera.main;
        return cam != null && cam.aspect > 0.01f ? cam.aspect : 16f / 9f;
    }

    /// <summary>
    /// Nudges the editor into ticking and redrawing.
    ///
    /// Needed because the Game view renders from the camera's PREVIOUS
    /// transform until the brain has consumed the new follow offset. Without a
    /// tick, switching state appears to do nothing and then silently catches up
    /// on the next unrelated repaint.
    /// </summary>
    private static void RequestViewRefresh()
    {
        EditorApplication.QueuePlayerLoopUpdate();
        SceneView.RepaintAll();
    }

    /// <summary>
    /// Dimmed label/value block, matching VehicleTuningProfileEditor's derived
    /// readouts so the two inspectors read the same way. Arguments are label,
    /// value, label, value, and so on.
    /// </summary>
    private static void DerivedBox(params string[] labelValuePairs)
    {
        using (new EditorGUILayout.VerticalScope(EditorStyles.helpBox))
        using (new EditorGUI.DisabledScope(true))
        {
            for (int i = 0; i + 1 < labelValuePairs.Length; i += 2)
                EditorGUILayout.LabelField(labelValuePairs[i], labelValuePairs[i + 1]);
        }
    }
}
