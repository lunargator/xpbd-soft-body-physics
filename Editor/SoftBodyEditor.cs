using SoftBodyPhysics.Unity;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

// what to show in inspector
namespace SoftBodyPhysics.Editor {
    [CustomEditor(typeof(SoftBody))]
    public class SoftBodyEditor : UnityEditor.Editor {

        public override void OnInspectorGUI() {
            serializedObject.Update();
            SoftBody body = (SoftBody) target;
            DrawIssues(body);
            if(!Application.isPlaying) EditorGUILayout.HelpBox("Press Play to build this body. Mutable stats are on Soft Body Simulation.", MessageType.Info);

            if(Fold("Shape", true)) DrawShape(body);
            if(Fold("Material", true)) DrawMaterial(body);
            if(Fold("Simulation", true)) {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("mass"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("simulate"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("kinematic"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("gravityMultiplier"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("resetOnEnable"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("initializeOnEnable"));
            }

            if(Fold("Rendering", false)) {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("color"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("renderInterval"));
            }

            if(Fold("Debug", false)) {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("showParticles"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("showConstraints"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("showCollisionBounds"));
                if(body.Body != null) {
                    EditorGUILayout.LabelField("Live particles", body.Body.Particles.Length.ToString());
                    EditorGUILayout.LabelField("Constraints", body.Body.Constraints.Count.ToString());
                    EditorGUILayout.LabelField("Closed", body.Body.ClosedSurface ? "Yes" : "No");
                } else { EditorGUILayout.HelpBox("Turn on Particles or Constraints to draw the rest pose. Preview builds the live surface.", MessageType.None); }
            }

            serializedObject.ApplyModifiedProperties();
        }

        void DrawShape(SoftBody body) {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("shape"));
            SoftBody.ShapeKind shape = (SoftBody.ShapeKind)serializedObject.FindProperty("shape").enumValueIndex;
            if(shape == SoftBody.ShapeKind.Sphere) {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("radius"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("sphereSegments"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("sphereStacks"));
            } else if(shape == SoftBody.ShapeKind.Cube) {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("cubeSize"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("cubeCells"));
            } else {
                EditorGUILayout.PropertyField(serializedObject.FindProperty("sourceMesh"));
                EditorGUILayout.PropertyField(serializedObject.FindProperty("meshCellSize"));
            }

            EditorGUILayout.PropertyField(serializedObject.FindProperty("smoothSurface"));
            serializedObject.ApplyModifiedProperties();
            EditorGUILayout.LabelField("Particles", body.PreviewParticleCount().ToString());
            if(shape == SoftBody.ShapeKind.Mesh && body.HasBakedMesh()) EditorGUILayout.LabelField("Saved mesh", "Stored on this component.");
            if(GUILayout.Button(shape == SoftBody.ShapeKind.Mesh ? "Generate" : "Preview")) Generate(body);
        }

        void DrawMaterial(SoftBody body) {
            EditorGUI.BeginChangeCheck();
            EditorGUILayout.PropertyField(serializedObject.FindProperty("preset"));
            if(EditorGUI.EndChangeCheck()) {
                serializedObject.ApplyModifiedProperties();
                Undo.RecordObject(body, "Apply soft body preset");
                body.ApplyPreset(body.preset);
                EditorUtility.SetDirty(body);
                serializedObject.Update();
            }

            bool locked = body.preset != SoftBodyPreset.Custom;
            EditorGUI.BeginDisabledGroup(locked);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("stiffness"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("damping"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("friction"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("restitution"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("surfaceOverride"));
            EditorGUI.EndDisabledGroup();
            if(locked) EditorGUILayout.HelpBox("Choose Custom to edit these numbers.", MessageType.None);
            else if(!serializedObject.FindProperty("surfaceOverride").boolValue) EditorGUILayout.HelpBox("Friction and bounce stay at the material default until Surface Override is on.", MessageType.None);
        }

        static void DrawIssues(SoftBody body) {
            var issues = new List<string>();
            body.CollectIssues(issues);
            for (int i = 0; i < issues.Count; i++) EditorGUILayout.HelpBox(issues[i], MessageType.Warning);
        }

        static void Generate(SoftBody body) {
            Undo.RecordObject(body, "Generate soft body");
            try {
                if(body.shape == SoftBody.ShapeKind.Mesh) body.BakeMesh();
                else body.Initialize();
                EditorUtility.SetDirty(body);
            } catch (System.Exception error) { EditorUtility.DisplayDialog("Generate", error.Message, "OK"); }
        }

        static bool Fold(string name, bool startOpen) {
            string key = "SoftBodyPhysics.SoftBody." + name;
            bool open = EditorPrefs.GetBool(key, startOpen);
            bool next = EditorGUILayout.Foldout(open, name, true);
            if(next != open) EditorPrefs.SetBool(key, next);
            return next;
        }

        [MenuItem("Tools/Soft Body Physics/Validate Soft Bodies")]
        static void ValidateScene() {
            SoftBody[] bodies = Object.FindObjectsByType<SoftBody>(FindObjectsInactive.Include);
            var issues = new List<string>();
            int count = 0;
            for (int i = 0; i < bodies.Length; i++) {
                issues.Clear();
                bodies[i].CollectIssues(issues);
                for (int n = 0; n < issues.Count; n++) {
                    Debug.LogWarning(bodies[i].name + ": " + issues[n], bodies[i]);
                    count++;
                }
            }

            if(count == 0) Debug.Log("Soft bodies: " + bodies.Length + " checked, no issues.");
        }
    }

    [CustomEditor(typeof(SoftBodySimulation))]
    public class SoftBodySimulationEditor : UnityEditor.Editor {
        public override void OnInspectorGUI() {
            serializedObject.Update();
            EditorGUILayout.LabelField("Simulation", EditorStyles.boldLabel);
            EditorGUILayout.PropertyField(serializedObject.FindProperty("gravity"));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("solverIterations"), new GUIContent("Solver Iterations", "Constraint iterations inside one fixed step."));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("substeps"), new GUIContent("Max Steps Per Frame", "Most fixed steps one rendered frame may run."));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("fixedDeltaTime"), new GUIContent("Fixed Timestep", "Seconds per solver step. Separate from the render rate."));
            EditorGUILayout.PropertyField(serializedObject.FindProperty("colliderRoot"), new GUIContent("Collider Root", "Only colliders under this object are copied. Empty uses the cached scene list."));
            serializedObject.ApplyModifiedProperties();
            EditorGUILayout.HelpBox("The render rate is the player's frame rate. A frame longer than timestep × max steps drops the extra time. A collider created or destroyed during play is copied after SoftBodyColliders.Invalidate.", MessageType.None);

            var simulation = (SoftBodySimulation)target;
            if(!Application.isPlaying) {
                EditorGUILayout.HelpBox("Play mode steps every soft body in this world.", MessageType.None);
                return;
            }

            EditorGUILayout.LabelField("Bodies", simulation.World.Bodies.Count.ToString());
            EditorGUILayout.LabelField("Dropped This Frame", simulation.LastDroppedTime.ToString("0.000") + " s");
            EditorGUILayout.LabelField("Dropped Total", simulation.DroppedTime.ToString("0.000") + " s");
            EditorGUILayout.LabelField("Collider Sync", simulation.LastSyncMilliseconds.ToString("0.000") + " ms");
            EditorGUILayout.LabelField("Mesh Vertices", simulation.LastVertexMilliseconds.ToString("0.000") + " ms");
            EditorGUILayout.LabelField("Mesh Normals", simulation.LastNormalMilliseconds.ToString("0.000") + " ms");
            EditorGUILayout.LabelField("Mesh Upload", simulation.LastUploadMilliseconds.ToString("0.000") + " ms");
            EditorGUILayout.LabelField("Mesh Topology", simulation.LastTopologyMilliseconds.ToString("0.000") + " ms");
            if(simulation.Paused) {
                if(GUILayout.Button("Resume")) simulation.Resume();
            } else if(GUILayout.Button("Pause")) { simulation.Pause(); }
        }
    }
}
