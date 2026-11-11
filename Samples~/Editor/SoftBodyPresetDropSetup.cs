using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace SoftBodyPhysics.Samples {
    public static class SoftBodyPresetDropSetup {
        public static string ScenePath => SampleSceneKit.SceneFile("SoftBodyPresetDrop.unity");

        [MenuItem("Tools/Soft Body Physics/Create Preset Drop")]
        public static void CreateFromMenu() { Create(force: true); }

        [InitializeOnLoadMethod]
        static void Schedule() { EditorApplication.delayCall += () => Create(force: false); }

        public static void Create(bool force) {
            if(SampleSceneKit.ShouldSkip(ScenePath, force)) return;

            Scene scene = SampleSceneKit.OpenEmpty(out Scene previous);
            SampleSceneKit.CreateCamera(new Vector3(0f, 2.8f, -7.2f), Quaternion.Euler(14f, 0f, 0f), 40f, 0.1f);
            SampleSceneKit.CreateSun(1.5f, new Vector3(50f, -26f, 0f));
            SampleSceneKit.CreateSimulation(new Vector3(0f, -9.81f, 0f));
            SampleSceneKit.CreateBox("Floor", new Vector3(0f, -0.15f, 0f), new Vector3(10f, 0.3f, 6f), null, null);
            new GameObject("Preset Drop").AddComponent<SoftBodyPresetDrop>();
            SampleSceneKit.Save(scene, previous, ScenePath);
        }
    }
}
