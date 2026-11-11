using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace SoftBodyPhysics.Samples {
    public static class SoftBodyPresetDropSetup {
        public static string ScenePath => SampleSceneKit.SceneFile("Material Comp.unity");

        [MenuItem("Tools/Soft Body Physics/Create Material Comp")]
        public static void CreateFromMenu() { Create(force: true); }

        [InitializeOnLoadMethod]
        static void Schedule() { EditorApplication.delayCall += () => Create(force: false); }

        public static void Create(bool force) {
            if(SampleSceneKit.ShouldSkip(ScenePath, force)) return;

            Scene scene = SampleSceneKit.OpenEmpty(out Scene previous);
            Camera camera = SampleSceneKit.CreateCamera(new Vector3(0f, 3.12f, -8.46f), Quaternion.Euler(14f, 0f, 0f), 40f, 0.1f);
            camera.backgroundColor = new Color(0.04f, 0.045f, 0.06f, 1f);
            SampleSceneKit.CreateSun(1.5f, new Vector3(50f, -26f, 0f));
            SampleSceneKit.CreateSimulation(new Vector3(0f, -9.81f, 0f));
            SampleSceneKit.CreateBox("Floor", new Vector3(0f, -0.15f, 0f), new Vector3(10f, 0.3f, 6f), null, null);
            new GameObject("Material Comp").AddComponent<SoftBodyPresetDrop>();
            SampleSceneKit.Save(scene, previous, ScenePath);
        }
    }
}
