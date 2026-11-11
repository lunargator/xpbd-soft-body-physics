using UnityEditor;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace SoftBodyPhysics.Samples {
    public static class PlateSetup {
        public static string ScenePath => SampleSceneKit.SceneFile("Plate.unity");

        [MenuItem("Tools/Soft Body Physics/Create Plate")]
        public static void CreateFromMenu() { Create(force: true); }

        [InitializeOnLoadMethod]
        static void Schedule() { EditorApplication.delayCall += () => Create(force: false); }

        public static void Create(bool force) {
            if(SampleSceneKit.ShouldSkip(ScenePath, force)) return;

            Scene scene = SampleSceneKit.OpenEmpty(out Scene previous);
            Camera camera = SampleSceneKit.CreateCamera(new Vector3(1.6f, 1.74f, -6.0f), Quaternion.Euler(15f, 15f, 0f), 42f, 0.2f);
            camera.backgroundColor = new Color(0.04f, 0.045f, 0.06f, 1f);
            SampleSceneKit.CreateSun(1.6f, new Vector3(58f, -24f, 0f));
            SampleSceneKit.CreateSimulation(new Vector3(0f, -9.81f, 0f));
            new GameObject("Plate").AddComponent<Plate>();
            SampleSceneKit.Save(scene, previous, ScenePath);
        }
    }
}
