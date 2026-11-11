using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using SoftBodyPhysics.Unity;

namespace SoftBodyPhysics.Samples {

    public static class SampleSceneKit {
        public static string SamplesRoot() {
            string[] guids = AssetDatabase.FindAssets("SoftBodyBowl t:MonoScript");
            for (int i = 0; i < guids.Length; i++) {
                string path = AssetDatabase.GUIDToAssetPath(guids[i]);
                if(path.EndsWith("SoftBodyBowl.cs")) return Path.GetDirectoryName(path).Replace('\\', '/');
            }

            return "Assets/SoftBodyPhysicsSamples";
        }

        public static string SceneFile(string fileName) { return SamplesRoot() + "/Scenes/" + fileName; }

        public static string AbsolutePath(string assetPath) {
            if(string.IsNullOrEmpty(assetPath)) return string.Empty;
            if(assetPath.StartsWith("Assets/")) return Path.Combine(Application.dataPath, assetPath.Substring("Assets/".Length));
            return Path.GetFullPath(assetPath);
        }

        public static bool SceneFileExists(string assetPath) { return File.Exists(AbsolutePath(assetPath)); }

        public static void PrepareSceneFolder() { Directory.CreateDirectory(AbsolutePath(SamplesRoot() + "/Scenes")); }

        public static void AttachUrpCamera(GameObject cameraObject) {
            if(cameraObject == null) return;
            System.Type extra = System.Type.GetType(
                "UnityEngine.Rendering.Universal.UniversalAdditionalCameraData, Unity.RenderPipelines.Universal.Runtime");
            if(extra != null && cameraObject.GetComponent(extra) == null) cameraObject.AddComponent(extra);
        }

        public static bool ShouldSkip(string scenePath, bool force) {
            return EditorApplication.isPlayingOrWillChangePlaymode || (!force && SceneFileExists(scenePath));
        }

        public static Scene OpenEmpty(out Scene previous) {
            previous = SceneManager.GetActiveScene();
            Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Additive);
            EditorSceneManager.SetActiveScene(scene);
            return scene;
        }

        public static void Save(Scene scene, Scene previous, string scenePath) {
            PrepareSceneFolder();
            EditorSceneManager.SaveScene(scene, scenePath);
            Close(scene, previous);
        }

        public static void Close(Scene scene, Scene previous) {
            EditorSceneManager.CloseScene(scene, true);
            if(previous.IsValid()) EditorSceneManager.SetActiveScene(previous);
        }

        public static Camera CreateCamera(Vector3 position, Quaternion rotation, float fieldOfView, float nearClip) {
            var cameraObject = new GameObject("Main Camera");
            Camera camera = cameraObject.AddComponent<Camera>();
            cameraObject.tag = "MainCamera";
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.55f, 0.68f, 0.78f, 1f);
            camera.fieldOfView = fieldOfView;
            camera.nearClipPlane = nearClip;
            camera.transform.SetPositionAndRotation(position, rotation);
            cameraObject.AddComponent<AudioListener>();
            AttachUrpCamera(cameraObject);
            return camera;
        }

        public static Light CreateSun(float intensity, Vector3 euler) {
            var lightObject = new GameObject("Directional Light");
            Light light = lightObject.AddComponent<Light>();
            light.type = LightType.Directional;
            light.intensity = intensity;
            light.shadows = LightShadows.Soft;
            lightObject.transform.rotation = Quaternion.Euler(euler);
            return light;
        }

        public static SoftBodySimulation CreateSimulation(Vector3 gravity, int iterations = 4) {
            var host = new GameObject("Soft Body Simulation");
            SoftBodySimulation simulation = host.AddComponent<SoftBodySimulation>();
            simulation.gravity = gravity;
            simulation.solverIterations = iterations;
            simulation.substeps = 1;
            simulation.fixedDeltaTime = 1f / 60f;
            return simulation;
        }

        public static GameObject CreateBox(string name, Vector3 position, Vector3 scale, Material material, Transform parent) {
            var box = GameObject.CreatePrimitive(PrimitiveType.Cube);
            box.name = name;
            if(parent != null) box.transform.SetParent(parent, false);
            box.transform.SetPositionAndRotation(position, Quaternion.identity);
            box.transform.localScale = scale;
            if(material == null) material = StageMaterial(name);
            if(material != null) box.GetComponent<MeshRenderer>().sharedMaterial = material;
            return box;
        }

        public static Material StageMaterial(string objectName) {
            string file = "StageFloor.mat";
            if(!string.IsNullOrEmpty(objectName)) {
                if(objectName.StartsWith("Wall")) file = "StageWall.mat";
                else if(objectName.StartsWith("Launch") || objectName == "Buncher") file = "StageBoard.mat";
                else if(objectName.StartsWith("Spin")) file = "StageSpin.mat";
            }

            return AssetDatabase.LoadAssetAtPath<Material>(SamplesRoot() + "/Resources/" + file);
        }
    }
}
