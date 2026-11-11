using UnityEngine;
using UnityEngine.InputSystem;
using SoftBodyPhysics.Unity;

namespace SoftBodyPhysics.Samples {

    [DefaultExecutionOrder(-200)]
    public sealed class SoftBodyPresetDrop : MonoBehaviour {
        public const float SquishScale = 1.4f;

        static readonly SoftBodyPreset[] Presets = {
            SoftBodyPreset.FirmElastic,
            SoftBodyPreset.Rubber,
            SoftBodyPreset.Gelatin,
            SoftBodyPreset.Jelly,
            SoftBodyPreset.Slime
        };

        static readonly string[] Names = { "Firm", "Rubber", "Gelatin", "Jelly", "Slime" };

        static readonly Color[] Colors = {
            new Color(0.82f, 0.86f, 0.90f),
            new Color(0.86f, 0.18f, 0.20f),
            new Color(0.98f, 0.62f, 0.74f),
            new Color(0.45f, 0.88f, 0.38f),
            new Color(0.42f, 0.22f, 0.72f)
        };

        int builtEpoch = -1;
        bool slow;

        void OnDisable() { if(Time.timeScale != 1f) Time.timeScale = 1f; }

        void Update() {
            if(!Application.isPlaying) return;
            Application.targetFrameRate = 60;
            QualitySettings.vSyncCount = 0;
            SamplePaint.ShowSolids();
            if(builtEpoch != SamplePlay.Epoch) Build();
            if(Pressed(Key.F)) slow = !slow;
            Time.timeScale = slow ? 0.25f : 1f;
            if(Pressed(Key.R)) Drop();
            SampleTechView.Ensure(this);
        }

        static bool Pressed(Key key) {
            Keyboard keyboard = Keyboard.current;
            return keyboard != null && keyboard[key].wasPressedThisFrame;
        }

        void Build() {
            builtEpoch = SamplePlay.Epoch;
            SamplePaint.ShowSolids();
            SoftBodySimulation simulation = SoftBodySimulation.Ensure();
            simulation.Play(new Vector3(0f, -9.81f, 0f), 4, 1, 1f / 60f);
            simulation.Collision.ClearPlanes();
            simulation.Collision.ShellScale = SquishScale;
            Camera camera = Camera.main;
            if(camera != null) {
                camera.clearFlags = CameraClearFlags.SolidColor;
                camera.backgroundColor = new Color(0.04f, 0.045f, 0.06f, 1f);
            }

            Drop();
            SampleLights.EnableShadows();
        }

        void Drop() {
            ClearChildren();
            for (int i = 0; i < Presets.Length; i++) Spawn(i);
            SpawnCube();
        }

        void ClearChildren() {
            for (int i = transform.childCount - 1; i >= 0; i--) {
                Transform child = transform.GetChild(i);
                if(child.GetComponent<SampleTechView>() != null) continue;
                Destroy(child.gameObject);
            }
        }

        void Spawn(int index) {
            float x = (index - 2) * 1.15f;
            var host = new GameObject(Names[index]);
            host.transform.SetParent(transform, false);
            host.transform.position = new Vector3(x, 4.2f, 0f);
            SoftBody body = host.AddComponent<SoftBody>();
            body.shape = SoftBody.ShapeKind.Sphere;
            body.radius = 0.42f;
            body.sphereSegments = 10;
            body.sphereStacks = 6;
            body.mass = 1f;
            body.ApplyPreset(Presets[index]);
            body.color = Colors[index];
            body.smoothSurface = true;
            body.initializeOnEnable = false;
            body.resetOnEnable = false;
            body.Initialize();
        }

        void SpawnCube() {
            var host = new GameObject("Jelly Cube");
            host.transform.SetParent(transform, false);
            host.transform.position = new Vector3(0f, 4.2f, 1.35f);
            SoftBody body = host.AddComponent<SoftBody>();
            body.shape = SoftBody.ShapeKind.Cube;
            body.cubeSize = 0.72f;
            body.cubeCells = 2;
            body.mass = 1f;
            body.ApplyPreset(SoftBodyPreset.Jelly);
            body.color = new Color(0.98f, 0.82f, 0.30f);
            body.smoothSurface = true;
            body.initializeOnEnable = false;
            body.resetOnEnable = false;
            body.Initialize();
        }
    }
}
