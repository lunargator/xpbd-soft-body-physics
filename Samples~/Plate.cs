using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using SoftBodyPhysics.Unity;

namespace SoftBodyPhysics.Samples {

    [DefaultExecutionOrder(-200)]
    public sealed class Plate : MonoBehaviour {
        public const float PlateRadius = 1.8f;
        public const float PlateY = 0.08f;
        public const float Thickness = 0.07f;
        public const float SphereRadius = 0.22f;

        static readonly Color[] Colors = {
            new Color(0.93f, 0.34f, 0.28f),
            new Color(0.98f, 0.70f, 0.22f),
            new Color(0.84f, 0.22f, 0.34f),
            new Color(0.20f, 0.48f, 0.92f),
            new Color(0.22f, 0.72f, 0.46f),
            new Color(0.55f, 0.34f, 0.86f)
        };

        int builtEpoch = -1;
        int nextSphere;
        float orbitYaw;
        float orbitPitch;
        const float OrbitDistance = 6.4f;
        const float OrbitSpeed = 0.55f;

        void Update() {
            if(!Application.isPlaying) return;

            Application.targetFrameRate = 60;
            QualitySettings.vSyncCount = 0;
            if(builtEpoch != SamplePlay.Epoch) Build();
            SampleTechView.Ensure(this);
            if(Pressed(Key.D)) Drop(1);
            if(Pressed(Key.F)) Drop(3);
            if(Pressed(Key.G)) Drop(10);
            if(Pressed(Key.R)) ToggleSmooth();
        }

        void LateUpdate() {
            if(!Application.isPlaying) return;
            OrbitCamera();
        }

        void Build() {
            builtEpoch = SamplePlay.Epoch;
            SoftBodySimulation simulation = SoftBodySimulation.Ensure();
            simulation.Play(new Vector3(0f, -9.81f, 0f), 4, 1, 1f / 60f);
            simulation.Collision.ClearPlanes();
            simulation.Collision.ShellScale = 1.05f;
            BuildPlate();
            SoftBodyColliders.Invalidate();
            ClearPieces();
            nextSphere = 0;
            FrameCamera();
            SampleLights.EnableShadows();
        }

        void BuildPlate() {
            RemoveChild("Bowl");
            RemoveChild("Box");
            RemoveChild("Plate");

            const int sides = 48;
            float y0 = PlateY - Thickness;
            float y1 = PlateY;
            var vertices = new Vector3[2 + sides * 2];
            vertices[0] = new Vector3(0f, y1, 0f);
            vertices[1] = new Vector3(0f, y0, 0f);
            for (int i = 0; i < sides; i++) {
                float angle = i * Mathf.PI * 2f / sides;
                float x = Mathf.Cos(angle) * PlateRadius;
                float z = Mathf.Sin(angle) * PlateRadius;
                vertices[2 + i] = new Vector3(x, y1, z);
                vertices[2 + sides + i] = new Vector3(x, y0, z);
            }

            var triangles = new List<int>();
            for (int i = 0; i < sides; i++) {
                int next = (i + 1) % sides;
                int top = 2 + i;
                int topNext = 2 + next;
                int bottom = 2 + sides + i;
                int bottomNext = 2 + sides + next;
                triangles.Add(0);
                triangles.Add(top);
                triangles.Add(topNext);
                triangles.Add(1);
                triangles.Add(bottom);
                triangles.Add(bottomNext);
                triangles.Add(top);
                triangles.Add(bottomNext);
                triangles.Add(bottom);
                triangles.Add(top);
                triangles.Add(topNext);
                triangles.Add(bottomNext);
            }

            var mesh = new Mesh { name = "Plate", hideFlags = HideFlags.DontSave };
            mesh.vertices = vertices;
            mesh.SetTriangles(triangles, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            var plate = new GameObject("Plate");
            plate.transform.SetParent(transform, false);
            plate.AddComponent<MeshFilter>().sharedMesh = mesh;
            MeshRenderer renderer = plate.AddComponent<MeshRenderer>();
            renderer.sharedMaterial = SolidPaint.Create(new Color(0.80f, 0.88f, 0.95f, 1f), 0.55f);
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            BoxCollider box = plate.AddComponent<BoxCollider>();
            box.center = new Vector3(0f, PlateY - 0.15f, 0f);
            box.size = new Vector3(PlateRadius * 2f, 0.3f, PlateRadius * 2f);
        }

        void RemoveChild(string childName) {
            Transform existing = transform.Find(childName);
            if(existing == null) return;
            if(Application.isPlaying) Destroy(existing.gameObject);
            else DestroyImmediate(existing.gameObject);
        }

        void ClearPieces() {
            for (int i = transform.childCount - 1; i >= 0; i--) {
                Transform child = transform.GetChild(i);
                if(child.name == "Plate" || child.GetComponent<SampleTechView>() != null) continue;
                Destroy(child.gameObject);
            }
        }

        void Drop(int count) {
            float sky = PlateY + 4.2f;
            float reach = count <= 1 ? 0f : count <= 3 ? PlateRadius * 0.28f : PlateRadius * 0.48f;
            for (int i = 0; i < count; i++) {
                float angle = count <= 1 ? 0f : i * Mathf.PI * 2f / count + nextSphere * 0.2f;
                float height = sky + (count <= 3 ? i * 0.35f : (i % 2) * 0.28f);
                var position = new Vector3(Mathf.Cos(angle) * reach, height, Mathf.Sin(angle) * reach);
                SpawnSphere(nextSphere, position);
                nextSphere++;
            }
        }

        void SpawnSphere(int index, Vector3 position) {
            var host = new GameObject("Plate Sphere " + index);
            host.transform.SetParent(transform, false);
            host.transform.position = position;
            SoftBody body = host.AddComponent<SoftBody>();
            body.shape = SoftBody.ShapeKind.Sphere;
            body.radius = SphereRadius;
            body.sphereSegments = 6;
            body.sphereStacks = 4;
            body.mass = 1f;
            body.stiffness = 0.55f;
            body.damping = 0.45f;
            body.friction = 0.35f;
            body.restitution = 0.2f;
            body.surfaceOverride = true;
            body.gravityMultiplier = 1f;
            body.color = Colors[index % Colors.Length];
            body.smoothSurface = true;
            body.initializeOnEnable = false;
            body.resetOnEnable = true;
            body.Initialize();
        }

        void ToggleSmooth() {
            SoftBody[] bodies = GetComponentsInChildren<SoftBody>();
            if(bodies.Length == 0) return;
            bool next = !bodies[0].smoothSurface;
            for (int i = 0; i < bodies.Length; i++) bodies[i].smoothSurface = next;
        }

        static bool Pressed(Key key) {
            Keyboard keyboard = Keyboard.current;
            return keyboard != null && keyboard[key].wasPressedThisFrame;
        }

        void FrameCamera() {
            Camera camera = Camera.main;
            if(camera == null) return;
            var offset = new Vector3(1.6f, 5.6f, -3.4f);
            orbitYaw = Mathf.Atan2(offset.x, offset.z);
            orbitPitch = 15f * Mathf.Deg2Rad;
            camera.fieldOfView = 42f;
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.04f, 0.045f, 0.06f, 1f);
            PlaceCamera(camera);
        }

        void OrbitCamera() {
            Camera camera = Camera.main;
            if(camera == null) return;
            float yaw = 0.22f;
            float pitch = 0f;
            Keyboard keyboard = Keyboard.current;
            if(keyboard != null) {
                if(keyboard[Key.A].isPressed) yaw -= 1f;
                if(keyboard[Key.E].isPressed) yaw += 1f;
                if(keyboard[Key.W].isPressed) pitch += 1f;
                if(keyboard[Key.S].isPressed) pitch -= 1f;
            }

            orbitYaw += yaw * OrbitSpeed * Time.deltaTime;
            orbitPitch = Mathf.Clamp(orbitPitch + pitch * OrbitSpeed * Time.deltaTime, 5f * Mathf.Deg2Rad, 85f * Mathf.Deg2Rad);
            PlaceCamera(camera);
        }

        void PlaceCamera(Camera camera) {
            var center = new Vector3(0f, PlateY, 0f);
            float ring = Mathf.Cos(orbitPitch) * OrbitDistance;
            var position = center + new Vector3(Mathf.Sin(orbitYaw) * ring, Mathf.Sin(orbitPitch) * OrbitDistance, Mathf.Cos(orbitYaw) * ring);
            camera.transform.SetPositionAndRotation(position, Quaternion.LookRotation(center - position, Vector3.up));
        }
    }
}
