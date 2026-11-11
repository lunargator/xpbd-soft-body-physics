using System.Collections.Generic;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using UnityEngine;
using UnityEngine.InputSystem;
using SoftBodyPhysics.Unity;

namespace SoftBodyPhysics.Samples {

    public sealed class PhysicsDebugRenderer : MonoBehaviour {
        public const float ParticleSize = 0.045f;
        public const float CenterSize = 0.2f;
        public const float ContactSize = 0.06f;
        public const float NormalLength = 0.3f;

        public bool ShowParticles = true;
        public bool ShowDistanceConstraints;
        public bool ShowCenterOfMass = true;
        public bool ShowContacts = true;
        public bool ShowContactNormals = true;
        public bool ShowWireframe = true;
        public bool ShowKeyList = true;

        readonly List<Vector3> lineVertices = new List<Vector3>();
        readonly List<Color> lineColors = new List<Color>();
        readonly List<int> lineIndices = new List<int>();
        readonly HashSet<long> edges = new HashSet<long>();

        Mesh lineMesh;
        Material lineMaterial;

        static readonly Color ParticleColor = new Color(0.92f, 0.95f, 1f, 1f);
        static readonly Color DistanceColor = new Color(0.35f, 0.9f, 0.45f, 1f);
        static readonly Color CenterColor = new Color(1f, 0.84f, 0.2f, 1f);
        static readonly Color ContactColor = new Color(1f, 0.32f, 0.38f, 1f);
        static readonly Color NormalColor = new Color(1f, 0.55f, 0.2f, 1f);
        static readonly Color WireColor = new Color(0.65f, 0.82f, 1f, 0.85f);

        public Mesh LineMesh => lineMesh;

        public void Publish(PhysicsWorld world) {
            EnsureMesh();
            lineVertices.Clear();
            lineColors.Clear();
            lineIndices.Clear();
            if(world == null) {
                Upload();
                return;
            }

            IReadOnlyList<DeformableBody> bodies = world.Bodies;
            for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++) {
                DeformableBody body = bodies[bodyIndex];
                if(body == null) continue;
                if(ShowParticles) DrawParticles(body);
                if(ShowDistanceConstraints) DrawDistances(body);
                if(ShowCenterOfMass) AddCross(Local(body.CenterOfMass()), CenterSize, CenterColor);
                if(ShowWireframe) DrawWireframe(body);
            }

            if(ShowContacts || ShowContactNormals) DrawContacts(world);

            Upload();
        }

        void Update() {
            if(!Application.isPlaying) return;
            Keyboard keyboard = Keyboard.current;
            if(keyboard == null) return;
            if(Pressed(keyboard, Key.Digit1)) ShowParticles = !ShowParticles;
            if(Pressed(keyboard, Key.Digit2)) ShowDistanceConstraints = !ShowDistanceConstraints;
            if(Pressed(keyboard, Key.Digit3)) ShowCenterOfMass = !ShowCenterOfMass;
            if(Pressed(keyboard, Key.Digit4)) ShowContacts = !ShowContacts;
            if(Pressed(keyboard, Key.Digit5)) ShowContactNormals = !ShowContactNormals;
            if(Pressed(keyboard, Key.Digit6)) ShowWireframe = !ShowWireframe;
        }

        void OnGUI() {
            if(!Application.isPlaying || !ShowKeyList) return;

            const float width = 230f;
            GUI.Box(new Rect(12f, 12f, width, 148f), "Physics debug");
            GUI.Label(new Rect(24f, 36f, width - 24f, 18f), ToggleLabel("1 particles", ShowParticles));
            GUI.Label(new Rect(24f, 54f, width - 24f, 18f), ToggleLabel("2 distances", ShowDistanceConstraints));
            GUI.Label(new Rect(24f, 72f, width - 24f, 18f), ToggleLabel("3 center", ShowCenterOfMass));
            GUI.Label(new Rect(24f, 90f, width - 24f, 18f), ToggleLabel("4 contacts", ShowContacts));
            GUI.Label(new Rect(24f, 108f, width - 24f, 18f), ToggleLabel("5 normals", ShowContactNormals));
            GUI.Label(new Rect(24f, 126f, width - 24f, 18f), ToggleLabel("6 wireframe", ShowWireframe));
        }

        void DrawParticles(DeformableBody body) {
            Particle[] particles = body.Particles;
            for (int i = 0; i < particles.Length; i++) AddCross(Local(particles[i].Position), ParticleSize, ParticleColor);
        }

        void DrawDistances(DeformableBody body) {
            List<IConstraint> constraints = body.Constraints;
            Particle[] particles = body.Particles;
            for (int i = 0; i < constraints.Count; i++) {
                if(!(constraints[i] is DistanceConstraint distance)) continue;
                if(distance.IndexA < 0 || distance.IndexB < 0 || distance.IndexA >= particles.Length || distance.IndexB >= particles.Length)
                    continue;
                AddLine(Local(particles[distance.IndexA].Position), Local(particles[distance.IndexB].Position), DistanceColor);
            }
        }

        void DrawWireframe(DeformableBody body) {
            int[] triangles = body.Triangles;
            if(triangles == null) return;
            Particle[] particles = body.Particles;
            edges.Clear();
            for (int slot = 0; slot + 2 < triangles.Length; slot += 3) {
                DrawEdge(particles, triangles[slot], triangles[slot + 1]);
                DrawEdge(particles, triangles[slot + 1], triangles[slot + 2]);
                DrawEdge(particles, triangles[slot + 2], triangles[slot]);
            }
        }

        void DrawEdge(Particle[] particles, int a, int b) {
            if(a == b || a < 0 || b < 0 || a >= particles.Length || b >= particles.Length) return;
            int lo = a < b ? a : b;
            int hi = a < b ? b : a;
            if(!edges.Add(((long)lo << 32) | (uint)hi)) return;
            AddLine(Local(particles[a].Position), Local(particles[b].Position), WireColor);
        }

        void DrawContacts(PhysicsWorld world) {
            IReadOnlyList<DeformableBody> bodies = world.Bodies;
            int count = world.Collision.ContactCount;
            for (int i = 0; i < count; i++) {
                Contact contact = world.Collision.GetContact(i);
                if(contact.BodyIndex < 0 || contact.BodyIndex >= bodies.Count) continue;
                DeformableBody body = bodies[contact.BodyIndex];
                if(body == null || contact.ParticleIndex < 0 || contact.ParticleIndex >= body.Particles.Length) continue;
                Vector3 point = Local(body.Particles[contact.ParticleIndex].Position);
                if(ShowContacts) AddCross(point, ContactSize, ContactColor);
                if(!ShowContactNormals || contact.Normal.SqrLength < 0.5f) continue;
                Vector3 direction = transform.InverseTransformDirection(new Vector3(contact.Normal.X, contact.Normal.Y, contact.Normal.Z));
                AddLine(point, point + direction * NormalLength, NormalColor);
            }
        }

        void AddCross(Vector3 point, float size, Color color) {
            AddLine(point + Vector3.left * size, point + Vector3.right * size, color);
            AddLine(point + Vector3.down * size, point + Vector3.up * size, color);
            AddLine(point + Vector3.back * size, point + Vector3.forward * size, color);
        }

        void AddLine(Vector3 start, Vector3 end, Color color) {
            int index = lineVertices.Count;
            lineVertices.Add(start);
            lineVertices.Add(end);
            lineColors.Add(color);
            lineColors.Add(color);
            lineIndices.Add(index);
            lineIndices.Add(index + 1);
        }

        Vector3 Local(Vec3 position) { return transform.InverseTransformPoint(new Vector3(position.X, position.Y, position.Z)); }

        void EnsureMesh() {
            if(lineMesh == null) {
                lineMesh = new Mesh { name = "Physics Debug" };
                lineMesh.MarkDynamic();
            }

            MeshFilter filter = GetComponent<MeshFilter>();
            if(filter == null) filter = gameObject.AddComponent<MeshFilter>();
            filter.sharedMesh = lineMesh;

            MeshRenderer renderer = GetComponent<MeshRenderer>();
            if(renderer == null) renderer = gameObject.AddComponent<MeshRenderer>();
            if(lineMaterial == null) lineMaterial = CreateLineMaterial();
            renderer.sharedMaterial = lineMaterial;
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        }

        static Material CreateLineMaterial() {
            Shader shader = Shader.Find("Physics/DebugLines");
            if(shader == null) shader = Shader.Find("Universal Render Pipeline/Unlit");
            if(shader == null) shader = Shader.Find("Sprites/Default");
            return new Material(shader) { name = "Physics Debug Lines", hideFlags = HideFlags.DontSave };
        }

        void Upload() {
            lineMesh.Clear();
            lineMesh.MarkDynamic();
            lineMesh.SetVertices(lineVertices);
            lineMesh.SetColors(lineColors);
            lineMesh.SetIndices(lineIndices, MeshTopology.Lines, 0);
            lineMesh.RecalculateBounds();
        }

        static bool Pressed(Keyboard keyboard, Key key) { return keyboard[key].wasPressedThisFrame; }

        static string ToggleLabel(string name, bool enabled) { return name + (enabled ? "  on" : "  off"); }

        void OnDestroy() {
            DestroyOwned(lineMesh);
            DestroyOwned(lineMaterial);
            lineMesh = null;
            lineMaterial = null;
        }

        static void DestroyOwned(Object owned) {
            if(owned == null) return;
            if(Application.isPlaying) Destroy(owned);
            else DestroyImmediate(owned);
        }
    }
}
