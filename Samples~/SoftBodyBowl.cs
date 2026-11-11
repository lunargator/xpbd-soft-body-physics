using System.Collections.Generic;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Core;
using UnityEngine;
using UnityEngine.InputSystem;
using SoftBodyPhysics.Unity;

namespace SoftBodyPhysics.Samples {

    [DefaultExecutionOrder(-200)]
    public sealed class SoftBodyBowl : MonoBehaviour {
        public const float MinX = -1.15f;
        public const float MaxX = 1.15f;
        public const float MinY = 0.08f;
        public const float MaxY = 2.18f;
        public const float MinZ = -1.15f;
        public const float MaxZ = 1.15f;
        public const int SphereCount = 20;
        public const int RockCount = 6;
        public const float SphereRadius = 0.24f;
        public const float RockRadius = 0.26f;
        public const int PackedSphereCount = 32;
        public const int PackedRockCount = 4;
        public const float PackedRadius = 0.16f;
        public const float PackedRockRadius = 0.18f;
        public const float ShoveForce = 320f;
        public const float ShoveSeconds = 0.16f;
        public const float PlayShove = 78f;

        static readonly Color[] Colors = {
            new Color(0.93f, 0.34f, 0.28f),
            new Color(0.98f, 0.70f, 0.22f),
            new Color(0.84f, 0.22f, 0.34f),
            new Color(0.20f, 0.48f, 0.92f),
            new Color(0.22f, 0.72f, 0.46f),
            new Color(0.55f, 0.34f, 0.86f)
        };

        readonly List<SoftBody> spheres = new List<SoftBody>();
        readonly List<SoftBody> rocks = new List<SoftBody>();
        readonly List<BodyRest> sphereRest = new List<BodyRest>();
        readonly List<BodyRest> rockRest = new List<BodyRest>();
        readonly Dictionary<SoftBody, Vector3> lastShove = new Dictionary<SoftBody, Vector3>();
        readonly Dictionary<SoftBody, Vector3> shoveDirection = new Dictionary<SoftBody, Vector3>();
        float shoveUntil;
        int builtEpoch = -1;
        float orbitYaw;
        float orbitPitch;
        float clipClock;
        float clipYaw0;
        float clipPitch0;
        float steerYaw;
        float steerPitch;
        float orbitDistance;
        bool clipPlaying;
        const float OrbitRadiusNear = 3.92f;
        const float OrbitRadiusFar = 6.9f;
        const float OrbitSpeed = 1.25f;
        const float ClipSeconds = 12f;
        const float ClipYawSweep = 2.05f;

        public static CollisionPlane[] CreatePlanes() {
            return new[] {
                CollisionPlane.Horizontal(MinY),
                CollisionPlane.FromNormalAndPoint(new Vec3(0f, -1f, 0f), new Vec3(0f, MaxY, 0f)),
                CollisionPlane.FromNormalAndPoint(new Vec3(1f, 0f, 0f), new Vec3(MinX, 0f, 0f)),
                CollisionPlane.FromNormalAndPoint(new Vec3(-1f, 0f, 0f), new Vec3(MaxX, 0f, 0f)),
                CollisionPlane.FromNormalAndPoint(new Vec3(0f, 0f, 1f), new Vec3(0f, 0f, MinZ)),
                CollisionPlane.FromNormalAndPoint(new Vec3(0f, 0f, -1f), new Vec3(0f, 0f, MaxZ))
            };
        }

        public static void AddCollision(CollisionWorld collision) {
            if(collision == null) return;
            collision.ClearPlanes();
            CollisionPlane[] planes = CreatePlanes();
            for (int i = 0; i < planes.Length; i++) collision.AddPlane(planes[i]);
        }

        public static float Separation(CollisionPlane plane, Vec3 point) { return Vec3.Dot(plane.Normal, point) - plane.Offset; }

        public static Vec3 SlotCenter(int index) {
            const int columns = 4;
            const int rows = 3;
            int column = index % columns;
            int row = (index / columns) % rows;
            int layer = index / (columns * rows);
            float inset = SphereRadius + 0.06f;
            float x = Lerp(MinX + inset, MaxX - inset, column / (float)(columns - 1));
            float y = Lerp(MinY + inset, MaxY - inset, row / (float)(rows - 1));
            float z = Lerp(MinZ + inset, MaxZ - inset, layer <= 0 ? 0.28f : 0.72f);
            return new Vec3(x, y, z);
        }

        public static Vector3 PackedSpherePosition(int index) {
            const int columns = 4;
            const int rows = 4;
            int column = index % columns;
            int row = (index / columns) % rows;
            int layer = index / (columns * rows);
            float inset = PackedRadius + 0.05f;
            float x = Mathf.Lerp(MinX + inset, MaxX - inset, column / (float)(columns - 1));
            float z = Mathf.Lerp(MinZ + inset, MaxZ - inset, row / (float)(rows - 1));
            float y = Mathf.Lerp(MinY + inset, MaxY - inset, layer <= 0 ? 0.18f : 0.82f);
            return new Vector3(x, y, z);
        }

        public static Vector3 PackedRockPosition(int index) {
            float x = ((index % 2) - 0.5f) * 0.7f;
            float z = (index / 2) * 0.7f - 0.35f;
            return new Vector3(x, (MinY + MaxY) * 0.5f, z);
        }

        public static Vec3 RockSlot(int index) {
            float x = ((index % 3) - 1) * 0.46f;
            float y = 0.72f + index * 0.2f;
            float z = (index / 3) * 0.42f - 0.21f;
            return new Vec3(x, y, z);
        }

        void Update() {
            if(!Application.isPlaying) return;

            Application.targetFrameRate = 60;
            QualitySettings.vSyncCount = 0;
            if(builtEpoch != SamplePlay.Epoch) Build();
            SampleTechView.Ensure(this);
            if(Pressed(Key.F)) ShoveAll();
            if(Pressed(Key.R)) ToggleSmooth();
            if(Time.time < shoveUntil) ApplyShove();
        }

        void LateUpdate() {
            if(!Application.isPlaying) return;
            bool shoving = Time.time < shoveUntil;
            Hold(spheres, sphereRest, shoving);
            Hold(rocks, rockRest, shoving);
            OrbitCamera();
        }

        void Build() {
            builtEpoch = SamplePlay.Epoch;
            SoftBodySimulation simulation = SoftBodySimulation.Ensure();
            simulation.Play(new Vector3(0f, -9.81f, 0f), 4, 1, 1f / 60f);
            simulation.Collision.ClearPlanes();
            simulation.Collision.ShellScale = 1.05f;
            BuildGlass(transform);
            SoftBodyColliders.Invalidate();
            EnsureSpheres();
            EnsureRocks();
            FrameCamera();
            SampleLights.EnableShadows();
        }

        void EnsureSpheres() {
            for (int i = spheres.Count - 1; i >= 0; i--) { if(spheres[i] == null) spheres.RemoveAt(i); }

            if(spheres.Count > 0) return;

            for (int i = 0; i < PackedSphereCount; i++) spheres.Add(Spawn(i, PackedSpherePosition(i)));
        }

        SoftBody Spawn(int index, Vector3 position) {
            var host = new GameObject("Box Sphere " + index);
            host.transform.SetParent(transform, false);
            host.transform.position = position;
            SoftBody body = host.AddComponent<SoftBody>();
            body.shape = SoftBody.ShapeKind.Sphere;
            body.radius = PackedRadius;
            body.sphereSegments = 6;
            body.sphereStacks = 4;
            body.mass = 1f;
            body.stiffness = 0.35f;
            body.damping = 1.05f;
            body.friction = 0.65f;
            body.restitution = 0f;
            body.surfaceOverride = true;
            body.gravityMultiplier = 1f;
            body.color = Colors[index % Colors.Length];
            body.smoothSurface = true;
            body.initializeOnEnable = false;
            body.resetOnEnable = true;
            body.Initialize();
            return body;
        }

        void EnsureRocks() {
            for (int i = rocks.Count - 1; i >= 0; i--) { if(rocks[i] == null) rocks.RemoveAt(i); }

            if(rocks.Count > 0) return;

            for (int i = 0; i < PackedRockCount; i++) rocks.Add(SpawnRock(i, PackedRockPosition(i)));
        }

        SoftBody SpawnRock(int index, Vector3 position) {
            Mesh mesh = RockMesh.Create(17 + index * 29, PackedRockRadius, "Random Rock " + index);

            var host = new GameObject("Box Rock " + index);
            host.transform.SetParent(transform, false);
            host.transform.SetPositionAndRotation(position, Quaternion.Euler(18f * index, 40f * index, 12f * index));
            SoftBody body = host.AddComponent<SoftBody>();
            body.shape = SoftBody.ShapeKind.Mesh;
            body.sourceMesh = mesh;
            body.meshCellSize = 0f;
            body.mass = 1f;
            body.stiffness = 0.35f;
            body.damping = 1.05f;
            body.friction = 0.65f;
            body.restitution = 0f;
            body.surfaceOverride = true;
            body.gravityMultiplier = 1f;
            body.color = Colors[(index + 2) % Colors.Length];
            body.smoothSurface = true;
            body.initializeOnEnable = false;
            body.resetOnEnable = true;
            body.Initialize();
            return body;
        }

        void ToggleSmooth() {
            bool next = true;
            for (int i = 0; i < spheres.Count; i++) {
                if(spheres[i] == null) continue;
                next = !spheres[i].smoothSurface;
                break;
            }

            SetSmooth(spheres, next);
            SetSmooth(rocks, next);
        }

        static void SetSmooth(List<SoftBody> bodies, bool value) {
            for (int i = 0; i < bodies.Count; i++) {
                if(bodies[i] == null) continue;
                bodies[i].smoothSurface = value;
            }
        }

        void ShoveAll() {
            Aim(spheres);
            Aim(rocks);
            shoveUntil = Time.time + ShoveSeconds;
        }

        void Aim(List<SoftBody> bodies) {
            for (int i = 0; i < bodies.Count; i++) {
                SoftBody body = bodies[i];
                if(body == null) continue;
                lastShove.TryGetValue(body, out Vector3 previous);
                Vector3 direction = DifferentDirection(previous);
                lastShove[body] = direction;
                shoveDirection[body] = direction * PlayShove;
            }
        }

        static Vector3 DifferentDirection(Vector3 previous) {
            if(previous.sqrMagnitude < 1e-6f) {
                Vector3 first = Random.onUnitSphere;
                return first.sqrMagnitude < 1e-6f ? Vector3.up : first.normalized;
            }

            Vector3 last = previous.normalized;
            for (int attempt = 0; attempt < 16; attempt++) {
                Vector3 direction = Random.onUnitSphere;
                if(direction.sqrMagnitude < 1e-6f) continue;
                direction.Normalize();
                if(Vector3.Dot(direction, last) < -0.45f) return direction;
            }

            return -last;
        }

        void ApplyShove() {
            Push(spheres);
            Push(rocks);
        }

        bool ThroughAnother(SoftBody body) {
            if(body == null || body.Body == null) return false;
            Vec3 center = body.Body.CenterOfMass();
            float reach = Reach(body.Body);
            return Through(body, center, reach, spheres) || Through(body, center, reach, rocks);
        }

        static bool Through(SoftBody body, Vec3 center, float reach, List<SoftBody> others) {
            for (int i = 0; i < others.Count; i++) {
                SoftBody other = others[i];
                if(other == null || other == body || other.Body == null) continue;
                Vec3 gap = center - other.Body.CenterOfMass();
                float span = reach + Reach(other.Body);
                float limit = span * 0.78f;
                if(limit > 0f && gap.SqrLength < limit * limit) return true;
            }

            return false;
        }

        static float Reach(DeformableBody body) {
            Vec3 center = body.CenterOfMass();
            float reach = 0f;
            Particle[] particles = body.Particles;
            for (int i = 0; i < particles.Length; i++) {
                Vec3 offset = particles[i].Position - center;
                float length = offset.Length;
                if(length > reach) reach = length;
            }

            return reach;
        }

        void Hold(List<SoftBody> bodies, List<BodyRest> rests, bool shoving) {
            while (rests.Count < bodies.Count) rests.Add(new BodyRest());
            if(rests.Count > bodies.Count) rests.RemoveRange(bodies.Count, rests.Count - bodies.Count);

            for (int i = 0; i < bodies.Count; i++) {
                SoftBody body = bodies[i];
                if(body == null || body.Body == null) {
                    rests[i].Release();
                    continue;
                }

                bool awake = shoving || ThroughAnother(body) || Falling(body.Body);
                if(!awake) Quiet(body.Body);
                rests[i].Apply(body.Body, awake);
            }
        }

        static bool Falling(DeformableBody body) {
            Particle[] particles = body.Particles;
            if(particles == null) return false;
            float vertical = 0f;
            int count = 0;
            for (int i = 0; i < particles.Length; i++) {
                if(particles[i].InverseMass <= 0f) continue;
                vertical += particles[i].Velocity.Y;
                count++;
            }

            return count > 0 && vertical / count < -0.05f;
        }

        static void Quiet(DeformableBody body) {
            Particle[] particles = body.Particles;
            if(particles == null) return;

            float fastest = 0f;
            for (int i = 0; i < particles.Length; i++) {
                float speed = particles[i].Velocity.Length;
                if(speed > fastest) fastest = speed;
            }

            if(fastest > 0.55f || fastest < 1e-4f || Falling(body)) return;

            float scale = fastest > 0.2f ? 0.84f : 0.55f;
            for (int i = 0; i < particles.Length; i++) {
                Particle particle = particles[i];
                if(particle.InverseMass <= 0f) continue;
                particle.Velocity = particle.Velocity * scale;
                particles[i] = particle;
            }
        }

        void Push(List<SoftBody> bodies) {
            for (int i = 0; i < bodies.Count; i++) {
                SoftBody body = bodies[i];
                if(body == null || body.Body == null || !body.simulate || body.kinematic) continue;
                if(!shoveDirection.TryGetValue(body, out Vector3 force)) continue;
                body.AddForce(force);
            }
        }

        static bool Pressed(Key key) {
            Keyboard keyboard = Keyboard.current;
            return keyboard != null && keyboard[key].wasPressedThisFrame;
        }

        void FrameCamera() {
            Camera camera = Camera.main;
            if(camera == null) return;
            var offset = new Vector3(2.2f, 1.45f, -2.9f);
            orbitYaw = Mathf.Atan2(offset.x, offset.z);
            orbitPitch = Mathf.Asin(Mathf.Clamp(offset.y / offset.magnitude, -1f, 1f));
            orbitDistance = OrbitRadiusFar;
            clipYaw0 = orbitYaw;
            clipPitch0 = orbitPitch;
            clipClock = 0f;
            steerYaw = 0f;
            steerPitch = 0f;
            clipPlaying = true;
            lastShove.Clear();
            shoveDirection.Clear();
            camera.fieldOfView = 40f;
            camera.clearFlags = CameraClearFlags.SolidColor;
            camera.backgroundColor = new Color(0.04f, 0.045f, 0.06f, 1f);
            PlaceCamera(camera);
        }

        void OrbitCamera() {
            Camera camera = Camera.main;
            if(camera == null) return;
            float yaw = 0f;
            float pitch = 0f;
            Keyboard keyboard = Keyboard.current;
            if(keyboard != null) {
                if(keyboard[Key.A].isPressed) yaw -= 1f;
                if(keyboard[Key.D].isPressed) yaw += 1f;
                if(keyboard[Key.W].isPressed) pitch += 1f;
                if(keyboard[Key.S].isPressed) pitch -= 1f;
            }

            if(clipPlaying) {
                clipClock += Time.deltaTime;
                float along = Mathf.Clamp01(clipClock / ClipSeconds);
                float eased = along * along * (3f - 2f * along);
                steerYaw += yaw * OrbitSpeed * Time.deltaTime;
                steerPitch += pitch * OrbitSpeed * Time.deltaTime;
                orbitYaw = clipYaw0 + eased * ClipYawSweep + steerYaw;
                orbitPitch = Mathf.Clamp(clipPitch0 + Mathf.Sin(eased * Mathf.PI) * 0.32f + steerPitch, 0.15f, 1.15f);
                orbitDistance = Mathf.Lerp(OrbitRadiusFar, OrbitRadiusNear, eased);
                if(along >= 1f) clipPlaying = false;
            } else {
                orbitYaw += yaw * OrbitSpeed * Time.deltaTime;
                orbitPitch = Mathf.Clamp(orbitPitch + pitch * OrbitSpeed * Time.deltaTime, 0.15f, 1.15f);
            }

            PlaceCamera(camera);
        }

        void PlaceCamera(Camera camera) {
            var center = new Vector3(0f, (MinY + MaxY) * 0.5f, 0f);
            float ring = Mathf.Cos(orbitPitch) * orbitDistance;
            var position = center + new Vector3(Mathf.Sin(orbitYaw) * ring, Mathf.Sin(orbitPitch) * orbitDistance, Mathf.Cos(orbitYaw) * ring);
            camera.transform.SetPositionAndRotation(position, Quaternion.LookRotation(center - position, Vector3.up));
        }

        public static void BuildGlass(Transform parent) {
            BuildGlass(parent, MinX, MaxX, MinY, MaxY, MinZ, MaxZ, false);
        }

        public static void BuildGlass(Transform parent, float x0, float x1, float y0, float y1, float z0, float z1, bool openTop) {
            if(parent == null) return;
            RemoveChild(parent, "Bowl");
            RemoveChild(parent, "Box");
            var vertices = new[] {
                new Vector3(x0, y0, z0),
                new Vector3(x1, y0, z0),
                new Vector3(x1, y0, z1),
                new Vector3(x0, y0, z1),
                new Vector3(x0, y1, z0),
                new Vector3(x1, y1, z0),
                new Vector3(x1, y1, z1),
                new Vector3(x0, y1, z1)
            };
            var normals = new Vector3[8];
            for (int i = 0; i < normals.Length; i++) {
                Vector3 inward = new Vector3(0f, (y0 + y1) * 0.5f, 0f) - vertices[i];
                normals[i] = inward.normalized;
            }

            int[] triangles = openTop
                ? new[] {
                    0, 2, 1, 0, 3, 2,
                    0, 1, 5, 0, 5, 4,
                    1, 2, 6, 1, 6, 5,
                    2, 3, 7, 2, 7, 6,
                    3, 0, 4, 3, 4, 7
                }
                : new[] {
                    0, 2, 1, 0, 3, 2,
                    4, 5, 6, 4, 6, 7,
                    0, 1, 5, 0, 5, 4,
                    1, 2, 6, 1, 6, 5,
                    2, 3, 7, 2, 7, 6,
                    3, 0, 4, 3, 4, 7
                };
            var uvs = new Vector2[8];
            for (int i = 0; i < uvs.Length; i++) uvs[i] = new Vector2(vertices[i].x, vertices[i].y);

            var mesh = new Mesh { name = "Box", hideFlags = HideFlags.DontSave };
            mesh.vertices = vertices;
            mesh.normals = normals;
            mesh.uv = uvs;
            mesh.triangles = triangles;
            mesh.RecalculateBounds();

            var box = new GameObject("Box");
            box.transform.SetParent(parent, false);
            box.AddComponent<MeshFilter>().sharedMesh = mesh;
            MeshRenderer renderer = box.AddComponent<MeshRenderer>();
            renderer.sharedMaterial = CreateGlass();
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            renderer.receiveShadows = false;
            const float shell = 0.08f;
            float width = x1 - x0;
            float depth = z1 - z0;
            float height = y1 - y0;
            float midY = (y0 + y1) * 0.5f;
            Face(box.transform, new Vector3(0f, y0 - shell * 0.5f, 0f), new Vector3(width + shell, shell, depth + shell));
            if(!openTop) Face(box.transform, new Vector3(0f, y1 + shell * 0.5f, 0f), new Vector3(width + shell, shell, depth + shell));
            Face(box.transform, new Vector3(x0 - shell * 0.5f, midY, 0f), new Vector3(shell, height, depth));
            Face(box.transform, new Vector3(x1 + shell * 0.5f, midY, 0f), new Vector3(shell, height, depth));
            Face(box.transform, new Vector3(0f, midY, z0 - shell * 0.5f), new Vector3(width, height, shell));
            Face(box.transform, new Vector3(0f, midY, z1 + shell * 0.5f), new Vector3(width, height, shell));
            Rail(box.transform, new Vector3(x0, y0, z0), new Vector3(x1, y0, z0));
            Rail(box.transform, new Vector3(x1, y0, z0), new Vector3(x1, y0, z1));
            Rail(box.transform, new Vector3(x1, y0, z1), new Vector3(x0, y0, z1));
            Rail(box.transform, new Vector3(x0, y0, z1), new Vector3(x0, y0, z0));
            Rail(box.transform, new Vector3(x0, y1, z0), new Vector3(x1, y1, z0));
            Rail(box.transform, new Vector3(x1, y1, z0), new Vector3(x1, y1, z1));
            Rail(box.transform, new Vector3(x1, y1, z1), new Vector3(x0, y1, z1));
            Rail(box.transform, new Vector3(x0, y1, z1), new Vector3(x0, y1, z0));
            Rail(box.transform, new Vector3(x0, y0, z0), new Vector3(x0, y1, z0));
            Rail(box.transform, new Vector3(x1, y0, z0), new Vector3(x1, y1, z0));
            Rail(box.transform, new Vector3(x1, y0, z1), new Vector3(x1, y1, z1));
            Rail(box.transform, new Vector3(x0, y0, z1), new Vector3(x0, y1, z1));
        }

        static void Face(Transform parent, Vector3 center, Vector3 size) {
            var face = new GameObject("Bowl Face");
            face.transform.SetParent(parent, false);
            face.transform.localPosition = center;
            BoxCollider box = face.AddComponent<BoxCollider>();
            box.size = size;
        }

        static void Rail(Transform parent, Vector3 from, Vector3 to) {
            var rail = GameObject.CreatePrimitive(PrimitiveType.Cube);
            rail.name = "Box Edge";
            rail.transform.SetParent(parent, false);
            Collider collider = rail.GetComponent<Collider>();
            if(Application.isPlaying) Object.Destroy(collider);
            else Object.DestroyImmediate(collider);
            Vector3 span = to - from;
            rail.transform.localPosition = (from + to) * 0.5f;
            rail.transform.localRotation = Quaternion.FromToRotation(Vector3.right, span);
            rail.transform.localScale = new Vector3(span.magnitude, 0.035f, 0.035f);
            rail.GetComponent<MeshRenderer>().sharedMaterial = SolidPaint.Create(new Color(0.90f, 0.93f, 0.96f, 1f));
        }

        static void RemoveChild(Transform parent, string childName) {
            Transform existing = parent.Find(childName);
            if(existing == null) return;
            if(Application.isPlaying) Object.Destroy(existing.gameObject);
            else Object.DestroyImmediate(existing.gameObject);
        }

        public static Material CreateGlass() {
            Material material = SolidPaint.Create(new Color(0.70f, 0.84f, 0.92f, 0.55f), 0.7f);
            material.SetFloat("_Cull", 2f);
            material.SetFloat("_Surface", 1f);
            material.SetFloat("_Blend", 0f);
            material.SetOverrideTag("RenderType", "Transparent");
            material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            material.SetInt("_ZWrite", 0);
            material.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            material.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
            material.EnableKeyword("_ALPHABLEND_ON");
            material.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
            material.SetShaderPassEnabled("ShadowCaster", false);
            return material;
        }

        static float Lerp(float a, float b, float t) { return a + (b - a) * t; }
    }

    public sealed class BodyRest {
        public const float SettleDistance = 0.004f;
        public const int HoldFrames = 36;

        public int CalmSteps;
        public bool Held;
        public float LastComSpeed;
        public Vec3[] Pose;
        Vec3 lastCenter;
        bool hasCenter;

        public void Release() {
            CalmSteps = 0;
            Held = false;
            hasCenter = false;
        }

        public void Apply(DeformableBody body, bool awake) {
            if(body == null || body.Particles == null || awake) {
                Release();
                return;
            }

            if(Held && Pose != null && Pose.Length == body.Particles.Length) {
                Restore(body);
                return;
            }

            if(!IsCalm(body)) {
                CalmSteps = 0;
                return;
            }

            CalmSteps++;
            if(CalmSteps < HoldFrames) return;

            Particle[] particles = body.Particles;
            Pose = new Vec3[particles.Length];
            for (int i = 0; i < particles.Length; i++) Pose[i] = particles[i].Position;
            Held = true;
            Stop(particles);
        }

        Vec3 Center(Particle[] particles) {
            Vec3 sum = Vec3.Zero;
            int count = 0;
            for (int i = 0; i < particles.Length; i++) {
                if(particles[i].InverseMass <= 0f) continue;
                sum += particles[i].Position;
                count++;
            }

            return count == 0 ? Vec3.Zero : sum * (1f / count);
        }

        void Restore(DeformableBody body) {
            Particle[] particles = body.Particles;
            for (int i = 0; i < particles.Length; i++) {
                Particle particle = particles[i];
                particle.Position = Pose[i];
                particle.Previous = Pose[i];
                particle.Velocity = Vec3.Zero;
                particle.Force = Vec3.Zero;
                particles[i] = particle;
            }
        }

        public bool IsCalm(DeformableBody body) {
            Vec3 center = Center(body.Particles);
            if(!hasCenter) {
                lastCenter = center;
                hasCenter = true;
                LastComSpeed = 0f;
                return false;
            }

            Vec3 shift = center - lastCenter;
            LastComSpeed = shift.Length;
            lastCenter = center;
            return LastComSpeed < SettleDistance;
        }

        static void Stop(Particle[] particles) {
            for (int i = 0; i < particles.Length; i++) {
                Particle particle = particles[i];
                particle.Previous = particle.Position;
                particle.Velocity = Vec3.Zero;
                particle.Force = Vec3.Zero;
                particles[i] = particle;
            }
        }
    }
}
