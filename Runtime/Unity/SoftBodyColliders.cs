using System.Collections.Generic;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Core;
using UnityEngine;

namespace SoftBodyPhysics.Unity {

    public static class SoftBodyColliders {
        sealed class Memory {
            public Collider Collider;
            public Matrix4x4 Previous;
            public bool HasPrevious;
            public Mesh Source;
            public Vector3[] LocalVertices;
            public int[] Indices;
            public int Closed;
            public CollisionTriangle[] Baked;
            public Matrix4x4 BakedMatrix;
            public bool BakedReady;
        }

        static readonly List<Memory> memories = new List<Memory>();
        static readonly HashSet<string> unreadable = new HashSet<string>();
        static Collider[] listed = System.Array.Empty<Collider>();
        static Transform listedRoot;
        static bool listDirty = true;

        public static int LastSceneScans { get; private set; }
        public static int LastMeshReads { get; private set; }
        public static float LastSyncMilliseconds { get; private set; }

        public static void Invalidate() { listDirty = true; }

        public static System.Func<Mesh, bool> PrepareMesh;

        public static void Sync(CollisionWorld collision) { Sync(collision, null); }

        public static void Sync(CollisionWorld collision, Transform root) {
            if(collision == null) return;

            long started = System.Diagnostics.Stopwatch.GetTimestamp();
            LastMeshReads = 0;
            collision.ClearSolids();
            Collider[] colliders = CollidersFor(root);
            var seen = new List<Collider>(colliders.Length);
            for (int i = 0; i < colliders.Length; i++) {
                Collider collider = colliders[i];
                if(!Include(collider)) continue;
                seen.Add(collider);
                Capture(collider, out Vec3 motion, out ColliderPose pose);
                if(collider is BoxCollider box) AddBox(collision, box, motion, pose);
                else if(collider is SphereCollider sphere) AddSphere(collision, sphere, motion, pose);
                else if(collider is CapsuleCollider capsule) AddCapsule(collision, capsule, motion, pose);
                else if(collider is MeshCollider mesh) AddMesh(collision, mesh, motion, pose);
            }

            ForgetUnseen(seen);
            LastSyncMilliseconds = (float)((System.Diagnostics.Stopwatch.GetTimestamp() - started) * 1000.0 / System.Diagnostics.Stopwatch.Frequency);
        }

        static Collider[] CollidersFor(Transform root) {
            if(listedRoot != root) {
                listedRoot = root;
                listDirty = true;
            }

            if(!listDirty && ListedIntact()) {
                LastSceneScans = 0;
                return listed;
            }

            LastSceneScans = 1;
            listed = root == null
                ? Object.FindObjectsByType<Collider>(FindObjectsInactive.Exclude)
                : root.GetComponentsInChildren<Collider>(false);
            listDirty = false;
            return listed;
        }

        static bool ListedIntact() {
            if(listed == null) return false;
            for (int i = 0; i < listed.Length; i++) { if(listed[i] == null) return false; }

            return true;
        }

        static bool Include(Collider collider) {
            if(collider == null || !collider.enabled || collider.isTrigger) return false;
            if(collider.GetComponentInParent<SoftBody>() != null) return false;
            Rigidbody rigidbody = collider.attachedRigidbody;
            return rigidbody == null || rigidbody.isKinematic;
        }

        static void AddBox(CollisionWorld collision, BoxCollider box, Vec3 motion, ColliderPose pose) {
            Transform transform = box.transform;
            Vector3 center = transform.TransformPoint(box.center);
            Vector3 axisX = transform.TransformVector(new Vector3(box.size.x * 0.5f, 0f, 0f));
            Vector3 axisY = transform.TransformVector(new Vector3(0f, box.size.y * 0.5f, 0f));
            Vector3 axisZ = transform.TransformVector(new Vector3(0f, 0f, box.size.z * 0.5f));
            var shape = new CollisionOrientedBox {
                Center = ToVec(center),
                AxisX = ToVec(axisX),
                AxisY = ToVec(axisY),
                AxisZ = ToVec(axisZ),
                Motion = motion,
                Pose = pose
            };
            Surface(box, ref shape.UseSurface, ref shape.SurfaceFriction, ref shape.SurfaceRestitution);
            collision.AddOrientedBox(shape);
        }

        static void AddSphere(CollisionWorld collision, SphereCollider sphere, Vec3 motion, ColliderPose pose) {
            Transform transform = sphere.transform;
            Vector3 scale = transform.lossyScale;
            float scaled = Mathf.Max(Mathf.Abs(scale.x), Mathf.Max(Mathf.Abs(scale.y), Mathf.Abs(scale.z)));
            var shape = new CollisionSphere {
                Center = ToVec(transform.TransformPoint(sphere.center)),
                Radius = sphere.radius * scaled,
                Motion = motion,
                Pose = pose
            };
            Surface(sphere, ref shape.UseSurface, ref shape.SurfaceFriction, ref shape.SurfaceRestitution);
            collision.AddSphere(shape);
        }

        static void AddCapsule(CollisionWorld collision, CapsuleCollider capsule, Vec3 motion, ColliderPose pose) {
            Transform transform = capsule.transform;
            Vector3 axis = capsule.direction == 0 ? Vector3.right : capsule.direction == 2 ? Vector3.forward : Vector3.up;
            float height = Mathf.Max(capsule.height, capsule.radius * 2f);
            float half = height * 0.5f - capsule.radius;
            Vector3 scale = transform.lossyScale;
            float radial = capsule.direction == 0
                ? Mathf.Max(Mathf.Abs(scale.y), Mathf.Abs(scale.z))
                : capsule.direction == 2
                    ? Mathf.Max(Mathf.Abs(scale.x), Mathf.Abs(scale.y))
                    : Mathf.Max(Mathf.Abs(scale.x), Mathf.Abs(scale.z));
            var shape = new CollisionCapsule {
                Start = ToVec(transform.TransformPoint(capsule.center + axis * half)),
                End = ToVec(transform.TransformPoint(capsule.center - axis * half)),
                Radius = capsule.radius * radial,
                Motion = motion,
                Pose = pose
            };
            Surface(capsule, ref shape.UseSurface, ref shape.SurfaceFriction, ref shape.SurfaceRestitution);
            collision.AddCapsule(shape);
        }

        static void AddMesh(CollisionWorld collision, MeshCollider meshCollider, Vec3 motion, ColliderPose pose) {
            Mesh mesh = meshCollider.sharedMesh;
            if(mesh == null) return;
            if(!mesh.isReadable && PrepareMesh != null && PrepareMesh(mesh)) mesh = meshCollider.sharedMesh;
            if(mesh == null || !mesh.isReadable) {
                if(mesh != null && unreadable.Add(mesh.name)) Debug.LogWarning("Soft body collision skipped " + mesh.name + " because Read/Write is off.", meshCollider);
                return;
            }

            Memory memory = Remember(meshCollider);
            if(memory.Source != mesh || memory.LocalVertices == null) ReadMesh(memory, mesh);
            if(memory.LocalVertices == null || memory.Indices == null || memory.Indices.Length < 3) return;

            Transform transform = meshCollider.transform;
            Matrix4x4 current = transform.localToWorldMatrix;
            int use = 0;
            float friction = 0f;
            float restitution = 0f;
            Surface(meshCollider, ref use, ref friction, ref restitution);
            if(!memory.BakedReady || memory.BakedMatrix != current || memory.Baked == null)
                Bake(memory, transform, use, friction, restitution);
            else {
                for (int i = 0; i < memory.Baked.Length; i++) {
                    CollisionTriangle triangle = memory.Baked[i];
                    triangle.UseSurface = use;
                    triangle.SurfaceFriction = friction;
                    triangle.SurfaceRestitution = restitution;
                    memory.Baked[i] = triangle;
                }
            }

            for (int i = 0; i < memory.Baked.Length; i++) {
                CollisionTriangle triangle = memory.Baked[i];
                triangle.Motion = motion;
                triangle.Pose = pose;
                collision.AddTriangle(triangle);
            }
        }

        static void ReadMesh(Memory memory, Mesh mesh) {
            LastMeshReads++;
            Vector3[] vertices = mesh.vertices;
            int[] indices = mesh.triangles;
            memory.Source = mesh;
            memory.LocalVertices = vertices;
            memory.Indices = indices;
            memory.Closed = indices == null ? 0 : ClosedSurface(indices);
            memory.BakedReady = false;
        }

        static void Bake(Memory memory, Transform transform, int use, float friction, float restitution) {
            int[] indices = memory.Indices;
            Vector3[] vertices = memory.LocalVertices;
            int count = indices.Length / 3;
            if(memory.Baked == null || memory.Baked.Length != count) memory.Baked = new CollisionTriangle[count];
            int written = 0;
            for (int i = 0; i + 2 < indices.Length; i += 3) {
                int a = indices[i];
                int b = indices[i + 1];
                int c = indices[i + 2];
                if(a < 0 || b < 0 || c < 0 || a >= vertices.Length || b >= vertices.Length || c >= vertices.Length) continue;
                memory.Baked[written] = new CollisionTriangle {
                    A = ToVec(transform.TransformPoint(vertices[a])),
                    B = ToVec(transform.TransformPoint(vertices[b])),
                    C = ToVec(transform.TransformPoint(vertices[c])),
                    Closed = memory.Closed,
                    UseSurface = use,
                    SurfaceFriction = friction,
                    SurfaceRestitution = restitution
                };
                written++;
            }

            if(written != memory.Baked.Length) {
                var fitted = new CollisionTriangle[written];
                for (int i = 0; i < written; i++) fitted[i] = memory.Baked[i];
                memory.Baked = fitted;
            }

            memory.BakedMatrix = transform.localToWorldMatrix;
            memory.BakedReady = true;
        }

        static void Surface(Collider collider, ref int use, ref float friction, ref float restitution) {
            UnityEngine.PhysicsMaterial material = collider.sharedMaterial;
            if(material == null) return;
            use = 1;
            friction = material.dynamicFriction;
            restitution = material.bounciness;
        }

        static int ClosedSurface(int[] indices) {
            var edges = new Dictionary<long, int>();
            for (int i = 0; i + 2 < indices.Length; i += 3) {
                CountEdge(edges, indices[i], indices[i + 1]);
                CountEdge(edges, indices[i + 1], indices[i + 2]);
                CountEdge(edges, indices[i + 2], indices[i]);
            }

            foreach (int count in edges.Values) { if(count != 2) return 0; }

            return edges.Count > 0 ? 1 : 0;
        }

        static void CountEdge(Dictionary<long, int> edges, int a, int b) {
            int low = a < b ? a : b;
            int high = a < b ? b : a;
            long key = ((long)low << 32) | (uint)high;
            edges.TryGetValue(key, out int count);
            edges[key] = count + 1;
        }

        static void Capture(Collider collider, out Vec3 motion, out ColliderPose pose) {
            Matrix4x4 current = collider.transform.localToWorldMatrix;
            Memory memory = null;
            for (int i = 0; i < memories.Count; i++) {
                if(memories[i].Collider != collider) continue;
                memory = memories[i];
                break;
            }

            if(memory == null) memory = Remember(collider);

            Matrix4x4 previous = memory.HasPrevious ? memory.Previous : current;
            memory.Previous = current;
            memory.HasPrevious = true;
            Vector3 delta = current.GetPosition() - previous.GetPosition();
            motion = ToVec(delta);
            pose = new ColliderPose {
                HasBasis = 1,
                Origin = ToVec(current.GetPosition()),
                X = ToVec(current.MultiplyVector(Vector3.right)),
                Y = ToVec(current.MultiplyVector(Vector3.up)),
                Z = ToVec(current.MultiplyVector(Vector3.forward)),
                PreviousOrigin = ToVec(previous.GetPosition()),
                PreviousX = ToVec(previous.MultiplyVector(Vector3.right)),
                PreviousY = ToVec(previous.MultiplyVector(Vector3.up)),
                PreviousZ = ToVec(previous.MultiplyVector(Vector3.forward))
            };
        }

        static Memory Remember(Collider collider) {
            for (int i = 0; i < memories.Count; i++) { if(memories[i].Collider == collider) return memories[i]; }

            var memory = new Memory { Collider = collider };
            memories.Add(memory);
            return memory;
        }

        static void ForgetUnseen(List<Collider> seen) {
            for (int i = memories.Count - 1; i >= 0; i--) {
                if(memories[i].Collider != null && seen.Contains(memories[i].Collider)) continue;
                memories.RemoveAt(i);
            }
        }

        static Vec3 ToVec(Vector3 value) { return new Vec3(value.x, value.y, value.z); }
    }
}
