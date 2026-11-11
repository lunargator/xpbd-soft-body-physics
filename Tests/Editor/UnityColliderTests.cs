using System.Collections.Generic;
using NUnit.Framework;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;
using SoftBodyPhysics.Unity;
using UnityEngine;

namespace SoftBodyPhysics.Tests {
    public class UnityColliderTests {
        readonly List<Object> owned = new List<Object>();

        [Test]
        public void SoftSphereLandsOnAUnityBox() {
            Transform root = Root();
            GameObject floor = Child(root, "Floor");
            floor.transform.position = new Vector3(0f, -0.1f, 0f);
            BoxCollider box = floor.AddComponent<BoxCollider>();
            box.size = new Vector3(4f, 0.2f, 4f);
            try {
                DeformableBody body = Drop(root, new Vec3(0f, 1.2f, 0f));
                float minY = Lowest(body);
                Assert.Greater(minY, -0.03f);
                Assert.Less(minY, 0.2f);
            } finally { Cleanup(); }
        }

        [Test]
        public void SoftSphereLandsOnASphereCollider() {
            Transform root = Root();
            GameObject solid = Child(root, "Solid");
            solid.transform.position = new Vector3(0f, -3f, 0f);
            SphereCollider sphere = solid.AddComponent<SphereCollider>();
            sphere.radius = 3f;
            try {
                DeformableBody body = Drop(root, new Vec3(0f, 1.2f, 0f));
                Assert.Greater(Lowest(body), -0.05f);
                Assert.Less(Lowest(body), 0.25f);
            } finally { Cleanup(); }
        }

        [Test]
        public void SoftSphereLandsOnACapsuleCollider() {
            Transform root = Root();
            GameObject solid = Child(root, "Solid");
            solid.transform.position = new Vector3(0f, -2f, 0f);
            CapsuleCollider capsule = solid.AddComponent<CapsuleCollider>();
            capsule.direction = 0;
            capsule.radius = 2f;
            capsule.height = 6f;
            try {
                DeformableBody body = Drop(root, new Vec3(0f, 1.2f, 0f));
                Assert.Greater(Lowest(body), -0.05f);
                Assert.Less(Lowest(body), 0.25f);
            } finally { Cleanup(); }
        }

        [Test]
        public void SoftSphereLandsOnAStaticMesh() {
            Transform root = Root();
            GameObject floor = Child(root, "Mesh Floor");
            var mesh = new Mesh { name = "Floor Quad" };
            owned.Add(mesh);
            mesh.vertices = new[] {
                new Vector3(-3f, 0f, -3f),
                new Vector3(3f, 0f, -3f),
                new Vector3(3f, 0f, 3f),
                new Vector3(-3f, 0f, 3f)
            };
            mesh.triangles = new[] { 0, 2, 1, 0, 3, 2 };
            mesh.RecalculateNormals();
            MeshCollider collider = floor.AddComponent<MeshCollider>();
            collider.sharedMesh = mesh;
            try {
                DeformableBody body = Drop(root, new Vec3(0f, 1.2f, 0f));
                float minY = Lowest(body);
                Assert.Greater(minY, -0.03f);
                Assert.Less(minY, 0.2f);
            } finally { Cleanup(); }
        }

        [Test]
        public void KinematicPlatformLiftsTheSoftSphere() {
            Transform root = Root();
            GameObject platform = Child(root, "Platform");
            Rigidbody rigidbody = platform.AddComponent<Rigidbody>();
            rigidbody.isKinematic = true;
            BoxCollider box = platform.AddComponent<BoxCollider>();
            box.size = new Vector3(3f, 0.2f, 3f);
            box.center = new Vector3(0f, -0.1f, 0f);
            try {
                DeformableBody body = Drop(root, new Vec3(0f, 0.8f, 0f));
                float rested = Lowest(body);
                Assert.Greater(rested, -0.03f);
                for (int i = 0; i < 8; i++) {
                    platform.transform.position += new Vector3(0f, 0.05f, 0f);
                    SoftBodyColliders.Sync(bodyWorld.Collision, root);
                    bodyWorld.Step(1f / 60f);
                }

                Assert.Greater(Lowest(body), rested + 0.25f);
            } finally { Cleanup(); }
        }

        [Test]
        public void DynamicRigidbodiesAreNotCopied() {
            Transform root = Root();
            GameObject mover = Child(root, "Dynamic");
            Rigidbody rigidbody = mover.AddComponent<Rigidbody>();
            rigidbody.isKinematic = false;
            mover.AddComponent<BoxCollider>();
            var world = new PhysicsWorld(new PhysicsSettings());
            try {
                SoftBodyColliders.Sync(world.Collision, root);
                Assert.AreEqual(0, world.Collision.OrientedBoxCount);
                rigidbody.isKinematic = true;
                SoftBodyColliders.Sync(world.Collision, root);
                Assert.AreEqual(1, world.Collision.OrientedBoxCount);
            } finally { Cleanup(); }
        }

        PhysicsWorld bodyWorld;

        Transform Root() {
            var host = new GameObject("Collider Test");
            owned.Add(host);
            return host.transform;
        }

        GameObject Child(Transform root, string name) {
            var child = new GameObject(name);
            owned.Add(child);
            child.transform.SetParent(root, false);
            return child;
        }

        DeformableBody Drop(Transform root, Vec3 center) {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 0.3f;
            material.Friction = 0.4f;
            material.Restitution = 0.05f;
            DeformableBody body = SphereGeometry.Create(0.3f, 6, 4, material, center);
            SoftBodyShapes.Finish(body, material.Compliance);
            bodyWorld = new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 1f / 60f,
                SolverIterations = 4,
                MaxStepsPerFrame = 1
            });
            bodyWorld.AddBody(body);
            for (int step = 0; step < 120; step++) {
                SoftBodyColliders.Sync(bodyWorld.Collision, root);
                bodyWorld.Step(1f / 60f);
            }

            return body;
        }

        static float Lowest(DeformableBody body) {
            float minY = float.MaxValue;
            for (int i = 0; i < body.Particles.Length; i++) {
                float y = body.Particles[i].Position.Y;
                if(y < minY) minY = y;
                Assert.IsFalse(body.Particles[i].Position.HasNaN());
            }

            return minY;
        }

        void Cleanup() {
            for (int i = owned.Count - 1; i >= 0; i--) { if(owned[i] != null) Object.DestroyImmediate(owned[i]); }

            owned.Clear();
            bodyWorld = null;
        }
    }
}
