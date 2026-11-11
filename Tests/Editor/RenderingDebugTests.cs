using SoftBodyPhysics.Samples;
using NUnit.Framework;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;
using SoftBodyPhysics.Unity;
using UnityEngine;

namespace SoftBodyPhysics.Tests {
    public class RenderingDebugTests {
        [Test]
        public void MeshVerticesAndNormalsFollowTheParticles() {
            var center = new Vec3(2f, -3f, 4f);
            DeformableBody body = SphereGeometry.Create(1f, 4, 2, BodyMaterial.Default, center);
            Particle north = body.Particles[0];
            north.Position.Y += 1f;
            body.Particles[0] = north;

            GameObject view = Temporary("mesh-follow");
            Material material = new Material(Shader.Find("Sprites/Default"));
            material.hideFlags = HideFlags.HideAndDontSave;
            try {
                DeformableMeshRenderer renderer = view.AddComponent<DeformableMeshRenderer>();
                renderer.Bind(body, material);

                Mesh mesh = renderer.SharedMesh;
                Assert.AreEqual(body.Particles.Length, mesh.vertexCount);
                Assert.AreEqual(body.Triangles.Length, mesh.triangles.Length);
                Vector3 written = mesh.vertices[0];
                Assert.AreEqual(north.Position.X, written.x, 1e-4f);
                Assert.AreEqual(north.Position.Y, written.y, 1e-4f);
                Assert.AreEqual(north.Position.Z, written.z, 1e-4f);
                Assert.AreEqual(1f, mesh.normals[0].magnitude, 1e-3f);
                Assert.Greater(mesh.normals[0].y, 0.5f);
                Assert.AreSame(material, view.GetComponent<MeshRenderer>().sharedMaterial);

                north.Position.X += 0.25f;
                body.Particles[0] = north;
                renderer.Publish();
                Assert.AreEqual(north.Position.X, renderer.SharedMesh.vertices[0].x, 1e-4f);
            } finally {
                Object.DestroyImmediate(view);
                Object.DestroyImmediate(material);
            }
        }

        [Test]
        public void SplineSurfacePassesThroughTheParticles() {
            DeformableBody body = SphereGeometry.Create(1f, 4, 2, BodyMaterial.Default, new Vec3(2f, 3f, -4f));
            Particle north = body.Particles[0];
            north.Position.Y += 0.4f;
            body.Particles[0] = north;

            GameObject view = Temporary("spline");
            Material material = new Material(Shader.Find("Sprites/Default"));
            material.hideFlags = HideFlags.HideAndDontSave;
            try {
                DeformableMeshRenderer renderer = view.AddComponent<DeformableMeshRenderer>();
                renderer.Bind(body, material, 4, 2);

                int columns = 4 * DeformableMeshRenderer.SplineDivisions;
                int rows = 2 * DeformableMeshRenderer.SplineDivisions;
                int equator = 1 + (DeformableMeshRenderer.SplineDivisions - 1) * columns;
                Mesh mesh = renderer.SharedMesh;
                Assert.AreEqual(2 + (rows - 1) * columns, mesh.vertexCount);
                Assert.Greater(mesh.vertexCount, body.Particles.Length);
                Vector3 pole = mesh.vertices[0];
                Assert.AreEqual(north.Position.X, pole.x, 1e-3f);
                Assert.AreEqual(north.Position.Y, pole.y, 1e-3f);
                Assert.AreEqual(north.Position.Z, pole.z, 1e-3f);
                Vector3 side = mesh.vertices[equator];
                Assert.AreEqual(body.Particles[1].Position.X, side.x, 1e-3f);
                Assert.AreEqual(body.Particles[1].Position.Y, side.y, 1e-3f);
                Assert.AreEqual(body.Particles[1].Position.Z, side.z, 1e-3f);
                Assert.AreEqual(1f, mesh.normals[equator].magnitude, 1e-2f);

                north.Position.X += 0.3f;
                body.Particles[0] = north;
                renderer.Publish();
                Assert.AreEqual(north.Position.X, renderer.SharedMesh.vertices[0].x, 1e-3f);
            } finally {
                Object.DestroyImmediate(view);
                Object.DestroyImmediate(material);
            }
        }

        [Test]
        public void WireframeDrawsEachEdgeOnce() {
            DeformableBody body = SphereGeometry.Create(1f, 4, 2, BodyMaterial.Default, Vec3.Zero);
            GameObject view = Temporary("wireframe");
            try {
                PhysicsDebugRenderer debug = view.AddComponent<PhysicsDebugRenderer>();
                Silence(debug);
                debug.ShowWireframe = true;
                debug.Publish(WorldWith(body));

                Assert.AreEqual(24, debug.LineMesh.vertexCount);
                Assert.AreEqual(24, (int)debug.LineMesh.GetIndexCount(0));
            } finally { Object.DestroyImmediate(view); }
        }

        [Test]
        public void DistanceLinesFollowTheConstraints() {
            DeformableBody body = SphereGeometry.Create(1f, 4, 2, BodyMaterial.Default, Vec3.Zero);
            GameObject view = Temporary("distances");
            try {
                PhysicsDebugRenderer debug = view.AddComponent<PhysicsDebugRenderer>();
                Silence(debug);
                debug.ShowDistanceConstraints = true;
                debug.Publish(WorldWith(body));

                Assert.AreEqual(body.Constraints.Count * 2, debug.LineMesh.vertexCount);
                Assert.AreEqual(24, debug.LineMesh.vertexCount);
            } finally { Object.DestroyImmediate(view); }
        }

        [Test]
        public void CenterOfMassMarkSitsOnTheBody() {
            DeformableBody body = SphereGeometry.Create(1f, 4, 2, BodyMaterial.Default, Vec3.Zero);
            GameObject view = Temporary("center");
            try {
                PhysicsDebugRenderer debug = view.AddComponent<PhysicsDebugRenderer>();
                Silence(debug);
                debug.ShowCenterOfMass = true;
                debug.Publish(WorldWith(body));

                Assert.AreEqual(6, debug.LineMesh.vertexCount);
                Vector3 arm = new Vector3(PhysicsDebugRenderer.CenterSize, 0f, 0f);
                bool found = false;
                Vector3[] marks = debug.LineMesh.vertices;
                for (int i = 0; i < marks.Length; i++) { if((marks[i] - arm).sqrMagnitude < 1e-8f) found = true; }

                Assert.IsTrue(found);
            } finally { Object.DestroyImmediate(view); }
        }

        [Test]
        public void ContactNormalIsDrawnFromTheParticle() {
            var material = BodyMaterial.Default;
            material.Gravity = Vec3.Zero;
            material.Damping = 0f;
            material.Friction = 0f;
            material.Restitution = 0f;
            var body = new DeformableBody(1, material);
            var start = new Vec3(0.4f, -0.2f, -1.2f);
            body.Particles[0] = new Particle { Position = start, Previous = start, InverseMass = 1f };

            PhysicsWorld world = WorldWith(body);
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            world.Tick(0.02f);
            Assert.AreEqual(1, world.Collision.ContactCount);
            Assert.AreEqual(0f, body.Particles[0].Position.Y, 1e-4f);

            GameObject view = Temporary("contact-normal");
            try {
                PhysicsDebugRenderer debug = view.AddComponent<PhysicsDebugRenderer>();
                Silence(debug);
                debug.ShowContactNormals = true;
                debug.Publish(world);

                Assert.AreEqual(2, debug.LineMesh.vertexCount);
                Vector3 from = debug.LineMesh.vertices[0];
                Vector3 to = debug.LineMesh.vertices[1];
                Assert.AreEqual(0.4f, from.x, 1e-3f);
                Assert.AreEqual(0f, from.y, 1e-3f);
                Assert.AreEqual(-1.2f, from.z, 1e-3f);
                Assert.AreEqual(0.4f, to.x, 1e-3f);
                Assert.AreEqual(PhysicsDebugRenderer.NormalLength, to.y, 1e-3f);
                Assert.AreEqual(-1.2f, to.z, 1e-3f);
            } finally { Object.DestroyImmediate(view); }
        }

        static void Silence(PhysicsDebugRenderer debug) {
            debug.ShowParticles = false;
            debug.ShowDistanceConstraints = false;
            debug.ShowCenterOfMass = false;
            debug.ShowContacts = false;
            debug.ShowContactNormals = false;
            debug.ShowWireframe = false;
        }

        static PhysicsWorld WorldWith(DeformableBody body) {
            var world = new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 0.02f,
                SolverIterations = 1,
                MaxStepsPerFrame = 4
            });
            world.AddBody(body);
            return world;
        }

        static GameObject Temporary(string name) {
            var view = new GameObject(name);
            view.hideFlags = HideFlags.HideAndDontSave;
            return view;
        }
    }
}
