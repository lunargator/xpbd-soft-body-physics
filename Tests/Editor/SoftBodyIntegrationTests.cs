using System;
using NUnit.Framework;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Unity;
using UnityEngine;

namespace SoftBodyPhysics.Tests {
    public class SoftBodyIntegrationTests {
        [Test]
        public void DroppedBodiesLandAndTheMeshStaysOnTheParticles() {
            Cleanup();
            SoftBody sphere = Create(SoftBody.ShapeKind.Sphere);
            SoftBody cube = Create(SoftBody.ShapeKind.Cube);
            try {
                sphere.transform.position = new Vector3(0f, 2f, 0f);
                sphere.transform.rotation = Quaternion.Euler(0f, 30f, 15f);
                cube.transform.position = new Vector3(3f, 2f, 0f);
                sphere.Initialize();
                cube.Initialize();

                Assert.AreSame(sphere.Simulation, cube.Simulation);
                Assert.AreEqual(2, sphere.Simulation.World.Bodies.Count);
                Vector2[] sphereUv = sphere.GetComponentInChildren<SoftBodySurface>().SharedMesh.uv;
                AssertMeshOnParticles(sphere);
                AssertMeshOnParticles(cube);

                sphere.Simulation.Collision.AddPlane(CollisionPlane.Horizontal(0f));
                for (int i = 0; i < 80; i++) sphere.Simulation.World.Step(0.02f);
                sphere.GetComponentInChildren<SoftBodySurface>().Publish();
                cube.GetComponentInChildren<SoftBodySurface>().Publish();

                Assert.AreEqual(sphereUv[0].x, sphere.GetComponentInChildren<SoftBodySurface>().SharedMesh.uv[0].x, 1e-5f);
                Assert.AreEqual(sphereUv.Length, sphere.GetComponentInChildren<SoftBodySurface>().SharedMesh.uv.Length);
                AssertLanded(sphere);
                AssertLanded(cube);
                AssertMeshOnParticles(sphere);
                AssertMeshOnParticles(cube);
                AssertOutward(sphere);

                sphere.transform.SetPositionAndRotation(new Vector3(8f, 1f, -3f), Quaternion.Euler(40f, 10f, 70f));
                sphere.GetComponentInChildren<SoftBodySurface>().Publish();
                AssertMeshOnParticles(sphere);
            } finally { Cleanup(); }
        }

        [Test]
        public void ForcesStiffnessAndGravityStayOnEachBody() {
            Cleanup();
            SoftBody heavy = Create(SoftBody.ShapeKind.Sphere);
            SoftBody weightless = Create(SoftBody.ShapeKind.Sphere);
            try {
                heavy.transform.position = new Vector3(0f, 2f, 0f);
                weightless.transform.position = new Vector3(4f, 2f, 0f);
                weightless.gravityMultiplier = 0f;
                heavy.Initialize();
                weightless.Initialize();

                float heavyStart = heavy.Body.CenterOfMass().Y;
                float lightStart = weightless.Body.CenterOfMass().Y;
                for (int i = 0; i < 8; i++) heavy.Simulation.World.Step(0.02f);

                Assert.Less(heavy.Body.CenterOfMass().Y, heavyStart - 0.05f);
                Assert.AreEqual(lightStart, weightless.Body.CenterOfMass().Y, 1e-4f);

                heavy.AddForce(new Vector3(0f, 20f, 0f));
                float force = 0f;
                for (int i = 0; i < heavy.Body.Particles.Length; i++) force += heavy.Body.Particles[i].Force.Y;
                Assert.AreEqual(20f, force, 1e-3f);

                heavy.SetKinematic(true);
                heavy.AddForce(new Vector3(5f, 0f, 0f));
                Assert.AreEqual(0f, heavy.Body.Particles[0].Force.X, 1e-6f);
                heavy.mass = 4f;
                heavy.ApplySettings();
                Assert.AreEqual(0f, heavy.Body.Particles[0].InverseMass, 1e-6f);
                heavy.SetKinematic(false);
                Assert.Greater(heavy.Body.Particles[0].InverseMass, 0f);

                heavy.SetStiffness(1f);
                float firm = SoftBody.ComplianceFor(1f);
                Assert.AreEqual(firm, FirstDistance(heavy).Compliance, firm * 1e-4f);
                heavy.SetStiffness(0f);
                Assert.AreEqual(SoftBody.ComplianceFor(0f), FirstDistance(heavy).Compliance, 1e-5f);

                heavy.color = Color.blue;
                heavy.ApplySettings();
                Assert.AreEqual(Color.blue, heavy.GetComponentInChildren<MeshRenderer>().sharedMaterial.color);

                heavy.Simulation.Pause();
                Assert.IsTrue(heavy.Simulation.Paused);
                heavy.Simulation.Resume();
                Assert.IsFalse(heavy.Simulation.Paused);
            } finally { Cleanup(); }
        }

        [Test]
        public void DisplayMeshCanBeAssignedBeforeInitializeAndSurvivesARebuild() {
            Cleanup();
            SoftBody body = Create(SoftBody.ShapeKind.Sphere);
            Mesh early = Placeholder();
            Mesh source = null;
            try {
                body.SetDisplayMesh(early, new[] { 0, 1, 2, 3 }, null);
                body.Initialize();
                Assert.AreSame(early, body.GetComponentInChildren<SoftBodySurface>().SourceMesh);
                int count = body.Body.Particles.Length;
                source = ParticleMesh(body);
                int[] triangles = (int[])source.triangles.Clone();
                Vector2 second = source.uv2[3];
                Vector3 sourceBefore = source.vertices[3];
                body.SetDisplayMesh(source, null, new[] { body.GetComponentInChildren<MeshRenderer>().sharedMaterial });

                SoftBodySurface surface = body.GetComponentInChildren<SoftBodySurface>();
                Mesh runtime = surface.SharedMesh;
                Assert.AreSame(source, surface.SourceMesh);
                Assert.AreEqual(count, runtime.vertexCount);
                Assert.AreEqual(second.x, runtime.uv2[3].x, 1e-5f);
                CollectionAssert.AreEqual(triangles, runtime.GetTriangles(0));

                Particle moved = body.Body.Particles[3];
                Vector3 before = runtime.vertices[3];
                moved.Position.X += 0.3f;
                body.Body.Particles[3] = moved;
                surface.Publish();
                Assert.AreEqual(before.x + 0.3f, runtime.vertices[3].x, 1e-4f);
                Assert.AreEqual(second.y, runtime.uv2[3].y, 1e-5f);
                CollectionAssert.AreEqual(triangles, runtime.GetTriangles(0));
                Assert.AreEqual(sourceBefore.x, source.vertices[3].x, 1e-5f);

                Vector3 sourceVertex = source.vertices[0];
                Mesh leaked = runtime;
                UnityEngine.Object.DestroyImmediate(body.gameObject);
                body = null;
                Assert.IsTrue(leaked == null);
                Assert.AreEqual(sourceVertex.x, source.vertices[0].x, 1e-5f);
            } finally {
                if(body != null) UnityEngine.Object.DestroyImmediate(body.gameObject);
                if(early != null) UnityEngine.Object.DestroyImmediate(early);
                if(source != null) UnityEngine.Object.DestroyImmediate(source);
                Cleanup();
            }
        }

        [Test]
        public void AParticleIndexOutsideTheBodyIsRejected() {
            Cleanup();
            SoftBody body = Create(SoftBody.ShapeKind.Cube);
            Mesh source = Placeholder();
            try {
                body.Initialize();
                ArgumentException error = Assert.Throws<ArgumentException>(() => body.SetDisplayMesh(source, new[] { 0, 1, 2, 99 }, null));
                StringAssert.Contains("99", error.Message);

                body.simulate = false;
                Vector3 pinned = body.transform.position;
                body.ApplySettings();
                for (int i = 0; i < 5; i++) body.Simulation.World.Step(0.02f);
                Assert.AreEqual(pinned.y, body.Body.CenterOfMass().Y, 1e-4f);
                Assert.AreEqual(0, body.Simulation.World.Bodies.Count);
            } finally {
                UnityEngine.Object.DestroyImmediate(body.gameObject);
                UnityEngine.Object.DestroyImmediate(source);
                Cleanup();
            }
        }

        static void AssertLanded(SoftBody body) {
            float minY = float.PositiveInfinity;
            bool nan = false;
            for (int i = 0; i < body.Body.Particles.Length; i++) {
                Vec3 position = body.Body.Particles[i].Position;
                if(position.HasNaN() || body.Body.Particles[i].Velocity.HasNaN()) nan = true;
                if(position.Y < minY) minY = position.Y;
            }

            Assert.IsFalse(nan);
            Assert.Greater(minY, -0.05f);
            Assert.Less(body.Body.CenterOfMass().Y, 1.2f);
        }

        static void AssertMeshOnParticles(SoftBody body) {
            SoftBodySurface surface = body.GetComponentInChildren<SoftBodySurface>();
            Vector3[] vertices = surface.SharedMesh.vertices;
            Vector3[] normals = surface.SharedMesh.normals;
            Assert.AreEqual(vertices.Length, normals.Length);
            for (int i = 0; i < vertices.Length; i++) {
                int particle = surface.ParticleOf(i);
                Vec3 position = body.Body.Particles[particle].Position;
                Vector3 world = surface.transform.TransformPoint(vertices[i]);
                Assert.AreEqual(position.X, world.x, 1e-3f);
                Assert.AreEqual(position.Y, world.y, 1e-3f);
                Assert.AreEqual(position.Z, world.z, 1e-3f);
                Assert.AreEqual(1f, normals[i].magnitude, 1e-3f);
                Assert.IsFalse(float.IsNaN(vertices[i].x));
            }
        }

        static void AssertOutward(SoftBody body) {
            SoftBodySurface surface = body.GetComponentInChildren<SoftBodySurface>();
            Vector3[] vertices = surface.SharedMesh.vertices;
            Vector3[] normals = surface.SharedMesh.normals;
            Vec3 center = body.Body.CenterOfMass();
            int aligned = 0;
            int counted = 0;
            for (int i = 0; i < vertices.Length; i++) {
                Vector3 worldPoint = surface.transform.TransformPoint(vertices[i]);
                Vector3 radial = worldPoint - new Vector3(center.X, center.Y, center.Z);
                if(radial.sqrMagnitude < 0.04f) continue;
                counted++;
                Vector3 worldNormal = surface.transform.TransformDirection(normals[i]);
                if(Vector3.Dot(radial.normalized, worldNormal) > 0.35f) aligned++;
            }

            Assert.Greater(counted, 0);
            Assert.Greater(aligned / (float)counted, 0.8f);
        }

        static DistanceConstraint FirstDistance(SoftBody body) {
            for (int i = 0; i < body.Body.Constraints.Count; i++) {
                if(body.Body.Constraints[i] is DistanceConstraint distance) return distance;
            }

            Assert.Fail("The body has no distance constraint.");
            return null;
        }

        static Mesh Placeholder() {
            var mesh = new Mesh();
            mesh.vertices = new[] { Vector3.zero, Vector3.right, Vector3.up, Vector3.forward };
            mesh.uv = new[] { Vector2.zero, Vector2.right, Vector2.up, Vector2.one };
            mesh.triangles = new[] { 0, 1, 2, 0, 2, 3 };
            return mesh;
        }

        static Mesh ParticleMesh(SoftBody body) {
            int count = body.Body.Particles.Length;
            var vertices = new Vector3[count];
            var uv = new Vector2[count];
            var uv2 = new Vector2[count];
            for (int i = 0; i < count; i++) {
                Vec3 position = body.Body.Particles[i].Position;
                vertices[i] = body.transform.InverseTransformPoint(new Vector3(position.X, position.Y, position.Z));
                uv[i] = new Vector2(i / (float)count, 0.25f);
                uv2[i] = new Vector2(0.1f * i, 0.5f);
            }

            var mesh = new Mesh { name = "Identity Display" };
            mesh.vertices = vertices;
            mesh.uv = uv;
            mesh.uv2 = uv2;
            mesh.triangles = (int[])body.Body.Triangles.Clone();
            return mesh;
        }

        static SoftBody Create(SoftBody.ShapeKind shape) {
            var host = new GameObject(shape.ToString());
            SoftBody body = host.AddComponent<SoftBody>();
            body.shape = shape;
            body.sphereSegments = 6;
            body.sphereStacks = 4;
            body.cubeCells = 1;
            body.initializeOnEnable = false;
            return body;
        }

        static void Cleanup() {
            SoftBody[] bodies = UnityEngine.Object.FindObjectsByType<SoftBody>(FindObjectsInactive.Include);
            for (int i = 0; i < bodies.Length; i++) { if(bodies[i] != null) UnityEngine.Object.DestroyImmediate(bodies[i].gameObject); }

            SoftBodySimulation[] worlds = UnityEngine.Object.FindObjectsByType<SoftBodySimulation>(FindObjectsInactive.Include);
            for (int i = 0; i < worlds.Length; i++) { if(worlds[i] != null) UnityEngine.Object.DestroyImmediate(worlds[i].gameObject); }
        }
    }
}
