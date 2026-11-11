using SoftBodyPhysics.Unity;
using System;
using System.Collections.Generic;
using NUnit.Framework;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;

namespace SoftBodyPhysics.Tests {
    public class SphereGeometryTests {
        [Test]
        public void SphereIsAClosedMeshOfParticles() {
            const int segments = 6;
            const int stacks = 4;
            const float radius = 1.25f;
            var center = new Vec3(2f, -3f, 4f);
            DeformableBody body = Sphere(radius, segments, stacks, center);

            int rings = stacks - 1;
            Assert.AreEqual(rings * segments + 2, body.Particles.Length);
            Assert.AreEqual(body.Particles.Length, body.RestPositions.Length);
            Assert.AreEqual((segments * 2 + (stacks - 2) * segments * 2) * 3, body.Triangles.Length);

            var used = new bool[body.Particles.Length];
            var edgeUses = new Dictionary<long, int>();
            for (int t = 0; t < body.Triangles.Length; t += 3) {
                int i = body.Triangles[t];
                int j = body.Triangles[t + 1];
                int k = body.Triangles[t + 2];
                Assert.That(i, Is.InRange(0, body.Particles.Length - 1));
                Assert.That(j, Is.InRange(0, body.Particles.Length - 1));
                Assert.That(k, Is.InRange(0, body.Particles.Length - 1));
                used[i] = used[j] = used[k] = true;

                Vec3 a = body.Particles[i].Position;
                Vec3 normal = Cross(body.Particles[j].Position - a, body.Particles[k].Position - a);
                Assert.Greater(Vec3.Dot(normal, a - center), 0f);
                CountEdge(edgeUses, i, j);
                CountEdge(edgeUses, j, k);
                CountEdge(edgeUses, k, i);
            }

            for (int i = 0; i < used.Length; i++) Assert.IsTrue(used[i]);
            Assert.AreEqual(body.Triangles.Length / 2, edgeUses.Count);
            foreach (int uses in edgeUses.Values) Assert.AreEqual(2, uses);
        }

        [Test]
        public void VerticesLieOnTheSphereAndMatchTheirEdges() {
            const float radius = 0.8f;
            var center = new Vec3(-1f, 2f, 0.5f);
            DeformableBody body = Sphere(radius, 8, 5, center);

            Vec3 sum = Vec3.Zero;
            for (int i = 0; i < body.Particles.Length; i++) {
                Vec3 offset = body.Particles[i].Position - center;
                Assert.AreEqual(radius, offset.Length, 1e-4f);
                Assert.AreEqual(body.Particles[i].Position.X, body.RestPositions[i].X, 1e-6f);
                Assert.AreEqual(body.Particles[i].Position.Y, body.RestPositions[i].Y, 1e-6f);
                Assert.AreEqual(body.Particles[i].Position.Z, body.RestPositions[i].Z, 1e-6f);
                Assert.AreEqual(1f, body.Particles[i].InverseMass, 1e-6f);
                Assert.IsFalse(body.Particles[i].Position.HasNaN());
                sum += body.Particles[i].Position;
            }

            Vec3 average = sum * (1f / body.Particles.Length);
            Assert.AreEqual(center.X, average.X, 1e-4f);
            Assert.AreEqual(center.Y, average.Y, 1e-4f);
            Assert.AreEqual(center.Z, average.Z, 1e-4f);

            var edges = Edges(body);
            Assert.AreEqual(edges.Count, body.Constraints.Count);
            for (int i = 0; i < body.Constraints.Count; i++) {
                var constraint = (DistanceConstraint)body.Constraints[i];
                long key = EdgeKey(constraint.IndexA, constraint.IndexB);
                Assert.IsTrue(edges.Contains(key));
                float length = (body.Particles[constraint.IndexA].Position - body.Particles[constraint.IndexB].Position).Length;
                Assert.AreEqual(length, constraint.RestLength, 1e-5f);
                edges.Remove(key);
            }

            Assert.AreEqual(0, edges.Count);
        }

        [Test]
        public void RestSphereDoesNotDrift() {
            DeformableBody body = Sphere(1f, 5, 3, new Vec3(0f, 1f, -6f));
            var copy = new Vec3[body.Particles.Length];
            for (int i = 0; i < copy.Length; i++) copy[i] = body.Particles[i].Position;

            var world = new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 0.02f,
                SolverIterations = 4,
                MaxStepsPerFrame = 8
            });
            world.AddBody(body);
            world.Tick(0.1f);

            for (int i = 0; i < copy.Length; i++) {
                Assert.AreEqual(copy[i].X, body.Particles[i].Position.X, 1e-5f);
                Assert.AreEqual(copy[i].Y, body.Particles[i].Position.Y, 1e-5f);
                Assert.AreEqual(copy[i].Z, body.Particles[i].Position.Z, 1e-5f);
                Assert.IsFalse(body.Particles[i].Position.HasNaN());
            }
        }

        [Test]
        public void RejectsADegenerateSphere() {
            BodyMaterial material = BodyMaterial.Default;
            Assert.Throws<ArgumentOutOfRangeException>(() => SphereGeometry.Create(0f, 4, 3, material, Vec3.Zero));
            Assert.Throws<ArgumentOutOfRangeException>(() => SphereGeometry.Create(1f, 2, 3, material, Vec3.Zero));
            Assert.Throws<ArgumentOutOfRangeException>(() => SphereGeometry.Create(1f, 4, 1, material, Vec3.Zero));
        }

        static DeformableBody Sphere(float radius, int segments, int stacks, Vec3 center) {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = Vec3.Zero;
            material.Damping = 0f;
            material.Compliance = 0f;
            return SphereGeometry.Create(radius, segments, stacks, material, center);
        }

        static HashSet<long> Edges(DeformableBody body) {
            var edges = new HashSet<long>();
            for (int t = 0; t < body.Triangles.Length; t += 3) {
                edges.Add(EdgeKey(body.Triangles[t], body.Triangles[t + 1]));
                edges.Add(EdgeKey(body.Triangles[t + 1], body.Triangles[t + 2]));
                edges.Add(EdgeKey(body.Triangles[t + 2], body.Triangles[t]));
            }

            return edges;
        }

        static void CountEdge(Dictionary<long, int> uses, int a, int b) {
            long key = EdgeKey(a, b);
            uses.TryGetValue(key, out int count);
            uses[key] = count + 1;
        }

        static long EdgeKey(int a, int b) {
            int min = a < b ? a : b;
            int max = a < b ? b : a;
            return ((long)min << 32) | (uint)max;
        }

        static Vec3 Cross(Vec3 u, Vec3 v) {
            return new Vec3(
                u.Y * v.Z - u.Z * v.Y,
                u.Z * v.X - u.X * v.Z,
                u.X * v.Y - u.Y * v.X);
        }
    }
}
