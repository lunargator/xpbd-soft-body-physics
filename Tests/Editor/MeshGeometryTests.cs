using System;
using NUnit.Framework;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;
using SoftBodyPhysics.Unity;

namespace SoftBodyPhysics.Tests {
    public class MeshGeometryTests {
        [Test]
        public void TetrahedronIsClosedAndKeepsItsVolume() {
            MeshGeometry.Build build = MeshGeometry.Create(TetraVertices(), TetraTriangles(), 0f, Material(), new Vec3(0f, 0.45f, 0f));
            Assert.AreEqual(4, build.Body.Particles.Length);
            Assert.IsTrue(build.Body.ClosedSurface);
            Assert.AreEqual(4, build.VertexToParticle.Length);
            float rest = VolumeConstraint.SignedVolume(build.Body.RestPositions, build.Body.Triangles);
            Assert.Greater(rest, 0f);
            SoftBodyShapes.FinishSurface(build.Body, build.Body.Material.Compliance);

            var world = new PhysicsWorld(new PhysicsSettings { FixedDeltaTime = 0.02f, SolverIterations = 4, MaxStepsPerFrame = 1 });
            world.Collision.AddPlane(SoftBodyPhysics.Collision.CollisionPlane.Horizontal(0f));
            world.AddBody(build.Body);
            for (int step = 0; step < 40; step++) world.Step(0.02f);

            float volume = VolumeConstraint.SignedVolume(build.Body.Particles, build.Body.Triangles);
            Assert.Greater(volume, rest * 0.55f);
            Assert.Less(volume, rest * 1.45f);
            AssertNoNaN(build.Body);
            Assert.Greater(Extent(build.Body), 0.4f);
        }

        [Test]
        public void PyramidIsADifferentClosedShape() {
            var vertices = new[] {
                new Vec3(0f, 0f, 0f),
                new Vec3(1f, 0f, 0f),
                new Vec3(1f, 0f, 1f),
                new Vec3(0f, 0f, 1f),
                new Vec3(0.5f, 1f, 0.5f)
            };
            var triangles = new[] {
                0, 2, 1, 0, 3, 2,
                0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4
            };
            MeshGeometry.Build build = MeshGeometry.Create(vertices, triangles, 0f, Material(), new Vec3(2f, 0f, 0f));
            Assert.AreEqual(5, build.Body.Particles.Length);
            Assert.IsTrue(build.Body.ClosedSurface);
            Assert.Greater(VolumeConstraint.SignedVolume(build.Body.RestPositions, build.Body.Triangles), 0f);
            Assert.Greater(build.Body.Particles[0].Position.X, 1.5f);
        }

        [Test]
        public void OpenQuadDoesNotReceiveAVolumeConstraint() {
            var vertices = new[] {
                new Vec3(0f, 0f, 0f),
                new Vec3(1f, 0f, 0f),
                new Vec3(1f, 1f, 0f),
                new Vec3(0f, 1f, 0f)
            };
            var triangles = new[] { 0, 1, 2, 0, 2, 3 };
            MeshGeometry.Build build = MeshGeometry.Create(vertices, triangles, 0f, Material(), Vec3.Zero);
            Assert.IsFalse(build.Body.ClosedSurface);
            SoftBodyShapes.FinishSurface(build.Body, 0.001f);
            Assert.AreEqual(0, Count<VolumeConstraint>(build.Body));
            Assert.Greater(Count<DistanceConstraint>(build.Body), 0);
            Assert.AreEqual(1, Count<ShapeMatchingConstraint>(build.Body));
        }

        [Test]
        public void DuplicateVerticesWeldAndACoarseCellMergesNeighbors() {
            Vec3[] tetra = TetraVertices();
            var duplicated = new Vec3[tetra.Length + 1];
            for (int i = 0; i < tetra.Length; i++) duplicated[i] = tetra[i];
            duplicated[tetra.Length] = tetra[0];
            MeshGeometry.Build welded = MeshGeometry.Create(duplicated, WithExtraCorner(TetraTriangles(), tetra.Length), 0f, Material(), Vec3.Zero);
            Assert.AreEqual(4, welded.Body.Particles.Length);
            Assert.AreEqual(welded.VertexToParticle[0], welded.VertexToParticle[tetra.Length]);

            var sheet = new[] {
                new Vec3(0f, 0f, 0f),
                new Vec3(2f, 0f, 0f),
                new Vec3(2f, 2f, 0f),
                new Vec3(0f, 2f, 0f),
                new Vec3(0.05f, 0.02f, 0f)
            };
            MeshGeometry.Build coarse = MeshGeometry.Create(sheet, new[] { 0, 1, 2, 0, 2, 3 }, 0.2f, Material(), Vec3.Zero);
            Assert.AreEqual(4, coarse.Body.Particles.Length);
            Assert.AreEqual(coarse.VertexToParticle[0], coarse.VertexToParticle[4]);
        }

        [Test]
        public void BadMeshesExplainThemselves() {
            var exception = Assert.Throws<ArgumentException>(() => MeshGeometry.Create(new Vec3[0], TetraTriangles(), 0f, Material(), Vec3.Zero));
            StringAssert.Contains("no vertices", exception.Message);

            exception = Assert.Throws<ArgumentException>(() => MeshGeometry.Create(TetraVertices(), new[] { 0, 1, 9 }, 0f, Material(), Vec3.Zero));
            StringAssert.Contains("9", exception.Message);

            exception = Assert.Throws<ArgumentException>(() => MeshGeometry.Create(TetraVertices(), new[] { 0, 1 }, 0f, Material(), Vec3.Zero));
            StringAssert.Contains("no triangles", exception.Message);
        }

        static BodyMaterial Material() {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 0.4f;
            material.Compliance = 0.0004f;
            return material;
        }

        static Vec3[] TetraVertices() {
            return new[] {
                new Vec3(0f, 0f, 0f),
                new Vec3(1f, 0f, 0f),
                new Vec3(0f, 1f, 0f),
                new Vec3(0f, 0f, 1f)
            };
        }

        static int[] TetraTriangles() { return new[] { 0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3 }; }

        static int[] WithExtraCorner(int[] triangles, int extra) {
            var copy = new int[triangles.Length + 3];
            for (int i = 0; i < triangles.Length; i++) copy[i] = triangles[i];
            copy[triangles.Length] = 0;
            copy[triangles.Length + 1] = 1;
            copy[triangles.Length + 2] = extra;
            return copy;
        }

        static int Count<T>(DeformableBody body) {
            int count = 0;
            for (int i = 0; i < body.Constraints.Count; i++) { if(body.Constraints[i] is T) count++; }

            return count;
        }

        static void AssertNoNaN(DeformableBody body) {
            for (int i = 0; i < body.Particles.Length; i++) Assert.IsFalse(body.Particles[i].Position.HasNaN());
        }

        static float Extent(DeformableBody body) {
            Vec3 min = body.Particles[0].Position;
            Vec3 max = min;
            for (int i = 1; i < body.Particles.Length; i++) {
                Vec3 position = body.Particles[i].Position;
                min = new Vec3(Math.Min(min.X, position.X), Math.Min(min.Y, position.Y), Math.Min(min.Z, position.Z));
                max = new Vec3(Math.Max(max.X, position.X), Math.Max(max.Y, position.Y), Math.Max(max.Z, position.Z));
            }

            return (max - min).Length;
        }
    }
}
