using SoftBodyPhysics.Unity;
using NUnit.Framework;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;

namespace SoftBodyPhysics.Tests {
    public class CubeGeometryTests {
        [Test]
        public void CubeIsAClosedOutwardSurface() {
            var center = new Vec3(1f, 2f, -4f);
            DeformableBody body = CubeGeometry.Create(1f, 1, BodyMaterial.Default, center);

            Assert.AreEqual(8, body.Particles.Length);
            Assert.IsTrue(body.ClosedSurface);
            Assert.AreEqual(12 * 3, body.Triangles.Length);
            for (int t = 0; t < body.Triangles.Length; t += 3) {
                Vec3 a = body.Particles[body.Triangles[t]].Position;
                Vec3 b = body.Particles[body.Triangles[t + 1]].Position;
                Vec3 c = body.Particles[body.Triangles[t + 2]].Position;
                Vec3 normal = Cross(b - a, c - a);
                Assert.Greater(Vec3.Dot(normal, a - center), 0f);
            }

            float volume = VolumeConstraint.SignedVolume(body.RestPositions, body.Triangles);
            Assert.AreEqual(1f, volume, 1e-4f);
        }

        [Test]
        public void FinerCubeKeepsItsVolume() {
            DeformableBody body = CubeGeometry.Create(2f, 2, BodyMaterial.Default, Vec3.Zero);
            Assert.AreEqual(26, body.Particles.Length);
            float volume = VolumeConstraint.SignedVolume(body.RestPositions, body.Triangles);
            Assert.AreEqual(8f, volume, 1e-3f);
        }

        [Test]
        public void RejectsADegenerateCube() {
            Assert.Throws<System.ArgumentOutOfRangeException>(() => CubeGeometry.Create(0f, 1, BodyMaterial.Default, Vec3.Zero));
            Assert.Throws<System.ArgumentOutOfRangeException>(() => CubeGeometry.Create(1f, 0, BodyMaterial.Default, Vec3.Zero));
        }

        static Vec3 Cross(Vec3 u, Vec3 v) {
            return new Vec3(
                u.Y * v.Z - u.Z * v.Y,
                u.Z * v.X - u.X * v.Z,
                u.X * v.Y - u.Y * v.X);
        }
    }
}
