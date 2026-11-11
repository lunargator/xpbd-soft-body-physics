using NUnit.Framework;
using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Tests {
    public class MassDistributionTests {
        [Test]
        public void AreaWeightsKeepTheTotalAndAvoidZeroMass() {
            var positions = new[] {
                new Vec3(0f, 0f, 0f),
                new Vec3(2f, 0f, 0f),
                new Vec3(0f, 2f, 0f),
                new Vec3(0.05f, 0.05f, 0f)
            };
            int[] triangles = { 0, 1, 2, 0, 1, 3 };
            var inverse = new float[4];
            MassDistribution.ByArea(inverse, positions, triangles, 4f);

            float total = 0f;
            for (int i = 0; i < inverse.Length; i++) {
                Assert.Greater(inverse[i], 0f);
                Assert.IsFalse(float.IsInfinity(inverse[i]));
                total += 1f / inverse[i];
            }

            Assert.AreEqual(4f, total, 1e-4f);
            Assert.Less(inverse[2], inverse[3]);
        }

        [Test]
        public void ADegenerateMeshFallsBackToAnEvenSplit() {
            var positions = new[] {
                new Vec3(0f, 0f, 0f),
                new Vec3(1f, 0f, 0f),
                new Vec3(2f, 0f, 0f)
            };
            int[] triangles = { 0, 1, 2 };
            var inverse = new float[3];
            MassDistribution.ByArea(inverse, positions, triangles, 3f);
            Assert.AreEqual(inverse[0], inverse[1], 1e-5f);
            Assert.AreEqual(inverse[1], inverse[2], 1e-5f);
            Assert.AreEqual(1f, 1f / inverse[0], 1e-4f);
        }

        [Test]
        public void ZeroTotalMassFixesEveryParticle() {
            var inverse = new float[2];
            MassDistribution.ByArea(inverse, null, null, 0f);
            Assert.AreEqual(0f, inverse[0]);
            Assert.AreEqual(0f, inverse[1]);
        }
    }
}
