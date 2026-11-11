using SoftBodyPhysics.Unity;
using NUnit.Framework;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;

namespace SoftBodyPhysics.Tests {
    public class ShapeAndVolumeTests {
        [Test]
        public void RigidMotionIsLeftAlone() {
            DeformableBody translated = Octahedron();
            Shift(translated, new Vec3(3f, -2f, 1f));
            DeformableBody rotated = Octahedron();
            RotateAroundY(rotated);

            SolveShape(translated, compliance: 0f, iterations: 1);
            SolveShape(rotated, compliance: 0f, iterations: 1);

            AssertClose(new Vec3(3f, -1f, 1f), translated.Particles[0].Position);
            AssertClose(new Vec3(0f, 0f, -1f), rotated.Particles[1].Position);
            Assert.IsFalse(translated.Particles[0].Position.HasNaN());
            Assert.IsFalse(rotated.Particles[0].Position.HasNaN());
        }

        [Test]
        public void StiffShapeMatchingUndoesASquash() {
            DeformableBody body = Octahedron();
            SquashY(body, 0.5f);
            SolveShape(body, compliance: 0f, iterations: 1);

            Assert.AreEqual(1f, body.Particles[0].Position.Y, 1e-3f);
            Assert.AreEqual(-1f, body.Particles[body.Particles.Length - 1].Position.Y, 1e-3f);
            Assert.AreEqual(1f, body.Particles[1].Position.X, 1e-3f);
        }

        [Test]
        public void ComplianceLeavesSomeOfTheSquash() {
            DeformableBody body = Octahedron();
            SquashY(body, 0.5f);
            SolveShape(body, compliance: 0.0004f, iterations: 1);

            Assert.AreEqual(0.75f, body.Particles[0].Position.Y, 1e-3f);
        }

        [Test]
        public void ShapeMatchingDoesNotMoveAFixedParticle() {
            DeformableBody body = Octahedron();
            Particle pinned = body.Particles[1];
            pinned.InverseMass = 0f;
            body.Particles[1] = pinned;
            Vec3 held = pinned.Position;
            SquashY(body, 0.5f);
            held = body.Particles[1].Position;

            SolveShape(body, compliance: 0f, iterations: 1);

            AssertClose(held, body.Particles[1].Position);
        }

        [Test]
        public void OctahedronVolumeMatchesTheClosedSurface() {
            DeformableBody body = Octahedron();
            float volume = VolumeConstraint.SignedVolume(body.RestPositions, body.Triangles);
            Assert.AreEqual(4f / 3f, volume, 1e-4f);

            Shift(body, new Vec3(5f, -4f, 2f));
            Assert.AreEqual(4f / 3f, VolumeConstraint.SignedVolume(body.Particles, body.Triangles), 1e-3f);
        }

        [Test]
        public void VolumeConstraintRestoresAUniformShrink() {
            DeformableBody body = Octahedron();
            Scale(body, 0.5f);
            Assert.AreEqual((4f / 3f) * 0.125f, VolumeConstraint.SignedVolume(body.Particles, body.Triangles), 1e-4f);

            SolveVolume(body, compliance: 0f, iterations: 8);

            Assert.AreEqual(4f / 3f, VolumeConstraint.SignedVolume(body.Particles, body.Triangles), 1e-3f);
            Assert.AreEqual(1f, body.Particles[0].Position.Y, 1e-3f);
            Assert.IsFalse(body.Particles[2].Position.HasNaN());
        }

        [Test]
        public void VolumeGradientMatchesAFiniteDifference() {
            var positions = new[] {
                new Vec3(0.1f, 0.2f, -0.3f),
                new Vec3(1.2f, 0.1f, 0.2f),
                new Vec3(0.2f, 0.05f, 1.1f),
                new Vec3(0.4f, 1.3f, 0.25f)
            };
            int[] triangles = { 0, 2, 1, 0, 1, 3, 1, 2, 3, 0, 3, 2 };
            float volume = VolumeConstraint.SignedVolume(positions, triangles);
            Assert.Greater(Abs(volume), 0.05f);

            var gradient = new Vec3[4];
            VolumeConstraint.AccumulateGradient(positions, triangles, gradient);
            Vec3 sum = gradient[0] + gradient[1] + gradient[2] + gradient[3];
            Assert.Less(sum.Length, 1e-4f);

            const float step = 1e-4f;
            for (int vertex = 0; vertex < 4; vertex++) {
                for (int axis = 0; axis < 3; axis++) {
                    Vec3[] plus = Copy(positions);
                    Vec3[] minus = Copy(positions);
                    plus[vertex] = Shift(plus[vertex], axis, step);
                    minus[vertex] = Shift(minus[vertex], axis, -step);
                    float numerical = (VolumeConstraint.SignedVolume(plus, triangles) - VolumeConstraint.SignedVolume(minus, triangles)) / (2f * step);
                    float analytic = Axis(gradient[vertex], axis);
                    Assert.AreEqual(numerical, analytic, 1e-3f, "vertex " + vertex + " axis " + axis);
                }
            }
        }

        [Test]
        public void AnInvertedBodyIsNotPulledBackByVolume() {
            DeformableBody body = Tetrahedron();
            var constraint = new VolumeConstraint(body, 0f);
            Particle flipped = body.Particles[3];
            flipped.Position = new Vec3(0.2f, -1f, 0.2f);
            flipped.Previous = flipped.Position;
            body.Particles[3] = flipped;
            Assert.Less(constraint.RestVolume * VolumeConstraint.SignedVolume(body.Particles, body.Triangles), 0f);
            Vec3 held = body.Particles[3].Position;
            constraint.Solve(body, 0.02f);
            AssertClose(held, body.Particles[3].Position);
        }

        [Test]
        public void ADegenerateFaceDoesNotMakeTheVolumeGradientNaN() {
            var positions = new[] {
                new Vec3(0f, 0f, 0f),
                new Vec3(1f, 0f, 0f),
                new Vec3(2f, 0f, 0f),
                new Vec3(0f, 1f, 0f),
                new Vec3(0f, 0f, 1f)
            };
            int[] triangles = { 0, 1, 2, 0, 3, 4 };
            var gradient = new Vec3[5];
            VolumeConstraint.AccumulateGradient(positions, triangles, gradient);
            for (int i = 0; i < gradient.Length; i++) Assert.IsFalse(gradient[i].HasNaN());
            Assert.IsFalse(float.IsNaN(VolumeConstraint.SignedVolume(positions, triangles)));
        }

        [Test]
        public void OneRigidFitPerStepStaysTheShippedPath() {
            float extent = PerturbedExtent();
            Assert.IsFalse(float.IsNaN(extent));
            Assert.Greater(extent, 0.2f);
        }

        [Test]
        public void VolumeConstraintDoesNotMoveAFixedParticle() {
            DeformableBody body = Octahedron();
            Particle pinned = body.Particles[1];
            pinned.InverseMass = 0f;
            body.Particles[1] = pinned;
            Scale(body, 0.5f);
            Vec3 held = body.Particles[1].Position;

            SolveVolume(body, compliance: 0f, iterations: 4);

            AssertClose(held, body.Particles[1].Position);
        }

        static float PerturbedExtent() {
            DeformableBody body = Octahedron();
            SquashY(body, 0.6f);
            var constraint = new ShapeMatchingConstraint(body, 0.001f);
            for (int iteration = 0; iteration < 4; iteration++) {
                if(iteration == 2) {
                    Particle pushed = body.Particles[1];
                    pushed.Position += new Vec3(0.35f, 0f, 0f);
                    body.Particles[1] = pushed;
                }

                constraint.Solve(body, 0.02f);
            }

            return Abs(body.Particles[0].Position.Y - body.Particles[body.Particles.Length - 1].Position.Y);
        }

        static DeformableBody Tetrahedron() {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = Vec3.Zero;
            var body = new DeformableBody(4, material);
            body.Particles[0] = new Particle { Position = new Vec3(0f, 0f, 0f), Previous = new Vec3(0f, 0f, 0f), InverseMass = 1f };
            body.Particles[1] = new Particle { Position = new Vec3(1f, 0f, 0f), Previous = new Vec3(1f, 0f, 0f), InverseMass = 1f };
            body.Particles[2] = new Particle { Position = new Vec3(0f, 0f, 1f), Previous = new Vec3(0f, 0f, 1f), InverseMass = 1f };
            body.Particles[3] = new Particle { Position = new Vec3(0f, 1f, 0f), Previous = new Vec3(0f, 1f, 0f), InverseMass = 1f };
            body.Triangles = new[] { 0, 2, 1, 0, 1, 3, 1, 2, 3, 0, 3, 2 };
            body.ClosedSurface = true;
            body.CaptureRestPositions();
            return body;
        }

        static Vec3[] Copy(Vec3[] source) {
            var copy = new Vec3[source.Length];
            for (int i = 0; i < source.Length; i++) copy[i] = source[i];
            return copy;
        }

        static Vec3 Shift(Vec3 value, int axis, float amount) {
            if(axis == 0) return new Vec3(value.X + amount, value.Y, value.Z);
            if(axis == 1) return new Vec3(value.X, value.Y + amount, value.Z);
            return new Vec3(value.X, value.Y, value.Z + amount);
        }

        static float Axis(Vec3 value, int axis) {
            if(axis == 0) return value.X;
            if(axis == 1) return value.Y;
            return value.Z;
        }

        static float Abs(float value) { return value < 0f ? -value : value; }

        static DeformableBody Octahedron() {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = Vec3.Zero;
            material.Damping = 0f;
            material.Compliance = 0f;
            return SphereGeometry.Create(1f, 4, 2, material, Vec3.Zero);
        }

        static void SolveShape(DeformableBody body, float compliance, int iterations) {
            body.Constraints.Clear();
            body.AddConstraint(new ShapeMatchingConstraint(body, compliance));
            Step(body, iterations);
        }

        static void SolveVolume(DeformableBody body, float compliance, int iterations) {
            body.Constraints.Clear();
            body.AddConstraint(new VolumeConstraint(body, compliance));
            Step(body, iterations);
        }

        static void Step(DeformableBody body, int iterations) {
            var world = new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 0.02f,
                SolverIterations = iterations,
                MaxStepsPerFrame = 1
            });
            world.AddBody(body);
            world.Tick(0.02f);
        }

        static void SquashY(DeformableBody body, float scale) {
            for (int i = 0; i < body.Particles.Length; i++) {
                Particle particle = body.Particles[i];
                particle.Position.Y *= scale;
                particle.Previous = particle.Position;
                body.Particles[i] = particle;
            }
        }

        static void Scale(DeformableBody body, float scale) {
            for (int i = 0; i < body.Particles.Length; i++) {
                Particle particle = body.Particles[i];
                particle.Position *= scale;
                particle.Previous = particle.Position;
                body.Particles[i] = particle;
            }
        }

        static void Shift(DeformableBody body, Vec3 offset) {
            for (int i = 0; i < body.Particles.Length; i++) {
                Particle particle = body.Particles[i];
                particle.Position += offset;
                particle.Previous = particle.Position;
                body.Particles[i] = particle;
            }
        }

        static void RotateAroundY(DeformableBody body) {
            for (int i = 0; i < body.Particles.Length; i++) {
                Particle particle = body.Particles[i];
                Vec3 position = particle.Position;
                particle.Position = new Vec3(position.Z, position.Y, -position.X);
                particle.Previous = particle.Position;
                body.Particles[i] = particle;
            }
        }

        static void AssertClose(Vec3 expected, Vec3 actual) {
            Assert.AreEqual(expected.X, actual.X, 1e-3f);
            Assert.AreEqual(expected.Y, actual.Y, 1e-3f);
            Assert.AreEqual(expected.Z, actual.Z, 1e-3f);
        }
    }
}
