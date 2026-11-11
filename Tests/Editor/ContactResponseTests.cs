using NUnit.Framework;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;

namespace SoftBodyPhysics.Tests {
    public class ContactResponseTests {
        [Test]
        public void EqualMassesKeepTheirMomentum() {
            BodyMaterial material = Still();
            DeformableBody left = SphereGeometry.Create(0.5f, 4, 2, material, new Vec3(0f, 0f, 0f));
            DeformableBody right = SphereGeometry.Create(0.5f, 4, 2, material, new Vec3(0.8f, 0f, 0f));
            left.ClosedSurface = false;
            right.ClosedSurface = false;
            SetVelocity(left, new Vec3(1f, 0f, 0f));
            SetVelocity(right, new Vec3(-1f, 0f, 0f));
            float before = MomentumX(left) + MomentumX(right);

            PhysicsWorld world = World(4);
            world.AddBody(left);
            world.AddBody(right);
            world.Step(1f / 60f);

            float after = MomentumX(left) + MomentumX(right);
            Assert.AreEqual(before, after, 0.75f, "before " + before + " after " + after);
            Assert.IsFalse(left.Particles[0].Position.HasNaN());
            Assert.IsFalse(right.Particles[0].Position.HasNaN());
        }

        [Test]
        public void ALightBodyMovesFartherThanAHeavyBody() {
            DeformableBody point = Point(0.2f, -0.15f, 0.2f, 1f);
            SetVelocity(point, new Vec3(0f, -1f, 0f));
            DeformableBody heavy = Triangle(0.05f);
            PhysicsWorld world = World(4);
            world.AddBody(point);
            world.AddBody(heavy);
            world.Step(1f / 60f);

            float pointMove = Abs(point.Particles[0].Position.Y - (-0.15f));
            float hostMove = Abs(MeanY(heavy));
            Assert.Greater(pointMove, hostMove * 4f, "point " + pointMove + " host " + hostMove);
            Assert.AreEqual(0f, PlaneDistance(heavy, point.Particles[0].Position), 2e-3f);
        }

        [Test]
        public void AFixedBodyDoesNotTakeTheCorrection() {
            DeformableBody point = Point(0.2f, -0.2f, 0.2f, 1f);
            DeformableBody pinned = Triangle(0f);
            PhysicsWorld world = World(2);
            world.AddBody(point);
            world.AddBody(pinned);
            world.Step(1f / 60f);

            Assert.AreEqual(0f, pinned.Particles[0].Position.Y, 1e-5f);
            Assert.AreEqual(0f, pinned.Particles[1].Position.Y, 1e-5f);
            Assert.AreEqual(0f, pinned.Particles[2].Position.Y, 1e-5f);
            Assert.AreEqual(0f, point.Particles[0].Position.Y, 1e-3f);
        }

        [Test]
        public void OppositeVelocitiesKeepTheCombinedMomentum() {
            DeformableBody point = Point(0.2f, 0.05f, 0.2f, 1f);
            DeformableBody host = Triangle(1f);
            SetVelocity(point, new Vec3(0f, -2f, 0f));
            SetVelocity(host, new Vec3(0f, 0.5f, 0f));
            float before = MomentumY(point) + MomentumY(host);

            PhysicsWorld world = World(4);
            world.AddBody(point);
            world.AddBody(host);
            for (int i = 0; i < 3; i++) world.Step(1f / 60f);

            float after = MomentumY(point) + MomentumY(host);
            Assert.AreEqual(before, after, 0.15f, "before " + before + " after " + after);
        }

        [Test]
        public void TwoPointsShareOneTriangle() {
            DeformableBody first = Point(0.2f, -0.1f, 0.2f, 1f);
            DeformableBody second = Point(0.55f, -0.12f, 0.15f, 1f);
            DeformableBody host = Triangle(1f);
            PhysicsWorld world = World(4);
            world.AddBody(first);
            world.AddBody(second);
            world.AddBody(host);
            world.Step(1f / 60f);

            Assert.AreEqual(0f, PlaneDistance(host, first.Particles[0].Position), 0.03f);
            Assert.AreEqual(0f, PlaneDistance(host, second.Particles[0].Position), 0.03f);
            Assert.IsFalse(host.Particles[0].Position.HasNaN());
            Assert.Less(MeanY(host), -0.005f);
        }

        [Test]
        public void RestingContactDoesNotGainHeight() {
            BodyMaterial material = Still();
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 0.4f;
            var point = new DeformableBody(1, material);
            point.Particles[0] = Placed(0.25f, 0.2f, 0.25f, 1f);
            DeformableBody floor = Triangle(0f);
            PhysicsWorld world = World(4);
            world.AddBody(point);
            world.AddBody(floor);
            for (int i = 0; i < 40; i++) world.Step(1f / 60f);

            float rested = point.Particles[0].Position.Y;
            float speed = point.Particles[0].Velocity.Length;
            Assert.Less(Abs(rested), 0.03f, "height " + rested);
            Assert.Less(speed, 0.35f, "speed " + speed);
            for (int i = 0; i < 30; i++) world.Step(1f / 60f);

            Assert.Less(point.Particles[0].Position.Y, rested + 0.015f);
            Assert.Less(point.Particles[0].Velocity.Length, 0.35f);
        }

        static PhysicsWorld World(int iterations) {
            return new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 1f / 60f,
                SolverIterations = iterations,
                MaxStepsPerFrame = 1
            });
        }

        static BodyMaterial Still() {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = Vec3.Zero;
            material.Damping = 0f;
            material.Friction = 0f;
            material.Restitution = 0f;
            material.Compliance = 0f;
            return material;
        }

        static DeformableBody Triangle(float inverseMass) {
            var body = new DeformableBody(3, Still());
            body.Particles[0] = Placed(0f, 0f, 0f, inverseMass);
            body.Particles[1] = Placed(1f, 0f, 0f, inverseMass);
            body.Particles[2] = Placed(0f, 0f, 1f, inverseMass);
            body.Triangles = new[] { 0, 2, 1 };
            body.ClosedSurface = false;
            body.CaptureRestPositions();
            return body;
        }

        static DeformableBody Point(float x, float y, float z, float inverseMass) {
            var body = new DeformableBody(1, Still());
            body.Particles[0] = Placed(x, y, z, inverseMass);
            body.CaptureRestPositions();
            return body;
        }

        static Particle Placed(float x, float y, float z, float inverseMass) {
            var position = new Vec3(x, y, z);
            return new Particle {
                Position = position,
                Previous = position,
                InverseMass = inverseMass
            };
        }

        static void SetVelocity(DeformableBody body, Vec3 velocity) {
            for (int i = 0; i < body.Particles.Length; i++) {
                Particle particle = body.Particles[i];
                particle.Velocity = velocity;
                body.Particles[i] = particle;
            }
        }

        static float MomentumX(DeformableBody body) { return Momentum(body, 0); }

        static float MomentumY(DeformableBody body) { return Momentum(body, 1); }

        static float Momentum(DeformableBody body, int axis) {
            float sum = 0f;
            for (int i = 0; i < body.Particles.Length; i++) {
                Particle particle = body.Particles[i];
                if(particle.InverseMass <= 0f) continue;
                float speed = axis == 0 ? particle.Velocity.X : particle.Velocity.Y;
                sum += speed / particle.InverseMass;
            }

            return sum;
        }

        static float MeanY(DeformableBody body) {
            float sum = 0f;
            for (int i = 0; i < body.Particles.Length; i++) sum += body.Particles[i].Position.Y;
            return body.Particles.Length == 0 ? 0f : sum / body.Particles.Length;
        }

        static float PlaneDistance(DeformableBody surface, Vec3 point) {
            Vec3 a = surface.Particles[surface.Triangles[0]].Position;
            Vec3 b = surface.Particles[surface.Triangles[1]].Position;
            Vec3 c = surface.Particles[surface.Triangles[2]].Position;
            Vec3 normal = Vec3.Cross(b - a, c - a);
            float length = normal.Length;
            if(length < 1e-8f) return 0f;
            return Vec3.Dot(normal * (1f / length), point - a);
        }

        static float Abs(float value) { return value < 0f ? -value : value; }
    }
}
