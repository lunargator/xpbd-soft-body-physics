using SoftBodyPhysics.Unity;
using NUnit.Framework;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;

namespace SoftBodyPhysics.Tests {
    public class ImpulseAttachmentTests {
        [Test]
        public void ImpulseChangesOnlyThatParticlesVelocity() {
            BodyMaterial material = Still();
            var body = new DeformableBody(2, material);
            body.Particles[0] = Placed(0f, 0f, 0f, 2f);
            body.Particles[1] = Placed(1f, 0f, 0f, 1f);

            body.ApplyImpulse(0, new Vec3(1f, 0f, 0f));
            body.ApplyImpulse(1, new Vec3(0f, 3f, -1f));
            body.ApplyImpulse(1, new Vec3(float.NaN, 0f, 0f));
            body.ApplyImpulse(-1, new Vec3(4f, 0f, 0f));
            body.ApplyImpulse(4, new Vec3(4f, 0f, 0f));

            Assert.AreEqual(2f, body.Particles[0].Velocity.X, 1e-6f);
            Assert.AreEqual(0f, body.Particles[0].Velocity.Y, 1e-6f);
            Assert.AreEqual(0f, body.Particles[1].Velocity.X, 1e-6f);
            Assert.AreEqual(3f, body.Particles[1].Velocity.Y, 1e-6f);
            Assert.AreEqual(-1f, body.Particles[1].Velocity.Z, 1e-6f);
            Assert.AreEqual(0f, body.Particles[0].Position.X, 1e-6f);
            Assert.AreEqual(1f, body.Particles[1].Position.X, 1e-6f);
        }

        [Test]
        public void FixedParticleIgnoresAnImpulse() {
            var body = new DeformableBody(1, Still());
            body.Particles[0] = Placed(2f, 3f, 4f, 0f);
            body.ApplyImpulse(0, new Vec3(5f, -2f, 1f));

            Assert.AreEqual(0f, body.Particles[0].Velocity.X, 1e-6f);
            Assert.AreEqual(0f, body.Particles[0].Velocity.Y, 1e-6f);
            Assert.AreEqual(2f, body.Particles[0].Position.X, 1e-6f);
            Assert.AreEqual(3f, body.Particles[0].Position.Y, 1e-6f);
            Assert.AreEqual(4f, body.Particles[0].Position.Z, 1e-6f);
        }

        [Test]
        public void LocalizedImpulseTravelsWithTheBodyWhileTheShapeReturns() {
            BodyMaterial material = Still();
            DeformableBody body = SphereGeometry.Create(1f, 4, 2, material, Vec3.Zero);
            body.AddConstraint(new ShapeMatchingConstraint(body, 0f));
            float restExtent = Extent(body);
            body.ApplyImpulse(1, new Vec3(1f, 0f, 0f));

            PhysicsWorld world = World(4);
            world.AddBody(body);
            world.Tick(0.02f);

            Assert.AreEqual(1f / 6f, body.Particles[0].Velocity.X, 1e-3f);
            Assert.AreEqual(1f / 6f, body.Particles[1].Velocity.X, 1e-3f);
            Assert.AreEqual(0.02f / 6f, body.CenterOfMass().X, 1e-3f);
            Assert.AreEqual(restExtent, Extent(body), 1e-3f);

            for (int step = 0; step < 29; step++) world.Tick(0.02f);

            Assert.AreEqual(0.1f, body.CenterOfMass().X, 1e-2f);
            Assert.AreEqual(0f, body.CenterOfMass().Y, 1e-2f);
            Assert.AreEqual(restExtent, Extent(body), 2e-2f);
            Assert.AreEqual(body.CenterOfMass().X, body.Particles[0].Position.X, 2e-2f);
            Assert.AreEqual(1f, body.Particles[0].Position.Y, 2e-2f);
            Assert.IsFalse(body.Particles[1].Position.HasNaN());
        }

        [Test]
        public void StiffAttachmentMovesTheParticleToTheTarget() {
            var body = new DeformableBody(1, Still());
            body.Particles[0] = Placed(0f, 0f, 0f, 1f);
            var attachment = new AttachmentConstraint(0, new Vec3(1f, 2f, -3f), 0f);
            body.AddConstraint(attachment);

            PhysicsWorld world = World(1);
            world.TickBody(body, 0.02f);

            Assert.AreEqual(1f, body.Particles[0].Position.X, 1e-5f);
            Assert.AreEqual(2f, body.Particles[0].Position.Y, 1e-5f);
            Assert.AreEqual(-3f, body.Particles[0].Position.Z, 1e-5f);

            attachment.Target = new Vec3(-2f, 0.5f, 4f);
            world.TickBody(body, 0.02f);

            Assert.AreEqual(-2f, body.Particles[0].Position.X, 1e-4f);
            Assert.AreEqual(0.5f, body.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(4f, body.Particles[0].Position.Z, 1e-4f);
            Assert.IsFalse(body.Particles[0].Position.HasNaN());
        }

        [Test]
        public void ComplianceLeavesTheParticlePartwayToTheTarget() {
            BodyMaterial material = Still();
            material.Damping = 1000f;
            var body = new DeformableBody(1, material);
            body.Particles[0] = Placed(0f, 0f, 0f, 1f);
            body.AddConstraint(new AttachmentConstraint(0, new Vec3(1f, 0f, 0f), 0.0004f));

            PhysicsWorld world = World(1);
            world.TickBody(body, 0.02f);
            Assert.AreEqual(0.5f, body.Particles[0].Position.X, 1e-4f);

            world.TickBody(body, 0.02f);
            Assert.AreEqual(0.75f, body.Particles[0].Position.X, 1e-4f);
            Assert.AreEqual(0f, body.Particles[0].Position.Y, 1e-4f);
        }

        [Test]
        public void FixedParticleStaysWhenItIsAttached() {
            var body = new DeformableBody(1, Still());
            body.Particles[0] = Placed(1f, 2f, 3f, 0f);
            body.AddConstraint(new AttachmentConstraint(0, new Vec3(8f, 8f, 8f), 0f));
            World(4).TickBody(body, 0.02f);

            Assert.AreEqual(1f, body.Particles[0].Position.X, 1e-6f);
            Assert.AreEqual(2f, body.Particles[0].Position.Y, 1e-6f);
            Assert.AreEqual(3f, body.Particles[0].Position.Z, 1e-6f);
            Assert.AreEqual(0f, body.Particles[0].Velocity.X, 1e-6f);
        }

        [Test]
        public void MovingTheTargetPullsOneParticleAndTheOtherFollows() {
            var body = new DeformableBody(2, Still());
            body.Particles[0] = Placed(0f, 0f, 0f, 1f);
            body.Particles[1] = Placed(1f, 0f, 0f, 1f);
            body.AddConstraint(new DistanceConstraint(0, 1, 1f, 0f));
            var attachment = new AttachmentConstraint(0, new Vec3(0f, 1f, 0f), 0f);
            body.AddConstraint(attachment);

            World(8).TickBody(body, 0.02f);

            Assert.AreEqual(0f, body.Particles[0].Position.X, 1e-3f);
            Assert.AreEqual(1f, body.Particles[0].Position.Y, 1e-3f);
            Assert.AreEqual(1f, (body.Particles[1].Position - body.Particles[0].Position).Length, 2e-2f);
            Assert.Greater(body.Particles[1].Position.Y, 0.1f);
            Assert.Greater((body.Particles[1].Position - attachment.Target).Length, 0.5f);
            Assert.IsFalse(body.Particles[1].Position.HasNaN());
        }

        static BodyMaterial Still() {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = Vec3.Zero;
            material.Damping = 0f;
            material.Compliance = 0f;
            return material;
        }

        static PhysicsWorld World(int iterations) {
            return new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 0.02f,
                SolverIterations = iterations,
                MaxStepsPerFrame = 8
            });
        }

        static Particle Placed(float x, float y, float z, float inverseMass) {
            var position = new Vec3(x, y, z);
            return new Particle {
                Position = position,
                Previous = position,
                InverseMass = inverseMass
            };
        }

        static float Extent(DeformableBody body) {
            Vec3 center = body.CenterOfMass();
            float sum = 0f;
            for (int i = 0; i < body.Particles.Length; i++) sum += (body.Particles[i].Position - center).Length;
            return sum / body.Particles.Length;
        }
    }
}
