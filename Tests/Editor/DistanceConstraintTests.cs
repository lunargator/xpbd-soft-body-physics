using SoftBodyPhysics.Unity;
using System;
using NUnit.Framework;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Tests {
    public class DistanceConstraintTests {
        [Test]
        public void StiffPairReachesRestLengthInOneIteration() {
            DeformableBody body = Pair(0f, 2f, restLength: 1f, compliance: 0f, inverseMassB: 1f);
            World(iterations: 1).TickBody(body, 0.02f);

            Assert.AreEqual(0.5f, body.Particles[0].Position.X, 1e-5f);
            Assert.AreEqual(1.5f, body.Particles[1].Position.X, 1e-5f);
            Assert.AreEqual(1f, Distance(body, 0, 1), 1e-5f);
            Assert.AreEqual(1f, body.CenterOfMass().X, 1e-5f);
            Assert.AreEqual(0f, body.Particles[0].Position.Y, 1e-6f);
            Assert.IsFalse(body.Particles[0].Position.HasNaN());
        }

        [Test]
        public void FixedParticleStaysWhileTheFreeEndMoves() {
            DeformableBody body = Pair(0f, 2f, restLength: 1f, compliance: 0f, inverseMassB: 0f);
            World(iterations: 1).TickBody(body, 0.02f);

            Assert.AreEqual(2f, body.Particles[1].Position.X, 1e-5f);
            Assert.AreEqual(0f, body.Particles[1].Velocity.X, 1e-5f);
            Assert.AreEqual(1f, body.Particles[0].Position.X, 1e-5f);
            Assert.AreEqual(1f, Distance(body, 0, 1), 1e-5f);
        }

        [Test]
        public void ComplianceCorrectsAgainAfterLambdaReset() {
            DeformableBody body = Pair(0f, 2f, restLength: 1f, compliance: 0.0004f, inverseMassB: 1f);
            body.Material.Damping = 1000f;
            PhysicsWorld world = World(iterations: 1);

            world.TickBody(body, 0.02f);
            Assert.AreEqual(4f / 3f, Distance(body, 0, 1), 1e-4f);

            world.TickBody(body, 0.02f);
            Assert.AreEqual(10f / 9f, Distance(body, 0, 1), 1e-4f);
            Assert.IsFalse(body.Particles[0].Position.HasNaN());
            Assert.IsFalse(body.Particles[1].Velocity.HasNaN());
        }

        [Test]
        public void SharedChainTightensAsIterationsIncrease() {
            DeformableBody once = Chain();
            DeformableBody many = Chain();
            World(iterations: 1).TickBody(once, 0.02f);
            World(iterations: 8).TickBody(many, 0.02f);

            Assert.AreEqual(0.25f, once.Particles[0].Position.X, 1e-5f);
            Assert.AreEqual(1.625f, once.Particles[1].Position.X, 1e-5f);
            Assert.AreEqual(2.625f, once.Particles[2].Position.X, 1e-5f);
            Assert.AreEqual(0.375f, StretchError(once), 1e-4f);

            Assert.Less(StretchError(many), 1e-3f);
            Assert.AreEqual(0.5f, many.Particles[0].Position.X, 1e-3f);
            Assert.AreEqual(1.5f, many.Particles[1].Position.X, 1e-3f);
            Assert.AreEqual(2.5f, many.Particles[2].Position.X, 1e-3f);
            Assert.AreEqual(1.5f, many.CenterOfMass().X, 1e-4f);
        }

        [Test]
        public void StiffConstraintHoldsAParticleAgainstGravity() {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, -10f, 0f);
            material.Damping = 0f;
            var held = new DeformableBody(2, material);
            held.Particles[0] = ParticleAt(new Vec3(0f, 1f, 0f), 0f);
            held.Particles[1] = ParticleAt(new Vec3(0f, 0f, 0f), 1f);
            held.AddConstraint(new DistanceConstraint(0, 1, 1f, 0f));

            var dropped = new DeformableBody(2, material);
            dropped.Particles[0] = ParticleAt(new Vec3(0f, 1f, 0f), 0f);
            dropped.Particles[1] = ParticleAt(new Vec3(0f, 0f, 0f), 1f);

            PhysicsWorld world = World(iterations: 4);
            world.AddBody(held);
            world.AddBody(dropped);
            world.Tick(0.1f);

            Assert.AreEqual(1f, held.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(0f, held.Particles[1].Position.Y, 1e-4f);
            Assert.AreEqual(1f, Distance(held, 0, 1), 1e-4f);
            Assert.Less(dropped.Particles[1].Position.Y, -0.05f);
            Assert.IsFalse(held.Particles[1].Position.HasNaN());
        }

        [Test]
        public void CoincidentParticlesDoNotProduceNaNs() {
            DeformableBody body = Pair(0f, 0f, restLength: 1f, compliance: 0f, inverseMassB: 1f);
            World(iterations: 4).TickBody(body, 0.02f);

            Assert.AreEqual(0f, body.Particles[0].Position.X, 1e-6f);
            Assert.AreEqual(0f, body.Particles[1].Position.X, 1e-6f);
            Assert.IsFalse(body.Particles[0].Position.HasNaN());
            Assert.IsFalse(body.Particles[1].Position.HasNaN());
        }

        static PhysicsWorld World(int iterations) {
            return new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 0.02f,
                SolverIterations = iterations,
                MaxStepsPerFrame = 8
            });
        }

        static DeformableBody Pair(float x0, float x1, float restLength, float compliance, float inverseMassB) {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = Vec3.Zero;
            material.Damping = 0f;
            var body = new DeformableBody(2, material);
            body.Particles[0] = ParticleAt(new Vec3(x0, 0f, 0f), 1f);
            body.Particles[1] = ParticleAt(new Vec3(x1, 0f, 0f), inverseMassB);
            body.AddConstraint(new DistanceConstraint(0, 1, restLength, compliance));
            return body;
        }

        static DeformableBody Chain() {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = Vec3.Zero;
            material.Damping = 0f;
            var body = new DeformableBody(3, material);
            body.Particles[0] = ParticleAt(new Vec3(0f, 0f, 0f), 1f);
            body.Particles[1] = ParticleAt(new Vec3(1.5f, 0f, 0f), 1f);
            body.Particles[2] = ParticleAt(new Vec3(3f, 0f, 0f), 1f);
            body.AddConstraint(new DistanceConstraint(0, 1, 1f, 0f));
            body.AddConstraint(new DistanceConstraint(1, 2, 1f, 0f));
            return body;
        }

        static Particle ParticleAt(Vec3 position, float inverseMass) {
            return new Particle {
                Position = position,
                Previous = position,
                InverseMass = inverseMass
            };
        }

        static float Distance(DeformableBody body, int a, int b) {
            return (body.Particles[a].Position - body.Particles[b].Position).Length;
        }

        static float StretchError(DeformableBody body) { return Math.Abs(Distance(body, 0, 1) - 1f) + Math.Abs(Distance(body, 1, 2) - 1f); }
    }

    static class WorldTestExtensions {
        public static void TickBody(this PhysicsWorld world, DeformableBody body, float frameDelta) {
            if(world.Bodies.Count == 0) world.AddBody(body);
            world.Tick(frameDelta);
        }
    }
}
