using SoftBodyPhysics.Unity;
using System;
using NUnit.Framework;
using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Tests {
    public class PhysicsWorldTests {
        [Test]
        public void RemovingABodyDropsItFromTheWorld() {
            PhysicsWorld world = MakeWorld(0.02f);
            DeformableBody first = MakeBody(-10f, 0f);
            DeformableBody second = MakeBody(-10f, 0f);
            world.AddBody(first);
            world.AddBody(first);
            world.AddBody(second);

            Assert.AreEqual(2, world.Bodies.Count);
            Assert.IsTrue(world.RemoveBody(first));
            Assert.AreEqual(1, world.Bodies.Count);
            Assert.AreSame(second, world.Bodies[0]);
            Assert.IsFalse(world.RemoveBody(first));
        }

        [Test]
        public void FreeParticleFallsAndFixedParticleStays() {
            const float stepSeconds = 0.02f;
            const float gravity = -10f;
            const float duration = 0.1f;
            const int steps = 5;

            DeformableBody body = MakeBody(gravity, damping: 0f);
            PhysicsWorld world = MakeWorld(stepSeconds);
            world.AddBody(body);
            world.Tick(duration);

            float expectedY = 2f + gravity * stepSeconds * stepSeconds * steps * (steps + 1) * 0.5f;
            float analyticY = 2f + 0.5f * gravity * duration * duration;
            Particle free = body.Particles[0];
            Particle pinned = body.Particles[1];

            Assert.AreEqual(1f, free.Position.X, 1e-5f);
            Assert.AreEqual(3f, free.Position.Z, 1e-5f);
            Assert.AreEqual(expectedY, free.Position.Y, 1e-4f);
            Assert.LessOrEqual(Math.Abs(free.Position.Y - analyticY), 0.5f * Math.Abs(gravity) * stepSeconds * duration + 1e-4f);
            Assert.AreEqual(4f, pinned.Position.X, 1e-6f);
            Assert.AreEqual(5f, pinned.Position.Y, 1e-6f);
            Assert.AreEqual(6f, pinned.Position.Z, 1e-6f);
            Assert.AreEqual(0f, pinned.Velocity.Y, 1e-6f);
            Assert.IsFalse(free.Position.HasNaN());
            Assert.IsFalse(free.Velocity.HasNaN());
            Assert.AreEqual(0f, free.Force.SqrLength, 1e-8f);
        }

        [Test]
        public void SameSimulatedTimeIsIndependentOfFrameSplits() {
            DeformableBody single = MakeBody(-10f, 0f);
            DeformableBody split = MakeBody(-10f, 0f);
            PhysicsWorld singleWorld = MakeWorld(0.02f);
            PhysicsWorld splitWorld = MakeWorld(0.02f);
            singleWorld.AddBody(single);
            splitWorld.AddBody(split);

            singleWorld.Tick(0.1f);
            splitWorld.Tick(0.05f);
            splitWorld.Tick(0.05f);

            DeformableBody stepped = MakeBody(-10f, 0f);
            PhysicsWorld steppedWorld = MakeWorld(0.02f);
            steppedWorld.AddBody(stepped);
            for (int i = 0; i < 5; i++) steppedWorld.Tick(0.02f);

            Assert.AreEqual(single.Particles[0].Position.Y, split.Particles[0].Position.Y, 1e-5f);
            Assert.AreEqual(single.Particles[0].Position.Y, stepped.Particles[0].Position.Y, 1e-5f);
            Assert.AreEqual(single.Particles[0].Velocity.Y, split.Particles[0].Velocity.Y, 1e-5f);
            Assert.AreEqual(0f, singleWorld.Accumulator, 1e-6f);
            Assert.AreEqual(0f, splitWorld.Accumulator, 1e-6f);
            Assert.AreEqual(0f, singleWorld.DroppedTime, 1e-6f);
        }

        [Test]
        public void AFrameLongerThanTheCapDropsTheExtraTime() {
            PhysicsWorld world = new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 0.02f,
                SolverIterations = 1,
                MaxStepsPerFrame = 2
            });
            world.AddBody(MakeBody(-10f, 0f));
            world.Tick(0.1f);

            Assert.AreEqual(0.06f, world.LastDroppedTime, 1e-5f);
            Assert.AreEqual(0.06f, world.DroppedTime, 1e-5f);
            Assert.AreEqual(0f, world.Accumulator, 1e-5f);

            world.Tick(0.02f);
            Assert.AreEqual(0f, world.LastDroppedTime, 1e-5f);
            Assert.AreEqual(0.06f, world.DroppedTime, 1e-5f);
        }

        [Test]
        public void DampingReducesThePredictedSpeed() {
            DeformableBody undamped = MakeBody(-10f, 0f);
            DeformableBody damped = MakeBody(-10f, 4f);
            PhysicsWorld undampedWorld = MakeWorld(0.02f);
            PhysicsWorld dampedWorld = MakeWorld(0.02f);
            undampedWorld.AddBody(undamped);
            dampedWorld.AddBody(damped);

            undampedWorld.Tick(0.02f);
            dampedWorld.Tick(0.02f);

            Assert.Less(Math.Abs(damped.Particles[0].Velocity.Y), Math.Abs(undamped.Particles[0].Velocity.Y));
            Assert.Greater(damped.Particles[0].Position.Y, undamped.Particles[0].Position.Y);
        }

        static PhysicsWorld MakeWorld(float stepSeconds) {
            return new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = stepSeconds,
                SolverIterations = 4,
                MaxStepsPerFrame = 8
            });
        }

        static DeformableBody MakeBody(float gravity, float damping) {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, gravity, 0f);
            material.Damping = damping;
            var body = new DeformableBody(2, material);
            body.Particles[0] = new Particle {
                Position = new Vec3(1f, 2f, 3f),
                Previous = new Vec3(1f, 2f, 3f),
                InverseMass = 1f
            };
            body.Particles[1] = new Particle {
                Position = new Vec3(4f, 5f, 6f),
                Previous = new Vec3(4f, 5f, 6f),
                InverseMass = 0f
            };
            body.CaptureRestPositions();
            return body;
        }
    }
}
