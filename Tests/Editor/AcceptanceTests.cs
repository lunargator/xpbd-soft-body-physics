using SoftBodyPhysics.Samples;
using NUnit.Framework;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using UnityEngine;

namespace SoftBodyPhysics.Tests {
    public class AcceptanceTests {
        [Test]
        public void FallingSphereDeformsThenRests() {
            DeformableBody body = DeformablePhysicsTestSetup.CreateSphere(
                DeformablePhysicsTestSetup.BlueMaterial(),
                new Vec3(0f, 1.8f, 0f),
                DeformablePhysicsTestSetup.BlueShape,
                DeformablePhysicsTestSetup.BlueVolume);
            float restHeight = Height(body);
            PhysicsWorld world = new PhysicsWorld(DeformablePhysicsTestSetup.Settings());
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            world.AddBody(body);

            float smallest = restHeight;
            for (int step = 0; step < 180; step++) {
                world.Tick(0.02f);
                float height = Height(body);
                if(height < smallest) smallest = height;
            }

            float minY = MinY(body);
            float settled = Height(body);
            float speed = Fastest(body);
            Assert.GreaterOrEqual(minY, -1e-3f, "minY " + minY);
            Assert.LessOrEqual(minY, 0.08f, "minY " + minY);
            Assert.Less(smallest, restHeight * 0.97f, "impact height " + smallest + " rest " + restHeight);
            Assert.Greater(settled, restHeight * 0.72f, "settled " + settled);
            Assert.Less(settled, restHeight * 0.99f, "settled " + settled);
            Assert.Less(speed, 0.2f, "speed " + speed);
        }

        [Test]
        public void MovingAnAttachmentDeformsTheBody() {
            BodyMaterial material = DeformablePhysicsTestSetup.AmberMaterial();
            material.Gravity = Vec3.Zero;
            material.Damping = 1.5f;
            DeformableBody body = DeformablePhysicsTestSetup.CreateSphere(
                material,
                Vec3.Zero,
                DeformablePhysicsTestSetup.AmberShape,
                DeformablePhysicsTestSetup.AmberVolume);
            int attachedIndex = 0;
            int freeIndex = body.Particles.Length - 1;
            Vec3 freeStart = body.Particles[freeIndex].Position;
            float restGap = (freeStart - body.Particles[attachedIndex].Position).Length;
            var attachment = new AttachmentConstraint(attachedIndex, body.Particles[attachedIndex].Position, 0.0002f);
            body.AddConstraint(attachment);

            PhysicsWorld world = new PhysicsWorld(DeformablePhysicsTestSetup.Settings());
            world.AddBody(body);
            var stepMove = new Vec3(0.02f, 0.028f, 0f);
            for (int step = 0; step < 24; step++) {
                attachment.Target += stepMove;
                world.Tick(0.02f);
            }

            float follow = (body.Particles[attachedIndex].Position - attachment.Target).Length;
            float gap = (body.Particles[freeIndex].Position - body.Particles[attachedIndex].Position).Length;
            float freeTravel = (body.Particles[freeIndex].Position - freeStart).Length;
            Assert.Less(follow, 0.15f, "follow " + follow);
            Assert.Greater(freeTravel, 0.2f, "free travel " + freeTravel);
            Assert.Greater(System.Math.Abs(gap - restGap), 0.04f, "gap " + gap + " rest " + restGap);
            Assert.Greater(gap, restGap * 0.55f, "collapsed " + gap);
            Assert.IsFalse(body.Particles[freeIndex].Position.HasNaN());
        }

        static float Height(DeformableBody body) {
            float min = float.PositiveInfinity;
            float max = float.NegativeInfinity;
            for (int i = 0; i < body.Particles.Length; i++) {
                float y = body.Particles[i].Position.Y;
                if(y < min) min = y;
                if(y > max) max = y;
            }

            return max - min;
        }

        static float MinY(DeformableBody body) {
            float min = float.PositiveInfinity;
            for (int i = 0; i < body.Particles.Length; i++) { if(body.Particles[i].Position.Y < min) min = body.Particles[i].Position.Y; }

            return min;
        }

        static float Fastest(DeformableBody body) {
            float fastest = 0f;
            for (int i = 0; i < body.Particles.Length; i++) {
                Assert.IsFalse(body.Particles[i].Position.HasNaN());
                float speed = body.Particles[i].Velocity.Length;
                if(speed > fastest) fastest = speed;
            }

            return fastest;
        }
    }
}
