using SoftBodyPhysics.Unity;
using NUnit.Framework;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;

namespace SoftBodyPhysics.Tests {
    public class BoxCollisionTests {
        [Test]
        public void ParticleInsideABoxLeavesThroughTheNearestFace() {
            CollisionBox box = CollisionBox.FromCenterSize(Vec3.Zero, new Vec3(2f, 2f, 2f));
            PhysicsWorld world = World(1);
            world.Collision.AddBox(box);

            DeformableBody top = Particle(new Vec3(0f, 0.7f, 0.2f), Vec3.Zero, 0f);
            DeformableBody side = Particle(new Vec3(0.85f, 0.1f, -0.2f), Vec3.Zero, 0f);
            DeformableBody back = Particle(new Vec3(0f, 0f, -0.9f), Vec3.Zero, 0f);
            world.AddBody(top);
            world.AddBody(side);
            world.AddBody(back);
            world.Tick(0.02f);

            Assert.AreEqual(0f, top.Particles[0].Position.X, 1e-4f);
            Assert.AreEqual(1f, top.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(0.2f, top.Particles[0].Position.Z, 1e-4f);
            Assert.AreEqual(1f, side.Particles[0].Position.X, 1e-4f);
            Assert.AreEqual(0.1f, side.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(-0.2f, side.Particles[0].Position.Z, 1e-4f);
            Assert.AreEqual(-1f, back.Particles[0].Position.Z, 1e-4f);
            Assert.AreEqual(0f, back.Particles[0].Position.X, 1e-4f);
            Assert.IsFalse(top.Particles[0].Position.HasNaN());
        }

        [Test]
        public void ParticleOutsideTheBoxIsLeftAlone() {
            PhysicsWorld world = World(4);
            world.Collision.AddBox(CollisionBox.FromCenterSize(Vec3.Zero, new Vec3(2f, 2f, 2f)));
            DeformableBody body = Particle(new Vec3(4f, 4f, 4f), Vec3.Zero, 0f);
            world.AddBody(body);
            world.Tick(0.1f);

            Assert.AreEqual(4f, body.Particles[0].Position.X, 1e-5f);
            Assert.AreEqual(4f, body.Particles[0].Position.Y, 1e-5f);
            Assert.AreEqual(4f, body.Particles[0].Position.Z, 1e-5f);
            Assert.AreEqual(0, world.Collision.ContactCount);
        }

        [Test]
        public void HittingTheTopUsesRestitutionAndFriction() {
            var box = new CollisionBox { Min = new Vec3(-2f, 0f, -2f), Max = new Vec3(2f, 1f, 2f) };

            DeformableBody bounce = Particle(new Vec3(0f, 1.01f, 0f), new Vec3(0f, -1f, 0f), 0f);
            bounce.Material.Restitution = 0.5f;
            PhysicsWorld bouncing = World(1);
            bouncing.Collision.AddBox(box);
            bouncing.AddBody(bounce);
            bouncing.Tick(0.02f);

            var platform = new CollisionBox { Min = new Vec3(-2f, -1f, -2f), Max = new Vec3(2f, 0f, 2f) };
            DeformableBody sand = Particle(Vec3.Zero, new Vec3(1f, 0f, 0f), -10f);
            sand.Material.Friction = 0.5f;
            PhysicsWorld rough = World(1);
            rough.Collision.AddBox(platform);
            rough.AddBody(sand);
            rough.Tick(0.02f);

            Assert.AreEqual(1f, bounce.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(0.5f, bounce.Particles[0].Velocity.Y, 1e-3f);
            Assert.AreEqual(0f, sand.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(0.7f, sand.Particles[0].Velocity.X, 1e-3f);
        }

        [Test]
        public void ParticleUnderGravityRestsOnTheBox() {
            PhysicsWorld world = World(4);
            world.Collision.AddBox(new CollisionBox {
                Min = new Vec3(-1f, -1f, -1f),
                Max = new Vec3(1f, 0.5f, 1f)
            });
            DeformableBody body = Particle(new Vec3(0f, 0.5f, 0f), Vec3.Zero, -10f);
            body.Material.Friction = 0.4f;
            body.Material.Restitution = 0.2f;
            world.AddBody(body);

            for (int step = 0; step < 50; step++) world.Tick(0.02f);

            Assert.AreEqual(0.5f, body.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(0f, body.Particles[0].Velocity.Y, 1e-4f);
            Assert.IsFalse(body.Particles[0].Position.HasNaN());
        }

        [Test]
        public void SoftSphereLandsOnABoxWithoutEnteringIt() {
            var box = new CollisionBox {
                Min = new Vec3(-1f, 0f, -1f),
                Max = new Vec3(1f, 0.5f, 1f)
            };
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, -10f, 0f);
            material.Damping = 2f;
            material.Compliance = 0.0002f;
            material.Friction = 0.6f;
            material.Restitution = 0f;
            DeformableBody body = SphereGeometry.Create(0.4f, 6, 4, material, new Vec3(0f, 1.5f, 0f));
            body.AddConstraint(new ShapeMatchingConstraint(body, 0.0005f));
            body.AddConstraint(new VolumeConstraint(body, 0.0002f));

            var world = new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 0.02f,
                SolverIterations = 8,
                MaxStepsPerFrame = 8
            });
            world.Collision.AddBox(box);
            world.AddBody(body);
            for (int step = 0; step < 150; step++) world.Tick(0.02f);

            float minY = float.PositiveInfinity;
            float maxY = float.NegativeInfinity;
            for (int i = 0; i < body.Particles.Length; i++) {
                Vec3 position = body.Particles[i].Position;
                Assert.IsFalse(position.HasNaN());
                Assert.IsFalse(IsInside(position, box));
                if(position.Y < minY) minY = position.Y;
                if(position.Y > maxY) maxY = position.Y;
            }

            Assert.GreaterOrEqual(minY, box.Max.Y - 1e-3f);
            Assert.LessOrEqual(minY, box.Max.Y + 0.05f);
            Assert.Greater(maxY - minY, 0.35f);
        }

        static bool IsInside(Vec3 point, CollisionBox box) {
            return point.X > box.Min.X && point.X < box.Max.X
                && point.Y > box.Min.Y && point.Y < box.Max.Y - 1e-3f
                && point.Z > box.Min.Z && point.Z < box.Max.Z;
        }

        static PhysicsWorld World(int iterations) {
            return new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 0.02f,
                SolverIterations = iterations,
                MaxStepsPerFrame = 8
            });
        }

        static DeformableBody Particle(Vec3 position, Vec3 velocity, float gravity) {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, gravity, 0f);
            material.Damping = 0f;
            material.Friction = 0f;
            material.Restitution = 0f;
            var body = new DeformableBody(1, material);
            body.Particles[0] = new Particle {
                Position = position,
                Previous = position,
                Velocity = velocity,
                InverseMass = 1f
            };
            return body;
        }
    }
}
