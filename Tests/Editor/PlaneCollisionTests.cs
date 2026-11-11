using SoftBodyPhysics.Unity;
using NUnit.Framework;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;

namespace SoftBodyPhysics.Tests {
    public class PlaneCollisionTests {
        [Test]
        public void ParticleBelowAPlaneIsPushedOutAlongTheNormal() {
            PhysicsWorld floor = World(iterations: 1);
            floor.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            DeformableBody body = Particle(new Vec3(1.5f, -0.2f, -3f), new Vec3(0f, 0f, 0f), gravity: 0f);
            floor.AddBody(body);
            floor.Tick(0.02f);

            Assert.AreEqual(1.5f, body.Particles[0].Position.X, 1e-4f);
            Assert.AreEqual(0f, body.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(-3f, body.Particles[0].Position.Z, 1e-4f);

            PhysicsWorld wall = World(iterations: 1);
            wall.Collision.AddPlane(CollisionPlane.FromNormalAndPoint(new Vec3(1f, 0f, 0f), Vec3.Zero));
            DeformableBody side = Particle(new Vec3(-0.25f, 2f, 3f), Vec3.Zero, gravity: 0f);
            wall.AddBody(side);
            wall.Tick(0.02f);

            Assert.AreEqual(0f, side.Particles[0].Position.X, 1e-4f);
            Assert.AreEqual(2f, side.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(3f, side.Particles[0].Position.Z, 1e-4f);
            Assert.IsFalse(side.Particles[0].Position.HasNaN());
        }

        [Test]
        public void ParticleAboveThePlaneIsLeftAlone() {
            PhysicsWorld world = World(iterations: 4);
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            DeformableBody body = Particle(new Vec3(0f, 1f, 0f), Vec3.Zero, gravity: 0f);
            world.AddBody(body);
            world.Tick(0.1f);

            Assert.AreEqual(1f, body.Particles[0].Position.Y, 1e-5f);
            Assert.AreEqual(0, world.Collision.ContactCount);
        }

        [Test]
        public void RestitutionScalesTheOutgoingNormalSpeed() {
            DeformableBody bounce = Particle(new Vec3(0f, 0.01f, 0f), new Vec3(0f, -1f, 0f), gravity: 0f);
            bounce.Material.Restitution = 0.5f;
            PhysicsWorld bouncing = World(iterations: 1);
            bouncing.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            bouncing.AddBody(bounce);
            bouncing.Tick(0.02f);

            DeformableBody dead = Particle(new Vec3(0f, 0.01f, 0f), new Vec3(0f, -1f, 0f), gravity: 0f);
            dead.Material.Restitution = 0f;
            PhysicsWorld resting = World(iterations: 1);
            resting.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            resting.AddBody(dead);
            resting.Tick(0.02f);

            Assert.AreEqual(0f, bounce.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(0.5f, bounce.Particles[0].Velocity.Y, 1e-3f);
            Assert.AreEqual(0f, dead.Particles[0].Velocity.Y, 1e-3f);
        }

        [Test]
        public void FrictionReducesTheSlideAlongThePlane() {
            PhysicsWorld slippery = World(iterations: 1);
            slippery.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            DeformableBody ice = Particle(Vec3.Zero, new Vec3(1f, 0f, 0f), gravity: -10f);
            ice.Material.Friction = 0f;
            slippery.AddBody(ice);

            PhysicsWorld rough = World(iterations: 1);
            rough.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            DeformableBody sand = Particle(Vec3.Zero, new Vec3(1f, 0f, 0f), gravity: -10f);
            sand.Material.Friction = 0.5f;
            rough.AddBody(sand);

            slippery.Tick(0.02f);
            rough.Tick(0.02f);

            Assert.AreEqual(1f, ice.Particles[0].Velocity.X, 1e-3f);
            Assert.AreEqual(0.7f, sand.Particles[0].Velocity.X, 1e-3f);
            Assert.AreEqual(0f, sand.Particles[0].Position.Y, 1e-4f);
        }

        [Test]
        public void ParticleUnderGravityRestsOnThePlane() {
            PhysicsWorld world = World(iterations: 4);
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            DeformableBody body = Particle(Vec3.Zero, Vec3.Zero, gravity: -10f);
            body.Material.Friction = 0.5f;
            body.Material.Restitution = 0.2f;
            world.AddBody(body);

            for (int step = 0; step < 50; step++) world.Tick(0.02f);

            Assert.AreEqual(0f, body.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(0f, body.Particles[0].Velocity.Y, 1e-4f);
            Assert.IsFalse(body.Particles[0].Position.HasNaN());
        }

        [Test]
        public void SoftSphereLandsWithoutSinking() {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, -10f, 0f);
            material.Damping = 2f;
            material.Compliance = 0.0002f;
            material.Friction = 0.6f;
            material.Restitution = 0f;
            DeformableBody body = SphereGeometry.Create(0.5f, 6, 4, material, new Vec3(0f, 1.2f, 0f));
            body.AddConstraint(new ShapeMatchingConstraint(body, 0.0005f));
            body.AddConstraint(new VolumeConstraint(body, 0.0002f));

            var world = new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 0.02f,
                SolverIterations = 8,
                MaxStepsPerFrame = 8
            });
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            world.AddBody(body);
            for (int step = 0; step < 150; step++) world.Tick(0.02f);

            float minY = float.PositiveInfinity;
            float maxY = float.NegativeInfinity;
            Vec3 com = Vec3.Zero;
            for (int i = 0; i < body.Particles.Length; i++) {
                float y = body.Particles[i].Position.Y;
                if(y < minY) minY = y;
                if(y > maxY) maxY = y;
                com += body.Particles[i].Position;
                Assert.IsFalse(body.Particles[i].Position.HasNaN());
            }

            com *= 1f / body.Particles.Length;
            Assert.GreaterOrEqual(minY, -1e-3f);
            Assert.LessOrEqual(minY, 0.05f);
            Assert.Greater(maxY - minY, 0.45f);
            Assert.Less(com.Y, 1f);
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
