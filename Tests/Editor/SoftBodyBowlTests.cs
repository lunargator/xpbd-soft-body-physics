using SoftBodyPhysics.Samples;
using NUnit.Framework;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;
using SoftBodyPhysics.Unity;

namespace SoftBodyPhysics.Tests {
    public class SoftBodyBowlTests {
        [Test]
        public void SpheresRestInTheBoxUntilShoved() {
            var world = new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 1f / 60f,
                SolverIterations = 4,
                MaxStepsPerFrame = 1
            });
            SoftBodyBowl.AddCollision(world.Collision);
            Assert.AreEqual(6, world.Collision.PlaneCount);

            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 0.4f;
            material.Compliance = SoftBody.ComplianceFor(0.35f);
            int[] slots = { 0, 6, 13, 19 };
            var bodies = new DeformableBody[slots.Length];
            for (int i = 0; i < slots.Length; i++) {
                bodies[i] = Sphere(material, SoftBodyBowl.SlotCenter(slots[i]));
                world.AddBody(bodies[i]);
            }

            CollisionPlane[] planes = SoftBodyBowl.CreatePlanes();
            bool touched = false;
            for (int step = 0; step < 120; step++) {
                world.Step(1f / 60f);
                if(world.Collision.ContactCount > 0) touched = true;
                for (int i = 0; i < bodies.Length; i++) AssertInside(bodies[i], planes, -0.2f);
            }

            Assert.IsTrue(touched);
            var direction = new Vec3(0.9f, 0.2f, -0.4f);
            Vec3 shove = direction * (SoftBodyBowl.ShoveForce / direction.Length);
            int shoveSteps = (int)(SoftBodyBowl.ShoveSeconds * 60f);
            float fastest = 0f;
            for (int step = 0; step < shoveSteps; step++) {
                for (int i = 0; i < bodies.Length; i++) AddForce(bodies[i], shove);
                world.Step(1f / 60f);
                for (int i = 0; i < bodies.Length; i++) {
                    AssertInside(bodies[i], planes, -0.45f);
                    fastest = Fastest(bodies[i], fastest);
                }
            }

            Assert.Greater(fastest, 1.5f);
        }

        [Test]
        public void RandomRocksStayClosedInsideTheBox() {
            var world = new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 1f / 60f,
                SolverIterations = 4,
                MaxStepsPerFrame = 1
            });
            SoftBodyBowl.AddCollision(world.Collision);
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 0.4f;
            material.Compliance = SoftBody.ComplianceFor(0.35f);

            DeformableBody first = Rock(3, material, new Vec3(-0.15f, 0.9f, 0.05f));
            DeformableBody second = Rock(11, material, new Vec3(0.32f, 1.25f, -0.08f));
            Assert.AreEqual(42, first.Particles.Length);
            Assert.AreEqual(42, second.Particles.Length);
            Assert.IsTrue(first.ClosedSurface);
            Assert.IsTrue(second.ClosedSurface);
            float rest = VolumeConstraint.SignedVolume(first.RestPositions, first.Triangles);
            Assert.Greater(rest, 0f);
            world.AddBody(first);
            world.AddBody(second);

            CollisionPlane[] planes = SoftBodyBowl.CreatePlanes();
            bool touched = false;
            for (int step = 0; step < 90; step++) {
                world.Step(1f / 60f);
                if(world.Collision.ContactCount > 0) touched = true;
                AssertInside(first, planes, -0.35f);
                AssertInside(second, planes, -0.35f);
            }

            Assert.IsTrue(touched);
            float volume = VolumeConstraint.SignedVolume(first.Particles, first.Triangles);
            Assert.Greater(volume, rest * 0.45f);
            Assert.Less(volume, rest * 1.7f);
        }

        [Test]
        public void SettledBodiesStayPut() {
            var world = new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 1f / 60f,
                SolverIterations = 4,
                MaxStepsPerFrame = 1
            });
            SoftBodyBowl.AddCollision(world.Collision);
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 0.4f;
            material.Compliance = SoftBody.ComplianceFor(0.35f);
            int[] slots = { 0, 6, 13, 19 };
            var bodies = new DeformableBody[slots.Length];
            var rests = new BodyRest[slots.Length];
            for (int i = 0; i < slots.Length; i++) {
                bodies[i] = Sphere(material, SoftBodyBowl.SlotCenter(slots[i]));
                rests[i] = new BodyRest();
                world.AddBody(bodies[i]);
            }

            for (int step = 0; step < 240; step++) {
                world.Step(1f / 60f);
                for (int i = 0; i < bodies.Length; i++) rests[i].Apply(bodies[i], false);
            }

            for (int i = 0; i < bodies.Length; i++) Assert.IsTrue(rests[i].Held, "body " + i + " com speed " + rests[i].LastComSpeed);

            Vec3[][] parked = CopyPositions(bodies);
            for (int step = 0; step < 60; step++) {
                world.Step(1f / 60f);
                for (int i = 0; i < bodies.Length; i++) rests[i].Apply(bodies[i], false);
            }

            for (int i = 0; i < bodies.Length; i++) {
                Assert.IsTrue(rests[i].Held);
                Assert.Less(Fastest(bodies[i], 0f), 0.001f);
                for (int p = 0; p < bodies[i].Particles.Length; p++) {
                    Vec3 moved = bodies[i].Particles[p].Position - parked[i][p];
                    Assert.Less(moved.SqrLength, 1e-8f);
                }
            }

            var direction = new Vec3(0.9f, 0.2f, -0.4f);
            Vec3 shove = direction * (SoftBodyBowl.ShoveForce / direction.Length);
            for (int i = 0; i < bodies.Length; i++) AddForce(bodies[i], shove);
            world.Step(1f / 60f);
            for (int i = 0; i < bodies.Length; i++) rests[i].Apply(bodies[i], true);
            for (int i = 0; i < bodies.Length; i++) Assert.IsFalse(rests[i].Held);
        }

        static Vec3[][] CopyPositions(DeformableBody[] bodies) {
            var copy = new Vec3[bodies.Length][];
            for (int i = 0; i < bodies.Length; i++) {
                Particle[] particles = bodies[i].Particles;
                copy[i] = new Vec3[particles.Length];
                for (int p = 0; p < particles.Length; p++) copy[i][p] = particles[p].Position;
            }

            return copy;
        }

        static DeformableBody Rock(int seed, BodyMaterial material, Vec3 offset) {
            RandomRock.Create(seed, SoftBodyBowl.RockRadius, out Vec3[] vertices, out int[] triangles);
            MeshGeometry.Build build = MeshGeometry.Create(vertices, triangles, 0f, material, offset);
            SoftBodyShapes.FinishSurface(build.Body, material.Compliance);
            return build.Body;
        }

        static DeformableBody Sphere(BodyMaterial material, Vec3 center) {
            DeformableBody body = SphereGeometry.Create(SoftBodyBowl.SphereRadius, 6, 4, material, center);
            SoftBodyShapes.Finish(body, material.Compliance);
            return body;
        }

        static void AddForce(DeformableBody body, Vec3 force) {
            int count = 0;
            for (int i = 0; i < body.Particles.Length; i++) { if(body.Particles[i].InverseMass > 0f) count++; }

            if(count == 0) return;
            Vec3 share = force * (1f / count);
            for (int i = 0; i < body.Particles.Length; i++) {
                if(body.Particles[i].InverseMass <= 0f) continue;
                Particle particle = body.Particles[i];
                particle.Force += share;
                body.Particles[i] = particle;
            }
        }

        static void AssertInside(DeformableBody body, CollisionPlane[] planes, float limit) {
            for (int i = 0; i < body.Particles.Length; i++) {
                Vec3 position = body.Particles[i].Position;
                Assert.IsFalse(position.HasNaN());
                for (int plane = 0; plane < planes.Length; plane++) Assert.Greater(SoftBodyBowl.Separation(planes[plane], position), limit);
            }
        }

        static float Fastest(DeformableBody body, float fastest) {
            for (int i = 0; i < body.Particles.Length; i++) {
                float speed = body.Particles[i].Velocity.Length;
                if(speed > fastest) fastest = speed;
            }

            return fastest;
        }
    }
}
