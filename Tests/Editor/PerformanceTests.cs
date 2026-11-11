using SoftBodyPhysics.Samples;
using System;
using System.IO;
using System.Text;
using NUnit.Framework;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Unity;
using UnityEngine;

namespace SoftBodyPhysics.Tests {
    public class PerformanceTests {
        [Test]
        public void BenchmarkRecordsStepCosts() {
            var report = new StringBuilder();
            StepProfile one = Measure(1, 6, 4, 3f, "one sphere", report);
            StepProfile separated = Measure(8, 6, 4, 3f, "eight separated", report);
            StepProfile pile = Measure(8, 6, 4, 0.2f, "eight piled", report);
            StepProfile fine = Measure(1, 12, 8, 3f, "fine sphere", report);
            string text = report.ToString();
            File.WriteAllText(Path.Combine(Directory.GetParent(Application.dataPath).FullName, "Temp", "PhysicsBenchmark.txt"), text);

            Assert.Greater(one.TotalMilliseconds, 0f);
            Assert.Greater(separated.ParticleCount, one.ParticleCount);
            Assert.AreEqual(fine.ParticleCount, 86);
            Assert.Greater(pile.CollisionMilliseconds, separated.CollisionMilliseconds);
        }

        [Test]
        public void BowlStaysInsideAFrame() {
            PhysicsWorld world = World();
            BodyMaterial material = Surface(0.35f);
            material.Damping = 0.4f;
            CollisionPlane[] walls = SoftBodyBowl.CreatePlanes();
            for (int i = 0; i < walls.Length; i++) world.Collision.AddPlane(walls[i]);
            for (int i = 0; i < SoftBodyBowl.SphereCount; i++)
                world.AddBody(SoftBodyShapes.Sphere(SoftBodyBowl.SphereRadius, 6, 4, material, SoftBodyBowl.SlotCenter(i)));
            var report = new StringBuilder();
            for (int i = 0; i < 8; i++) world.Step(1f / 60f);
            StepProfile falling = Average(world, 12);
            for (int i = 0; i < 70; i++) world.Step(1f / 60f);
            StepProfile resting = Average(world, 12);
            Finite(world);
            report.AppendLine(
                "bowl fall bodies " + falling.BodyCount
                + " particles " + falling.ParticleCount
                + " total " + falling.TotalMilliseconds.ToString("0.000")
                + " collision " + falling.CollisionMilliseconds.ToString("0.000"));
            report.AppendLine(
                "bowl rest bodies " + resting.BodyCount
                + " particles " + resting.ParticleCount
                + " total " + resting.TotalMilliseconds.ToString("0.000")
                + " collision " + resting.CollisionMilliseconds.ToString("0.000")
                + " queries " + world.Collision.SurfaceQueries
                + " skips " + world.Collision.SurfaceSkips
                + " motion " + world.Collision.SurfaceMotion.ToString("0.0000")
                + " bin " + world.Collision.BinHits + "/" + world.Collision.BinMisses + "/" + world.Collision.BinUnsure
                + " full " + world.Collision.FullQueries
                + " prepare " + world.Collision.PrepareMilliseconds.ToString("0.00")
                + " surface " + world.Collision.SurfaceMilliseconds.ToString("0.00")
                + " solve " + world.Collision.SolveMilliseconds.ToString("0.00")
                + " broad " + world.Collision.BroadMilliseconds.ToString("0.00")
                + " hold " + world.Collision.HoldMilliseconds.ToString("0.00"));
            string path = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "Temp", "PhysicsBenchmark.txt");
            File.AppendAllText(path, report.ToString());

            Assert.Less(falling.TotalMilliseconds, 10f);
            Assert.Less(resting.TotalMilliseconds, 8f);
        }

        [Test]
        public void CourseStaysInsideAFrame() {
            PhysicsWorld world = World();
            BodyMaterial material = Surface(0.4f);
            material.Damping = 0.35f;
            material.Friction = 0.45f;
            material.Restitution = 0f;
            AddCourseColliders(world.Collision);
            Vector3[] spawns = {
                new Vector3(0f, 1.2f, 0f),
                new Vector3(-0.5f, 1.7f, 0.2f),
                new Vector3(0.5f, 1.7f, -0.2f),
                new Vector3(0f, 2.2f, 0.4f),
                new Vector3(-0.4f, 2.2f, -0.3f),
                new Vector3(0.4f, 2.6f, 0.1f),
                new Vector3(0f, 3.0f, 0f)
            };
            for (int i = 0; i < spawns.Length; i++) {
                world.AddBody(SoftBodyShapes.Sphere(
                    0.22f,
                    12,
                    8,
                    material,
                    new Vec3(spawns[i].x, spawns[i].y, spawns[i].z)));
            }

            var report = new StringBuilder();
            for (int i = 0; i < 8; i++) world.Step(1f / 60f);
            StepProfile falling = Average(world, 8);
            for (int i = 0; i < 50; i++) world.Step(1f / 60f);
            StepProfile resting = Average(world, 8);
            Finite(world);
            report.AppendLine(
                "course fall bodies " + falling.BodyCount
                + " particles " + falling.ParticleCount
                + " total " + falling.TotalMilliseconds.ToString("0.000")
                + " collision " + falling.CollisionMilliseconds.ToString("0.000")
                + " broad " + world.Collision.BroadMilliseconds.ToString("0.00"));
            report.AppendLine(
                "course rest bodies " + resting.BodyCount
                + " particles " + resting.ParticleCount
                + " total " + resting.TotalMilliseconds.ToString("0.000")
                + " collision " + resting.CollisionMilliseconds.ToString("0.000")
                + " broad " + world.Collision.BroadMilliseconds.ToString("0.00")
                + " prepare " + world.Collision.PrepareMilliseconds.ToString("0.00")
                + " surface " + world.Collision.SurfaceMilliseconds.ToString("0.00")
                + " solve " + world.Collision.SolveMilliseconds.ToString("0.00")
                + " tests " + world.Collision.OrientedTests + "/" + world.Collision.SphereTests + "/" + world.Collision.TriangleTests);
            string path = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "Temp", "PhysicsBenchmark.txt");
            File.AppendAllText(path, report.ToString());

            Assert.Less(falling.TotalMilliseconds, 12f);
            Assert.Less(resting.TotalMilliseconds, 8f);
        }

        [Test]
        public void SolverTimeGrowsWithParticleCount() {
            var report = new StringBuilder();
            report.AppendLine(Conditions());
            int[] segments = { 3, 6, 8, 12 };
            int[] stacks = { 2, 4, 6, 8 };
            float first = 0f;
            float last = 0f;
            int lastParticles = 0;
            for (int i = 0; i < segments.Length; i++) {
                Measured step = TimeOneSphere(segments[i], stacks[i], 4, 0);
                report.AppendLine(PaceLine("particle sweep", step));
                Assert.Greater(step.Profile.ParticleCount, lastParticles);
                Assert.Greater(step.Profile.TotalMilliseconds, 0f);
                if(i == 0) first = step.Profile.TotalMilliseconds;
                last = step.Profile.TotalMilliseconds;
                lastParticles = step.Profile.ParticleCount;
            }

            Assert.AreEqual(5, TimeOneSphere(3, 2, 4, 0).Profile.ParticleCount);
            Assert.AreEqual(86, lastParticles);
            Assert.Greater(last, first);
            AppendBenchmark(report);
        }

        [Test]
        public void SolverTimeGrowsWithConstraintCount() {
            var report = new StringBuilder();
            report.AppendLine(Conditions());
            int[] extra = { 0, 40, 120 };
            float first = 0f;
            float last = 0f;
            int lastConstraints = -1;
            int particles = 0;
            for (int i = 0; i < extra.Length; i++) {
                Measured step = TimeOneSphere(6, 4, 4, extra[i]);
                report.AppendLine(PaceLine("constraint sweep", step));
                Assert.Greater(step.Constraints, lastConstraints);
                Assert.Greater(step.Profile.ConstraintMilliseconds, 0f);
                if(i == 0) {
                    first = step.Profile.ConstraintMilliseconds;
                    particles = step.Profile.ParticleCount;
                }
                last = step.Profile.ConstraintMilliseconds;
                lastConstraints = step.Constraints;
            }

            Assert.AreEqual(particles, TimeOneSphere(6, 4, 4, 120).Profile.ParticleCount);
            Assert.Greater(last, first);
            AppendBenchmark(report);
        }

        [Test]
        public void SolverTimeGrowsWithIterations() {
            var report = new StringBuilder();
            report.AppendLine(Conditions());
            int[] iterations = { 1, 2, 4, 8 };
            float first = 0f;
            float last = 0f;
            int particles = 0;
            for (int i = 0; i < iterations.Length; i++) {
                Measured step = TimeOneSphere(6, 4, iterations[i], 0);
                report.AppendLine(PaceLine("iteration sweep " + iterations[i], step));
                Assert.Greater(step.Profile.TotalMilliseconds, 0f);
                if(i == 0) {
                    first = step.Profile.TotalMilliseconds;
                    particles = step.Profile.ParticleCount;
                } else Assert.AreEqual(particles, step.Profile.ParticleCount);
                last = step.Profile.TotalMilliseconds;
            }

            Assert.Greater(last, first);
            AppendBenchmark(report);
        }

        static void AddCourseColliders(CollisionWorld collision) {
            collision.AddOrientedBox(Box(new Vec3(0f, -0.1f, 0f), new Vec3(4f, 0.1f, 4f)));
            collision.AddOrientedBox(Box(new Vec3(-3f, 1f, 0f), new Vec3(0.1f, 1f, 3f)));
            collision.AddOrientedBox(Box(new Vec3(3f, 1f, 0f), new Vec3(0.1f, 1f, 3f)));
            collision.AddOrientedBox(Box(new Vec3(0f, 1f, -3f), new Vec3(3f, 1f, 0.1f)));
            collision.AddOrientedBox(Box(new Vec3(0f, 1f, 3f), new Vec3(3f, 1f, 0.1f)));
            collision.AddOrientedBox(Box(new Vec3(1.2f, 0.2f, 1.2f), new Vec3(0.6f, 0.15f, 0.6f)));
            collision.AddSphere(new CollisionSphere { Center = new Vec3(-1.2f, 0.45f, 0.4f), Radius = 0.45f });
            collision.AddCapsule(new CollisionCapsule {
                Start = new Vec3(0.2f, 0.3f, -1.1f),
                End = new Vec3(0.2f, 0.9f, -1.1f),
                Radius = 0.2f
            });
            AddCourseTriangle(collision, new Vec3(-0.4f, 0.4f, 1.4f), new Vec3(0f, 0.1f, 1.4f), new Vec3(0.4f, 0.4f, 1.4f), 0);
            AddCourseTriangle(collision, new Vec3(-0.4f, 0.4f, 1.8f), new Vec3(0.4f, 0.4f, 1.8f), new Vec3(0f, 0.1f, 1.8f), 0);
            Vec3 crate = new Vec3(-1.4f, 0.35f, -1.2f);
            float h = 0.3f;
            AddCourseTriangle(collision, crate + new Vec3(-h, -h, -h), crate + new Vec3(h, -h, -h), crate + new Vec3(h, -h, h), 1);
            AddCourseTriangle(collision, crate + new Vec3(-h, -h, -h), crate + new Vec3(h, -h, h), crate + new Vec3(-h, -h, h), 1);
            AddCourseTriangle(collision, crate + new Vec3(-h, h, -h), crate + new Vec3(h, h, h), crate + new Vec3(h, h, -h), 1);
            AddCourseTriangle(collision, crate + new Vec3(-h, h, -h), crate + new Vec3(-h, h, h), crate + new Vec3(h, h, h), 1);
        }

        static CollisionOrientedBox Box(Vec3 center, Vec3 half) {
            return new CollisionOrientedBox {
                Center = center,
                AxisX = new Vec3(half.X, 0f, 0f),
                AxisY = new Vec3(0f, half.Y, 0f),
                AxisZ = new Vec3(0f, 0f, half.Z)
            };
        }

        static void AddCourseTriangle(CollisionWorld collision, Vec3 a, Vec3 b, Vec3 c, int closed) {
            collision.AddTriangle(new CollisionTriangle { A = a, B = b, C = c, Closed = closed });
        }

        static StepProfile Average(PhysicsWorld world, int samples) {
            StepProfile sum = default;
            for (int i = 0; i < samples; i++) {
                world.Step(1f / 60f);
                StepProfile step = world.LastProfile;
                sum.PredictMilliseconds += step.PredictMilliseconds;
                sum.ConstraintMilliseconds += step.ConstraintMilliseconds;
                sum.CollisionMilliseconds += step.CollisionMilliseconds;
                sum.VelocityMilliseconds += step.VelocityMilliseconds;
            }

            float scale = 1f / samples;
            sum.PredictMilliseconds *= scale;
            sum.ConstraintMilliseconds *= scale;
            sum.CollisionMilliseconds *= scale;
            sum.VelocityMilliseconds *= scale;
            sum.BodyCount = world.LastProfile.BodyCount;
            sum.ParticleCount = world.LastProfile.ParticleCount;
            return sum;
        }

        [Test]
        public void FastImpactStaysFinite() {
            PhysicsWorld world = World();
            DeformableBody body = Sphere(0.3f, 6, 4, new Vec3(0f, 1.2f, 0f));
            for (int i = 0; i < body.Particles.Length; i++) {
                Particle particle = body.Particles[i];
                particle.Velocity = new Vec3(0f, -120f, 0f);
                body.Particles[i] = particle;
            }

            world.AddBody(body);
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            for (int i = 0; i < 40; i++) world.Step(1f / 60f);

            Finite(world);
            Assert.Greater(Lowest(body), -0.05f);
        }

        [Test]
        public void LargeDeformationDoesNotExplode() {
            PhysicsWorld world = World();
            DeformableBody body = Sphere(0.4f, 6, 4, Vec3.Zero);
            Particle pulled = body.Particles[0];
            pulled.Position = new Vec3(6f, 1f, 0f);
            pulled.Previous = pulled.Position;
            body.Particles[0] = pulled;
            world.AddBody(body);
            for (int i = 0; i < 20; i++) world.Step(1f / 60f);

            Finite(world);
            Assert.Greater(Spread(body), 0.05f);
            Assert.Less(body.CenterOfMass().SqrLength, 80f);
        }

        [Test]
        public void ExtremeStiffnessAndTimestepsStayFinite() {
            BodyMaterial soft = Surface(0f);
            BodyMaterial firm = Surface(1f);
            PhysicsWorld world = World();
            DeformableBody jelly = SoftBodyShapes.Sphere(0.3f, 6, 4, soft, new Vec3(-1f, 1f, 0f));
            DeformableBody rigid = SoftBodyShapes.Sphere(0.3f, 6, 4, firm, new Vec3(1f, 1f, 0f));
            world.AddBody(jelly);
            world.AddBody(rigid);
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            for (int i = 0; i < 50; i++) world.Step(1f / 60f);
            Finite(world);
            Assert.Less(Height(jelly), Height(rigid) - 0.02f);

            PhysicsWorld fine = World();
            fine.AddBody(Sphere(0.3f, 6, 4, new Vec3(0f, 0.6f, 0f)));
            fine.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            for (int i = 0; i < 20; i++) fine.Step(0.001f);
            Finite(fine);

            PhysicsWorld coarse = World();
            coarse.AddBody(Sphere(0.3f, 6, 4, new Vec3(0f, 1.5f, 0f)));
            coarse.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            for (int i = 0; i < 12; i++) coarse.Step(0.05f);
            Finite(coarse);
            Assert.Greater(Lowest(coarse.Bodies[0]), -0.2f);
        }

        [Test]
        public void LowAndHighCountsAndOddCollidersStayFinite() {
            PhysicsWorld low = World();
            low.AddBody(Sphere(0.3f, 3, 2, new Vec3(0f, 0.8f, 0f)));
            low.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            for (int i = 0; i < 15; i++) low.Step(1f / 60f);
            Finite(low);
            Assert.AreEqual(5, low.Bodies[0].Particles.Length);

            PhysicsWorld high = World();
            high.AddBody(Sphere(0.3f, 12, 8, new Vec3(0f, 0.8f, 0f)));
            high.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            for (int i = 0; i < 10; i++) high.Step(1f / 60f);
            Finite(high);
            Assert.AreEqual(86, high.Bodies[0].Particles.Length);

            PhysicsWorld crowd = World();
            for (int i = 0; i < 10; i++) crowd.AddBody(Sphere(0.25f, 6, 4, new Vec3((i % 5) * 0.7f, 0.7f, (i / 5) * 0.7f)));
            crowd.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            crowd.Collision.AddSphere(new CollisionSphere { Center = new Vec3(1.4f, -1.5f, 0.3f), Radius = 1.6f });
            crowd.Collision.AddCapsule(new CollisionCapsule {
                Start = new Vec3(-1f, 0.3f, 1.2f),
                End = new Vec3(2f, 0.3f, 1.2f),
                Radius = 0.2f
            });
            crowd.Collision.AddOrientedBox(new CollisionOrientedBox {
                Center = new Vec3(1.2f, 0.4f, -0.8f),
                AxisX = new Vec3(0.15f, 0f, 0.05f),
                AxisY = new Vec3(0f, 0.45f, 0f),
                AxisZ = new Vec3(-0.05f, 0f, 0.2f)
            });
            for (int i = 0; i < 20; i++) crowd.Step(1f / 60f);
            Finite(crowd);
        }

        struct Measured {
            public StepProfile Profile;
            public int Constraints;
        }

        static Measured TimeOneSphere(int segments, int stacks, int iterations, int extraDistances) {
            var world = new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 1f / 60f,
                SolverIterations = iterations,
                MaxStepsPerFrame = 1
            });
            DeformableBody body = Sphere(0.35f, segments, stacks, new Vec3(0f, 1f, 0f));
            int count = body.Particles.Length;
            for (int n = 0; n < extraDistances; n++) {
                int indexA = n % count;
                int indexB = (n * 3 + 1) % count;
                if(indexA == indexB) indexB = (indexB + 1) % count;
                body.AddConstraint(DistanceConstraint.Create(body, indexA, indexB));
            }

            world.AddBody(body);
            for (int i = 0; i < 8; i++) world.Step(1f / 60f);
            return new Measured {
                Profile = Average(world, 30),
                Constraints = body.Constraints.Count
            };
        }

        static string PaceLine(string label, Measured step) {
            return label
                + " particles " + step.Profile.ParticleCount
                + " constraints " + step.Constraints
                + " predict " + step.Profile.PredictMilliseconds.ToString("0.000")
                + " constraint " + step.Profile.ConstraintMilliseconds.ToString("0.000")
                + " collision " + step.Profile.CollisionMilliseconds.ToString("0.000")
                + " velocity " + step.Profile.VelocityMilliseconds.ToString("0.000")
                + " total " + step.Profile.TotalMilliseconds.ToString("0.000");
        }

        static string Conditions() {
            return "host " + SystemInfo.processorType
                + " | cores " + SystemInfo.processorCount
                + " | ram " + SystemInfo.systemMemorySize + " MB"
                + " | " + SystemInfo.operatingSystem
                + " | Unity " + Application.unityVersion
                + " | one sphere, no colliders, step 1/60 s, warmup 8, average of 30 steps";
        }

        static void AppendBenchmark(StringBuilder report) {
            string path = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "Temp", "PhysicsBenchmark.txt");
            File.AppendAllText(path, report.ToString());
        }

        static StepProfile Measure(int count, int segments, int stacks, float spacing, string label, StringBuilder report) {
            PhysicsWorld world = World();
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            for (int i = 0; i < count; i++) {
                var center = new Vec3((i % 4) * spacing, 0.9f, (i / 4) * spacing);
                world.AddBody(Sphere(0.35f, segments, stacks, center));
            }

            for (int i = 0; i < 8; i++) world.Step(1f / 60f);

            int collections = GC.CollectionCount(0);
            long memory = GC.GetTotalMemory(false);
            StepProfile sum = default;
            const int samples = 20;
            for (int i = 0; i < samples; i++) {
                world.Step(1f / 60f);
                StepProfile step = world.LastProfile;
                sum.PredictMilliseconds += step.PredictMilliseconds;
                sum.ConstraintMilliseconds += step.ConstraintMilliseconds;
                sum.CollisionMilliseconds += step.CollisionMilliseconds;
                sum.VelocityMilliseconds += step.VelocityMilliseconds;
            }

            float scale = 1f / samples;
            sum.PredictMilliseconds *= scale;
            sum.ConstraintMilliseconds *= scale;
            sum.CollisionMilliseconds *= scale;
            sum.VelocityMilliseconds *= scale;
            sum.BodyCount = world.LastProfile.BodyCount;
            sum.ParticleCount = world.LastProfile.ParticleCount;
            Finite(world);
            report.AppendLine(
                label
                + " bodies " + sum.BodyCount
                + " particles " + sum.ParticleCount
                + " predict " + sum.PredictMilliseconds.ToString("0.000")
                + " constraint " + sum.ConstraintMilliseconds.ToString("0.000")
                + " collision " + sum.CollisionMilliseconds.ToString("0.000")
                + " velocity " + sum.VelocityMilliseconds.ToString("0.000")
                + " total " + sum.TotalMilliseconds.ToString("0.000")
                + " gen0 " + (GC.CollectionCount(0) - collections)
                + " bytes " + (GC.GetTotalMemory(false) - memory));
            return sum;
        }

        static PhysicsWorld World() {
            return new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 1f / 60f,
                SolverIterations = 4,
                MaxStepsPerFrame = 1
            });
        }

        static BodyMaterial Surface(float stiffness) {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 0.15f;
            material.Friction = 0.3f;
            material.Restitution = 0.05f;
            material.Compliance = SoftBody.ComplianceFor(stiffness);
            return material;
        }

        static DeformableBody Sphere(float radius, int segments, int stacks, Vec3 center) {
            return SoftBodyShapes.Sphere(radius, segments, stacks, Surface(0.55f), center);
        }

        static void Finite(PhysicsWorld world) {
            for (int bodyIndex = 0; bodyIndex < world.Bodies.Count; bodyIndex++) {
                Particle[] particles = world.Bodies[bodyIndex].Particles;
                for (int i = 0; i < particles.Length; i++) {
                    Assert.IsFalse(particles[i].Position.HasNaN());
                    Assert.IsFalse(particles[i].Velocity.HasNaN());
                    Assert.Less(particles[i].Position.SqrLength, 1e6f);
                }
            }
        }

        static float Lowest(DeformableBody body) {
            float min = body.Particles[0].Position.Y;
            for (int i = 1; i < body.Particles.Length; i++) { if(body.Particles[i].Position.Y < min) min = body.Particles[i].Position.Y; }

            return min;
        }

        static float Height(DeformableBody body) {
            float min = body.Particles[0].Position.Y;
            float max = min;
            for (int i = 1; i < body.Particles.Length; i++) {
                float y = body.Particles[i].Position.Y;
                if(y < min) min = y;
                if(y > max) max = y;
            }

            return max - min;
        }

        static float Spread(DeformableBody body) {
            Vec3 center = body.CenterOfMass();
            float max = 0f;
            for (int i = 0; i < body.Particles.Length; i++) {
                float distance = (body.Particles[i].Position - center).Length;
                if(distance > max) max = distance;
            }

            return max;
        }
    }
}
