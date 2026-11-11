using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;
using SoftBodyPhysics.Unity;
using UnityEditor;
using UnityEngine;

namespace SoftBodyPhysics.Tests {
    public class ValidationTests {
        readonly List<Object> owned = new List<Object>();

        [Test]
        public void TinyAndHugeMassesStayFinite() {
            BodyMaterial material = Still();
            var body = new DeformableBody(2, material);
            body.Particles[0] = Placed(0f, 0f, 0f, 1e-6f);
            body.Particles[1] = Placed(2f, 0f, 0f, 1e6f);
            body.AddConstraint(new DistanceConstraint(0, 1, 1f, 0f));
            PhysicsWorld world = World(8);
            world.AddBody(body);
            world.Step(1f / 60f);

            AssertFinite(body);
            float heavyMove = Abs(body.Particles[0].Position.X);
            float lightMove = Abs(body.Particles[1].Position.X - 2f);
            Assert.Greater(lightMove, heavyMove * 100f, "light " + lightMove + " heavy " + heavyMove);
            Assert.Less(heavyMove, 0.01f);
        }

        [Test]
        public void ALargeForceStaysFinite() {
            BodyMaterial material = Still();
            DeformableBody body = SphereGeometry.Create(0.5f, 6, 4, material, Vec3.Zero);
            Particle[] particles = body.Particles;
            for (int i = 0; i < particles.Length; i++) {
                Particle particle = particles[i];
                particle.Force = new Vec3(1e6f, 0f, 0f);
                particles[i] = particle;
            }

            PhysicsWorld world = World(4);
            world.AddBody(body);
            world.Step(1f / 60f);
            AssertFinite(body);
            Assert.Less(Abs(body.CenterOfMass().X), 1e5f);
        }

        [Test]
        public void AnExtremeSpeedStaysFinite() {
            PhysicsWorld world = World(4);
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            BodyMaterial material = Still();
            material.Restitution = 0.5f;
            material.Friction = 0.5f;
            var body = new DeformableBody(1, material);
            Particle particle = Placed(0f, 1f, 0f, 1f);
            particle.Velocity = new Vec3(0f, -1e4f, 0f);
            body.Particles[0] = particle;
            world.AddBody(body);
            world.Step(1f / 60f);

            AssertFinite(body);
            Assert.Less(Abs(body.Particles[0].Position.Y), 1e6f);
        }

        [Test]
        public void ADegenerateTriangleDoesNotProduceNaN() {
            BodyMaterial material = Still();
            var host = new DeformableBody(3, material);
            host.Particles[0] = Placed(0f, 0f, 0f, 1f);
            host.Particles[1] = Placed(1f, 0f, 0f, 1f);
            host.Particles[2] = Placed(2f, 0f, 0f, 1f);
            host.Triangles = new[] { 0, 1, 2 };
            var guest = new DeformableBody(1, material);
            guest.Particles[0] = Placed(1f, -0.2f, 0f, 1f);

            PhysicsWorld world = World(4);
            world.AddBody(host);
            world.AddBody(guest);
            world.Step(1f / 60f);

            AssertFinite(host);
            AssertFinite(guest);
            Assert.AreEqual(-0.2f, guest.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(0, world.Collision.ContactCount);
        }

        [Test]
        public void ZeroRestLengthPullsWithoutNaN() {
            BodyMaterial material = Still();
            var body = new DeformableBody(2, material);
            body.Particles[0] = Placed(0f, 0f, 0f, 1f);
            body.Particles[1] = Placed(1f, 0f, 0f, 1f);
            body.AddConstraint(new DistanceConstraint(0, 1, 0f, 0f));
            PhysicsWorld world = World(4);
            world.AddBody(body);
            world.Step(1f / 60f);

            AssertFinite(body);
            float length = (body.Particles[0].Position - body.Particles[1].Position).Length;
            Assert.Less(length, 0.05f, "length " + length);
        }

        [Test]
        public void ANearlyFlatVolumeStaysFinite() {
            DeformableBody body = Tetrahedron();
            body.AddConstraint(new VolumeConstraint(body, 0f));
            for (int i = 0; i < body.Particles.Length; i++) {
                Particle particle = body.Particles[i];
                particle.Position = new Vec3(particle.Position.X, particle.Position.Y * 1e-4f, particle.Position.Z);
                particle.Previous = particle.Position;
                body.Particles[i] = particle;
            }

            float before = VolumeConstraint.SignedVolume(body.Particles, body.Triangles);
            PhysicsWorld world = World(4);
            world.AddBody(body);
            world.Step(1f / 60f);

            AssertFinite(body);
            float after = VolumeConstraint.SignedVolume(body.Particles, body.Triangles);
            Assert.IsFalse(float.IsNaN(after) || float.IsInfinity(after));
            Assert.Greater(Abs(after), Abs(before));
        }

        [Test]
        public void AnInvertedSurfaceStaysFiniteForManySteps() {
            DeformableBody body = Tetrahedron();
            body.Material.Gravity = new Vec3(0f, -9.81f, 0f);
            Particle apex = body.Particles[3];
            apex.Position = new Vec3(0.2f, -1f, 0.2f);
            apex.Previous = apex.Position;
            body.Particles[3] = apex;
            body.AddConstraint(new VolumeConstraint(body, 0f));
            Assert.Less(body.Constraints[0] is VolumeConstraint volume ? volume.RestVolume * VolumeConstraint.SignedVolume(body.Particles, body.Triangles) : 0f, 0f);

            PhysicsWorld world = World(4);
            world.AddBody(body);
            for (int i = 0; i < 60; i++) world.Step(1f / 60f);

            AssertFinite(body);
            VolumeConstraint constraint = (VolumeConstraint)body.Constraints[0];
            Assert.Less(constraint.RestVolume * VolumeConstraint.SignedVolume(body.Particles, body.Triangles), 0f);
        }

        [Test]
        public void ACollapsedShapeMatchStaysFinite() {
            DeformableBody body = Tetrahedron();
            body.AddConstraint(new ShapeMatchingConstraint(body, 0f));
            for (int i = 0; i < body.Particles.Length; i++) {
                Particle particle = body.Particles[i];
                particle.Position = new Vec3(particle.Position.X, 0f, particle.Position.Z);
                particle.Previous = particle.Position;
                body.Particles[i] = particle;
            }

            PhysicsWorld world = World(4);
            world.AddBody(body);
            world.Step(1f / 60f);

            AssertFinite(body);
            float extent = 0f;
            for (int i = 0; i < body.Particles.Length; i++) {
                float y = Abs(body.Particles[i].Position.Y);
                if(y > extent) extent = y;
            }

            Assert.Greater(extent, 0.05f, "extent " + extent);
        }

        [Test]
        public void ADeepPenetrationLeavesTheBox() {
            PhysicsWorld world = World(4);
            world.Collision.AddBox(CollisionBox.FromCenterSize(Vec3.Zero, new Vec3(2f, 2f, 2f)));
            var body = new DeformableBody(1, Still());
            body.Particles[0] = Placed(0f, 0f, 0f, 1f);
            world.AddBody(body);
            world.Step(1f / 60f);

            AssertFinite(body);
            Vec3 position = body.Particles[0].Position;
            float outside = Abs(position.X);
            if(Abs(position.Y) > outside) outside = Abs(position.Y);
            if(Abs(position.Z) > outside) outside = Abs(position.Z);
            Assert.Greater(outside, 0.99f, "position " + position.X + " " + position.Y + " " + position.Z);
        }

        [Test]
        public void ExtremeFrictionAndRestitutionStayFinite() {
            BodyMaterial material = Still();
            material.Friction = 8f;
            material.Restitution = 4f;
            var body = new DeformableBody(1, material);
            Particle particle = Placed(0f, 0.2f, 0f, 1f);
            particle.Velocity = new Vec3(6f, -8f, 0f);
            body.Particles[0] = particle;
            PhysicsWorld world = World(4);
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            world.AddBody(body);
            world.Step(1f / 60f);

            AssertFinite(body);
            Assert.Greater(body.Particles[0].Position.Y, -0.05f);
        }

        [Test]
        public void AOneToFourMassRatioKeepsMomentum() {
            BodyMaterial material = Still();
            var light = new DeformableBody(1, material);
            Particle moving = Placed(0.25f, -0.15f, 0.25f, 1f);
            moving.Velocity = new Vec3(0f, -1f, 0f);
            light.Particles[0] = moving;
            var heavy = new DeformableBody(3, material);
            heavy.Particles[0] = Placed(0f, 0f, 0f, 0.25f);
            heavy.Particles[1] = Placed(1f, 0f, 0f, 0.25f);
            heavy.Particles[2] = Placed(0f, 0f, 1f, 0.25f);
            heavy.Triangles = new[] { 0, 2, 1 };
            float before = MomentumY(light) + MomentumY(heavy);

            PhysicsWorld world = World(4);
            world.AddBody(light);
            world.AddBody(heavy);
            world.Step(1f / 60f);

            AssertFinite(light);
            AssertFinite(heavy);
            float after = MomentumY(light) + MomentumY(heavy);
            Assert.AreEqual(before, after, 0.35f, "before " + before + " after " + after);
            Assert.Greater(Abs(light.Particles[0].Position.Y - (-0.15f)), Abs(MeanY(heavy)) * 2f);
        }

        [Test]
        public void ARestingSphereStaysFiniteForALongRun() {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 0.4f;
            material.Restitution = 0.05f;
            material.Friction = 0.4f;
            DeformableBody body = SphereGeometry.Create(0.5f, 6, 4, material, new Vec3(0f, 1f, 0f));
            PhysicsWorld world = World(4);
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            world.AddBody(body);
            const int steps = 180;
            for (int i = 0; i < steps; i++) world.Step(1f / 60f);

            AssertFinite(body);
            float height = body.CenterOfMass().Y;
            float speed = MaxSpeed(body);
            string path = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "Temp", "Soak.txt");
            File.WriteAllText(path, "steps " + steps + " height " + height.ToString("0.000") + " speed " + speed.ToString("0.000") + " particles " + body.Particles.Length);
            Assert.Greater(height, 0.25f, "height " + height);
            Assert.Less(height, 0.9f, "height " + height);
            Assert.Less(speed, 2f, "speed " + speed);
        }

        [Test]
        public void ABodyDoesNotCollideWithItsOwnSurface() {
            BodyMaterial material = Still();
            var self = new DeformableBody(4, material);
            self.Particles[0] = Placed(0f, 0f, 0f, 1f);
            self.Particles[1] = Placed(1f, 0f, 0f, 1f);
            self.Particles[2] = Placed(0f, 0f, 1f, 1f);
            self.Particles[3] = Placed(0.2f, -0.2f, 0.2f, 1f);
            self.Triangles = new[] { 0, 2, 1 };
            self.ClosedSurface = true;
            PhysicsWorld world = World(4);
            world.AddBody(self);
            world.Step(1f / 60f);

            AssertFinite(self);
            Assert.AreEqual(-0.2f, self.Particles[3].Position.Y, 1e-5f);
            Assert.AreEqual(0, world.Collision.ContactCount);
        }

        [Test]
        public void TwoWorldsDoNotShareAStep() {
            BodyMaterial material = Still();
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            var left = new DeformableBody(1, material);
            var right = new DeformableBody(1, material);
            left.Particles[0] = Placed(0f, 1f, 0f, 1f);
            right.Particles[0] = Placed(0f, 1f, 0f, 1f);
            PhysicsWorld first = World(4);
            PhysicsWorld second = World(4);
            first.AddBody(left);
            second.AddBody(right);
            for (int i = 0; i < 10; i++) first.Step(1f / 60f);

            Assert.Less(left.Particles[0].Position.Y, 0.9f);
            Assert.AreEqual(1f, right.Particles[0].Position.Y, 1e-5f);
            Assert.AreEqual(1, second.Bodies.Count);
        }

        [Test]
        public void DestroyingAndRebuildingKeepsTheSurvivors() {
            Cleanup();
            try {
                SoftBody first = CreateSphere();
                SoftBody second = CreateSphere();
                first.Initialize();
                second.Initialize();
                PhysicsWorld world = first.Simulation.World;
                Object.DestroyImmediate(first.gameObject);
                SoftBody rebuilt = CreateSphere();
                rebuilt.Initialize();
                Assert.AreEqual(2, world.Bodies.Count);
                world.Step(1f / 60f);
                AssertFinite(second.Body);
                AssertFinite(rebuilt.Body);
            } finally { Cleanup(); }
        }

        [Test]
        public void DisablingABodyRemovesItUntilEnabled() {
            Cleanup();
            try {
                SoftBody stayed = CreateSphere();
                SoftBody paused = CreateSphere();
                stayed.Initialize();
                paused.Initialize();
                PhysicsWorld world = stayed.Simulation.World;
                Assert.AreEqual(2, world.Bodies.Count);
                paused.enabled = false;
                Assert.AreEqual(1, world.Bodies.Count);
                Assert.AreSame(stayed.Body, world.Bodies[0]);
                world.Step(1f / 60f);
                paused.enabled = true;
                Assert.AreEqual(2, world.Bodies.Count);
                AssertFinite(stayed.Body);
                AssertFinite(paused.Body);
            } finally { Cleanup(); }
        }

        [Test]
        public void ASecondSimulationTurnsItselfOff() {
            Cleanup();
            try {
                var firstHost = new GameObject("First Simulation");
                owned.Add(firstHost);
                SoftBodySimulation first = firstHost.AddComponent<SoftBodySimulation>();
                var secondHost = new GameObject("Second Simulation");
                owned.Add(secondHost);
                SoftBodySimulation second = secondHost.AddComponent<SoftBodySimulation>();
                Assert.IsTrue(first.enabled);
                Assert.IsFalse(second.enabled);
                Assert.AreSame(first, SoftBodySimulation.Ensure());
                SoftBody body = CreateSphere();
                body.Initialize();
                Assert.AreSame(first, body.Simulation);
                Assert.AreEqual(1, first.World.Bodies.Count);
                Assert.AreEqual(0, second.World.Bodies.Count);
            } finally { Cleanup(); }
        }

        [Test]
        public void EnablingOutsidePlayDoesNotBuildTheBody() {
            Cleanup();
            try {
                var host = new GameObject("Play Gate");
                owned.Add(host);
                SoftBody body = host.AddComponent<SoftBody>();
                body.initializeOnEnable = true;
                Assert.IsFalse(Application.isPlaying);
                Assert.IsNull(body.Body);
                host.SetActive(false);
                host.SetActive(true);
                Assert.IsNull(body.Body);
                body.Initialize();
                Assert.IsNotNull(body.Body);
                Assert.Greater(body.Body.Particles.Length, 0);
                AssertFinite(body.Body);
            } finally { Cleanup(); }
        }

        [Test]
        public void ASerializedCopyKeepsTheBodySettings() {
            Cleanup();
            try {
                SoftBody source = CreateSphere();
                source.mass = 3.5f;
                source.stiffness = 0.8f;
                source.damping = 1.2f;
                source.radius = 0.42f;
                source.friction = 0.7f;
                source.restitution = 0.15f;
                var host = new GameObject("Copied Body");
                owned.Add(host);
                SoftBody copy = host.AddComponent<SoftBody>();
                EditorUtility.CopySerialized(source, copy);
                Assert.AreEqual(3.5f, copy.mass, 1e-6f);
                Assert.AreEqual(0.8f, copy.stiffness, 1e-6f);
                Assert.AreEqual(1.2f, copy.damping, 1e-6f);
                Assert.AreEqual(0.42f, copy.radius, 1e-6f);
                Assert.AreEqual(0.7f, copy.friction, 1e-6f);
                Assert.AreEqual(0.15f, copy.restitution, 1e-6f);
                copy.Initialize();
                Assert.AreEqual(0.42f, copy.Body.Particles[0].Position.Length, 0.02f);
            } finally { Cleanup(); }
        }

        [Test]
        public void BenchmarkScenariosStayNamed() {
            string path = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "Temp", "PhysicsBenchmark.txt");
            string text = File.ReadAllText(path);
            string[] names = {
                "one sphere",
                "eight separated",
                "eight piled",
                "fine sphere",
                "bowl fall",
                "bowl rest",
                "course fall",
                "course rest"
            };
            for (int i = 0; i < names.Length; i++) Assert.IsTrue(text.Contains(names[i]), names[i]);
        }

        void Cleanup() {
            SoftBody[] bodies = Resources.FindObjectsOfTypeAll<SoftBody>();
            for (int i = 0; i < bodies.Length; i++) {
                if(bodies[i] != null && bodies[i].gameObject.scene.IsValid()) Object.DestroyImmediate(bodies[i].gameObject);
            }

            SoftBodySimulation[] worlds = Resources.FindObjectsOfTypeAll<SoftBodySimulation>();
            for (int i = 0; i < worlds.Length; i++) {
                if(worlds[i] != null && worlds[i].gameObject.scene.IsValid()) Object.DestroyImmediate(worlds[i].gameObject);
            }

            for (int i = 0; i < owned.Count; i++) { if(owned[i] != null) Object.DestroyImmediate(owned[i]); }

            owned.Clear();
        }

        SoftBody CreateSphere() {
            var host = new GameObject("Validation Sphere");
            host.hideFlags = HideFlags.DontSave;
            owned.Add(host);
            SoftBody body = host.AddComponent<SoftBody>();
            body.shape = SoftBody.ShapeKind.Sphere;
            body.sphereSegments = 6;
            body.sphereStacks = 4;
            body.initializeOnEnable = false;
            return body;
        }

        static void AssertFinite(DeformableBody body) {
            for (int i = 0; i < body.Particles.Length; i++) {
                Particle particle = body.Particles[i];
                Assert.IsFalse(particle.Position.HasNaN() || Infinite(particle.Position), "position " + i);
                Assert.IsFalse(particle.Velocity.HasNaN() || Infinite(particle.Velocity), "velocity " + i);
            }
        }

        static bool Infinite(Vec3 value) { return float.IsInfinity(value.X) || float.IsInfinity(value.Y) || float.IsInfinity(value.Z); }

        static float MomentumY(DeformableBody body) {
            float momentum = 0f;
            for (int i = 0; i < body.Particles.Length; i++) {
                float inverse = body.Particles[i].InverseMass;
                if(inverse > 0f) momentum += body.Particles[i].Velocity.Y / inverse;
            }

            return momentum;
        }

        static float MeanY(DeformableBody body) {
            float sum = 0f;
            for (int i = 0; i < body.Particles.Length; i++) sum += body.Particles[i].Position.Y;
            return sum / body.Particles.Length;
        }

        static float MaxSpeed(DeformableBody body) {
            float max = 0f;
            for (int i = 0; i < body.Particles.Length; i++) {
                float speed = body.Particles[i].Velocity.Length;
                if(speed > max) max = speed;
            }

            return max;
        }

        static DeformableBody Tetrahedron() {
            var body = new DeformableBody(4, Still());
            body.Particles[0] = Placed(0f, 0f, 0f, 1f);
            body.Particles[1] = Placed(1f, 0f, 0f, 1f);
            body.Particles[2] = Placed(0f, 0f, 1f, 1f);
            body.Particles[3] = Placed(0f, 1f, 0f, 1f);
            body.Triangles = new[] { 0, 2, 1, 0, 1, 3, 1, 2, 3, 0, 3, 2 };
            body.ClosedSurface = true;
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

        static BodyMaterial Still() {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = Vec3.Zero;
            material.Damping = 0f;
            return material;
        }

        static PhysicsWorld World(int iterations) {
            return new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 1f / 60f,
                SolverIterations = iterations,
                MaxStepsPerFrame = 1
            });
        }

        static float Abs(float value) { return value < 0f ? -value : value; }
    }
}
