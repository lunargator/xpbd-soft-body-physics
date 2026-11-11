using SoftBodyPhysics.Unity;
using NUnit.Framework;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;

namespace SoftBodyPhysics.Tests {
    public class BodyCollisionTests {
        [Test]
        public void AVertexInsideATriangleIsPushedToTheSurface() {
            DeformableBody surface = Triangle();
            DeformableBody point = Particle(0.2f, -0.2f, 0.2f);
            PhysicsWorld world = World(1);
            world.AddBody(point);
            world.AddBody(surface);
            world.Tick(0.02f);

            Assert.AreEqual(0f, PlaneDistance(surface, point.Particles[0].Position), 1e-3f);
            Assert.Less(TriangleMeanY(surface), -0.01f);
            Assert.Greater(point.Particles[0].Position.Y, -0.19f);
            Assert.Less(point.Particles[0].Position.Y, -0.01f);
            Assert.AreEqual(0.2f, point.Particles[0].Position.X, 1e-3f);
        }

        [Test]
        public void AFixedTriangleDoesNotMove() {
            DeformableBody surface = Triangle();
            for (int i = 0; i < surface.Particles.Length; i++) {
                Particle pinned = surface.Particles[i];
                pinned.InverseMass = 0f;
                surface.Particles[i] = pinned;
            }

            DeformableBody point = Particle(0.2f, -0.2f, 0.2f);
            PhysicsWorld world = World(1);
            world.AddBody(point);
            world.AddBody(surface);
            world.Tick(0.02f);

            Assert.AreEqual(0f, point.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(0f, surface.Particles[0].Position.Y, 1e-5f);
            Assert.AreEqual(0f, surface.Particles[1].Position.Y, 1e-5f);
            Assert.AreEqual(0f, surface.Particles[2].Position.Y, 1e-5f);
        }

        [Test]
        public void OutsideAndTheSameBodyDoNotCollide() {
            DeformableBody surface = Triangle();
            DeformableBody above = Particle(0.2f, 0.2f, 0.2f);
            DeformableBody beside = Particle(2f, -0.2f, 2f);
            DeformableBody self = Triangle();
            for (int i = 0; i < self.Particles.Length; i++) {
                Particle shifted = self.Particles[i];
                shifted.Position += new Vec3(10f, 0f, 0f);
                shifted.Previous = shifted.Position;
                self.Particles[i] = shifted;
            }

            var inside = new Particle {
                Position = new Vec3(10.2f, -0.2f, 0.2f),
                Previous = new Vec3(10.2f, -0.2f, 0.2f),
                InverseMass = 1f
            };
            self.Particles = new[] { self.Particles[0], self.Particles[1], self.Particles[2], inside };
            self.Triangles = new[] { 0, 2, 1 };

            PhysicsWorld world = World(1);
            world.AddBody(surface);
            world.AddBody(above);
            world.AddBody(beside);
            world.AddBody(self);
            world.Tick(0.02f);

            Assert.AreEqual(0.2f, above.Particles[0].Position.Y, 1e-5f);
            Assert.AreEqual(2f, beside.Particles[0].Position.X, 1e-5f);
            Assert.AreEqual(-0.2f, self.Particles[3].Position.Y, 1e-5f);
            Assert.AreEqual(10.2f, self.Particles[3].Position.X, 1e-5f);
            Assert.AreEqual(0, world.Collision.ContactCount);
        }

        [Test]
        public void SeparatedSpheresDoNotCollideUntilTheyTouch() {
            BodyMaterial material = Still();
            DeformableBody left = SphereGeometry.Create(1f, 4, 2, material, new Vec3(0f, 0f, 0f));
            DeformableBody right = SphereGeometry.Create(1f, 4, 2, material, new Vec3(2.2f, 0f, 0f));
            Vec3 leftStart = left.Particles[1].Position;
            Vec3 rightStart = right.Particles[3].Position;

            PhysicsWorld world = World(4);
            world.AddBody(left);
            world.AddBody(right);
            world.Tick(0.02f);

            Assert.AreEqual(0, world.Collision.ContactCount);
            Assert.AreEqual(leftStart.X, left.Particles[1].Position.X, 1e-4f);
            Assert.AreEqual(rightStart.X, right.Particles[3].Position.X, 1e-4f);
            Assert.Greater(Clearance(left, right), 0f);
        }

        [Test]
        public void OverlappingSpheresSeparateWhenTheSurfacesCross() {
            BodyMaterial material = Still();
            DeformableBody left = SphereGeometry.Create(1f, 4, 2, material, new Vec3(0f, 0f, 0f));
            DeformableBody right = SphereGeometry.Create(1f, 4, 2, material, new Vec3(1.5f, 0f, 0f));
            left.AddConstraint(new ShapeMatchingConstraint(left, 0.0005f));
            right.AddConstraint(new ShapeMatchingConstraint(right, 0.0005f));
            float started = right.CenterOfMass().X - left.CenterOfMass().X;
            Assert.Less(Clearance(left, right), 0f);

            PhysicsWorld world = World(8);
            world.AddBody(left);
            world.AddBody(right);
            for (int step = 0; step < 10; step++) world.Tick(0.02f);

            Assert.Greater(Clearance(left, right), -0.02f);
            Assert.Greater(right.CenterOfMass().X - left.CenterOfMass().X, started);
        }

        [Test]
        public void ASphereDentsWhenItHitsAWall() {
            BodyMaterial material = Still();
            material.Restitution = 0.2f;
            material.Damping = 1.2f;
            DeformableBody sphere = SphereGeometry.Create(0.4f, 5, 3, material, new Vec3(0f, 1f, 0f));
            sphere.AddConstraint(new ShapeMatchingConstraint(sphere, 0.0008f));
            sphere.AddConstraint(new VolumeConstraint(sphere, 0.0005f));
            float rest = Width(sphere);
            for (int i = 0; i < sphere.Particles.Length; i++) {
                Particle particle = sphere.Particles[i];
                particle.Velocity = new Vec3(6f, 0f, 0f);
                sphere.Particles[i] = particle;
            }

            PhysicsWorld world = World(6);
            world.Collision.AddPlane(CollisionPlane.FromNormalAndPoint(new Vec3(-1f, 0f, 0f), new Vec3(0.55f, 0f, 0f)));
            world.AddBody(sphere);
            float smallest = rest;
            for (int step = 0; step < 12; step++) {
                world.Tick(0.02f);
                smallest = System.Math.Min(smallest, Width(sphere));
            }

            Assert.Less(smallest, rest * 0.9f, "width " + smallest + " rest " + rest);
            Assert.Greater(smallest, rest * 0.35f, "width " + smallest);
            Assert.Less(sphere.CenterOfMass().X, 0.7f);
        }

        [Test]
        public void ASphereCannotSlideThroughAnother() {
            BodyMaterial material = Still();
            material.Damping = 0.4f;
            DeformableBody mover = SphereGeometry.Create(0.5f, 6, 4, material, new Vec3(0f, 1f, 0f));
            DeformableBody blocker = SphereGeometry.Create(0.5f, 6, 4, material, new Vec3(1.25f, 1f, 0f));
            mover.AddConstraint(new ShapeMatchingConstraint(mover, 0.0002f));
            blocker.AddConstraint(new ShapeMatchingConstraint(blocker, 0.0002f));
            mover.AddConstraint(new VolumeConstraint(mover, 0.00015f));
            blocker.AddConstraint(new VolumeConstraint(blocker, 0.00015f));
            for (int i = 0; i < mover.Particles.Length; i++) {
                Particle particle = mover.Particles[i];
                particle.Velocity = new Vec3(6f, 0f, 0f);
                mover.Particles[i] = particle;
            }

            PhysicsWorld world = World(6);
            world.AddBody(mover);
            world.AddBody(blocker);
            float worst = float.PositiveInfinity;
            for (int step = 0; step < 40; step++) {
                world.Tick(0.02f);
                worst = System.Math.Min(worst, Clearance(mover, blocker));
                Assert.Less(mover.CenterOfMass().X, blocker.CenterOfMass().X);
            }

            Assert.Greater(worst, -0.08f, "clearance " + worst);
            Assert.Greater(Extent(mover), 0.32f);
            Assert.Greater(Extent(blocker), 0.32f);
        }

        [Test]
        public void FallingSpheresKeepTheirShapeUntilTheSurfacesMeet() {
            BodyMaterial material = Still();
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 1.2f;
            DeformableBody alone = SphereGeometry.Create(0.4f, 5, 3, material, new Vec3(-2f, 1.6f, 0f));
            DeformableBody left = SphereGeometry.Create(0.4f, 5, 3, material, new Vec3(-0.55f, 1.6f, 0f));
            DeformableBody right = SphereGeometry.Create(0.4f, 5, 3, material, new Vec3(0.55f, 1.6f, 0f));
            alone.AddConstraint(new ShapeMatchingConstraint(alone, 0.0002f));
            left.AddConstraint(new ShapeMatchingConstraint(left, 0.0002f));
            right.AddConstraint(new ShapeMatchingConstraint(right, 0.0002f));
            alone.AddConstraint(new VolumeConstraint(alone, 0.00015f));
            left.AddConstraint(new VolumeConstraint(left, 0.00015f));
            right.AddConstraint(new VolumeConstraint(right, 0.00015f));
            float rest = Extent(alone);
            PhysicsWorld world = World(6);
            world.AddBody(alone);
            world.AddBody(left);
            world.AddBody(right);

            for (int step = 0; step < 25; step++) world.Tick(0.02f);

            Assert.AreEqual(0, world.Collision.ContactCount);
            Assert.Greater(Clearance(left, right), 0.05f);
            Assert.Greater(Extent(alone), rest * 0.9f, "alone " + Extent(alone));
            Assert.Greater(Extent(left), rest * 0.9f, "left " + Extent(left));
            Assert.Greater(Extent(right), rest * 0.9f, "right " + Extent(right));
        }

        [Test]
        public void ASmallPileSquishesWithoutCollapsingToAPoint() {
            BodyMaterial material = Still();
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 1.2f;
            material.Compliance = 0.0004f;
            material.Friction = 0.22f;
            material.Restitution = 0.08f;
            PhysicsWorld world = World(6);
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            var pile = new DeformableBody[3];
            float rest = 0f;
            for (int i = 0; i < pile.Length; i++) {
                pile[i] = SphereGeometry.Create(0.4f, 5, 3, material, new Vec3(0.04f * i, 0.42f + i * 0.78f, 0f));
                pile[i].AddConstraint(new ShapeMatchingConstraint(pile[i], 0.0002f));
                pile[i].AddConstraint(new VolumeConstraint(pile[i], 0.00015f));
                world.AddBody(pile[i]);
                rest = Extent(pile[i]);
            }

            for (int step = 0; step < 90; step++) world.Tick(0.02f);

            float smallest = float.PositiveInfinity;
            float fastest = 0f;
            for (int i = 0; i < pile.Length; i++) {
                smallest = System.Math.Min(smallest, Extent(pile[i]));
                fastest = System.Math.Max(fastest, Fastest(pile[i]));
                Assert.Greater(pile[i].CenterOfMass().Y, 0.15f);
            }

            float separated = (pile[0].CenterOfMass() - pile[2].CenterOfMass()).Length;
            Assert.Greater(smallest, rest * 0.72f, "extent " + smallest + " rest " + rest);
            Assert.Greater(separated, 0.45f, "separation " + separated);
            Assert.Less(fastest, 6f, "speed " + fastest);
        }

        [Test]
        public void AFastSphereStopsOnTheSurfaceInsteadOfPassingThrough() {
            BodyMaterial material = Still();
            DeformableBody mover = SphereGeometry.Create(1f, 4, 2, material, Vec3.Zero);
            DeformableBody blocker = SphereGeometry.Create(1f, 4, 2, material, new Vec3(2.2f, 0f, 0f));
            for (int i = 0; i < mover.Particles.Length; i++) {
                Particle particle = mover.Particles[i];
                particle.Velocity = new Vec3(125f, 0f, 0f);
                mover.Particles[i] = particle;
            }

            PhysicsWorld world = World(2);
            world.AddBody(mover);
            world.AddBody(blocker);
            world.Tick(0.02f);

            for (int i = 0; i < mover.Particles.Length; i++) Assert.IsFalse(mover.Particles[i].Position.HasNaN());

            float nose = mover.Particles[1].Position.X;
            Assert.Greater(world.Collision.ContactCount, 0);
            Assert.Greater(nose, 0.8f, "nose " + nose);
            Assert.Less(nose, 1.8f, "nose " + nose);
            Assert.Greater(Extent(blocker), 0.85f, "blocker extent " + Extent(blocker));
        }

        [Test]
        public void AVertexThatMissesAFaceDoesNotDragTheFaceWithIt() {
            DeformableBody surface = Triangle();
            for (int i = 0; i < surface.Particles.Length; i++) {
                Particle pinned = surface.Particles[i];
                pinned.InverseMass = 0f;
                surface.Particles[i] = pinned;
            }

            var passer = new DeformableBody(3, Still());
            passer.Particles[0] = Placed(-0.1f, 0.05f, 0.2f);
            Particle moving = passer.Particles[0];
            moving.Velocity = new Vec3(0f, -15f, 0f);
            passer.Particles[0] = moving;
            passer.Particles[1] = Placed(0.5f, 0.3f, 0.2f);
            passer.Particles[2] = Placed(-0.1f, 0.5f, 1.5f);
            passer.Triangles = new[] { 0, 1, 2 };

            PhysicsWorld world = World(4);
            world.AddBody(passer);
            world.AddBody(surface);
            world.Tick(0.02f);

            Assert.AreEqual(-0.25f, passer.Particles[0].Position.Y, 1e-3f);
            Assert.AreEqual(0f, surface.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(0f, surface.Particles[1].Position.Y, 1e-4f);
            Assert.AreEqual(0f, surface.Particles[2].Position.Y, 1e-4f);
        }

        [Test]
        public void DroppedSpheresStayInContactInsteadOfFlinging() {
            string report =
                Scenario("one sphere", new Vec3(0f, 2.6f, -6f), true) +
                Pair();
            Assert.IsFalse(report.Contains("BAD"), report);
        }

        static string Pair() {
            BodyMaterial material = Still();
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 1.5f;
            material.Compliance = 0.0005f;
            material.Friction = 0.45f;
            material.Restitution = 0.2f;
            DeformableBody upper = SphereGeometry.Create(0.72f, 8, 5, material, new Vec3(0f, 2.6f, -6f));
            DeformableBody lower = SphereGeometry.Create(0.72f, 8, 5, material, new Vec3(0.15f, 0.72f, -6f));
            upper.AddConstraint(new ShapeMatchingConstraint(upper, 0.0004f));
            lower.AddConstraint(new ShapeMatchingConstraint(lower, 0.0004f));
            upper.AddConstraint(new VolumeConstraint(upper, 0.0005f));
            lower.AddConstraint(new VolumeConstraint(lower, 0.0005f));
            PhysicsWorld world = World(8);
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            world.AddBody(upper);
            world.AddBody(lower);
            float smallest = System.Math.Min(Extent(upper), Extent(lower));
            float fastest = 0f;
            for (int step = 0; step < 120; step++) {
                world.Tick(0.02f);
                smallest = System.Math.Min(smallest, System.Math.Min(Extent(upper), Extent(lower)));
                fastest = System.Math.Max(fastest, System.Math.Max(Fastest(upper), Fastest(lower)));
            }

            string state = smallest > 0.55f && fastest < 15f ? "OK " : "BAD";
            return state + " pair extent " + smallest.ToString("0.00") + " speed " + fastest.ToString("0.0") + "\n";
        }

        static string Scenario(string name, Vec3 center, bool floor) {
            BodyMaterial material = Still();
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 1.5f;
            material.Compliance = 0.0005f;
            material.Friction = 0.45f;
            material.Restitution = 0.2f;
            DeformableBody body = SphereGeometry.Create(0.72f, 8, 5, material, center);
            body.AddConstraint(new ShapeMatchingConstraint(body, 0.0004f));
            body.AddConstraint(new VolumeConstraint(body, 0.0005f));

            PhysicsWorld world = World(8);
            if(floor) world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            world.AddBody(body);

            float smallest = Extent(body);
            float fastest = 0f;
            int worstStep = 0;
            for (int step = 0; step < 80; step++) {
                world.Tick(0.02f);
                float extent = Extent(body);
                float speed = Fastest(body);
                if(extent < smallest) {
                    smallest = extent;
                    worstStep = step;
                }

                if(speed > fastest) fastest = speed;
            }

            string state = smallest > 0.55f && fastest < 20f ? "OK " : "BAD";
            return state + " " + name + " extent " + smallest.ToString("0.00") + " at " + worstStep + " speed " + fastest.ToString("0.0") + "\n";
        }

        [Test]
        public void AVertexBouncesOffAFixedTriangle() {
            DeformableBody surface = Triangle();
            for (int i = 0; i < surface.Particles.Length; i++) {
                Particle pinned = surface.Particles[i];
                pinned.InverseMass = 0f;
                surface.Particles[i] = pinned;
            }

            DeformableBody point = Particle(0.2f, 0.01f, 0.2f);
            Particle moving = point.Particles[0];
            moving.Velocity = new Vec3(0f, -1f, 0f);
            point.Particles[0] = moving;
            point.Material.Restitution = 0.5f;

            PhysicsWorld world = World(1);
            world.AddBody(point);
            world.AddBody(surface);
            world.Tick(0.02f);

            Assert.AreEqual(0f, point.Particles[0].Position.Y, 1e-4f);
            Assert.AreEqual(0.5f, point.Particles[0].Velocity.Y, 0.05f);
            Assert.AreEqual(0f, surface.Particles[0].Position.Y, 1e-5f);
        }

        static float Clearance(DeformableBody left, DeformableBody right) {
            float clearance = float.PositiveInfinity;
            clearance = System.Math.Min(clearance, ClearanceOf(left, right));
            clearance = System.Math.Min(clearance, ClearanceOf(right, left));
            return clearance;
        }

        static float ClearanceOf(DeformableBody mesh, DeformableBody other) {
            float nearest = float.PositiveInfinity;
            for (int i = 0; i < other.Particles.Length; i++) {
                Assert.IsFalse(other.Particles[i].Position.HasNaN());
                int[] triangles = mesh.Triangles;
                float best = float.NegativeInfinity;
                for (int slot = 0; slot + 2 < triangles.Length; slot += 3) {
                    Vec3 a = mesh.Particles[triangles[slot]].Position;
                    Vec3 b = mesh.Particles[triangles[slot + 1]].Position;
                    Vec3 c = mesh.Particles[triangles[slot + 2]].Position;
                    Vec3 raw = Vec3.Cross(b - a, c - a);
                    float length = raw.Length;
                    if(length < 1e-8f) continue;
                    float distance = Vec3.Dot(raw * (1f / length), other.Particles[i].Position - a);
                    if(distance > best) best = distance;
                }

                if(best < nearest) nearest = best;
            }

            return nearest;
        }

        static float Width(DeformableBody body) {
            float min = float.PositiveInfinity;
            float max = float.NegativeInfinity;
            for (int i = 0; i < body.Particles.Length; i++) {
                float x = body.Particles[i].Position.X;
                if(x < min) min = x;
                if(x > max) max = x;
            }

            return max - min;
        }

        static float Extent(DeformableBody body) {
            Vec3 center = body.CenterOfMass();
            float sum = 0f;
            for (int i = 0; i < body.Particles.Length; i++) sum += (body.Particles[i].Position - center).Length;
            return body.Particles.Length == 0 ? 0f : sum / body.Particles.Length;
        }

        static float Fastest(DeformableBody body) {
            float fastest = 0f;
            for (int i = 0; i < body.Particles.Length; i++) {
                Assert.IsFalse(body.Particles[i].Position.HasNaN());
                Assert.IsFalse(body.Particles[i].Velocity.HasNaN());
                float speed = body.Particles[i].Velocity.Length;
                if(speed > fastest) fastest = speed;
            }

            return fastest;
        }

        static float CenterY(DeformableBody point, DeformableBody surface) {
            float sum = point.Particles[0].Position.Y;
            for (int i = 0; i < surface.Particles.Length; i++) sum += surface.Particles[i].Position.Y;
            return sum / (surface.Particles.Length + 1);
        }

        static PhysicsWorld World(int iterations) {
            return new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 0.02f,
                SolverIterations = iterations,
                MaxStepsPerFrame = 4
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

        static float PlaneDistance(DeformableBody surface, Vec3 point) {
            Vec3 a = surface.Particles[surface.Triangles[0]].Position;
            Vec3 b = surface.Particles[surface.Triangles[1]].Position;
            Vec3 c = surface.Particles[surface.Triangles[2]].Position;
            Vec3 normal = Vec3.Cross(b - a, c - a);
            float length = normal.Length;
            if(length < 1e-8f) return 0f;
            normal = normal * (1f / length);
            return Vec3.Dot(normal, point - a);
        }

        static float TriangleMeanY(DeformableBody surface) {
            float sum = 0f;
            for (int i = 0; i < surface.Particles.Length; i++) sum += surface.Particles[i].Position.Y;
            return surface.Particles.Length == 0 ? 0f : sum / surface.Particles.Length;
        }

        static DeformableBody Triangle() {
            var body = new DeformableBody(3, Still());
            body.Particles[0] = Placed(0f, 0f, 0f);
            body.Particles[1] = Placed(1f, 0f, 0f);
            body.Particles[2] = Placed(0f, 0f, 1f);
            body.Triangles = new[] { 0, 2, 1 };
            return body;
        }

        static DeformableBody Particle(float x, float y, float z) {
            var body = new DeformableBody(1, Still());
            body.Particles[0] = Placed(x, y, z);
            return body;
        }

        static Particle Placed(float x, float y, float z) {
            var position = new Vec3(x, y, z);
            return new Particle {
                Position = position,
                Previous = position,
                InverseMass = 1f
            };
        }
    }
}
