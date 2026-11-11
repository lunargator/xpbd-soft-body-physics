using SoftBodyPhysics.Unity;
using NUnit.Framework;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;

namespace SoftBodyPhysics.Tests {
    public class ColliderPrimitiveTests {
        [Test]
        public void SpherePushesAnInteriorPointOutAndLeavesTheOutsideAlone() {
            PhysicsWorld world = World(1);
            world.Collision.AddSphere(new CollisionSphere { Center = Vec3.Zero, Radius = 1f });
            DeformableBody inside = Point(new Vec3(0f, 0.4f, 0f), Vec3.Zero);
            DeformableBody outside = Point(new Vec3(3f, 0f, 0f), Vec3.Zero);
            world.AddBody(inside);
            world.AddBody(outside);
            world.Step(0.02f);

            Assert.AreEqual(0f, inside.Particles[0].Position.X, 1e-3f);
            Assert.AreEqual(1f, inside.Particles[0].Position.Y, 1e-3f);
            Assert.AreEqual(0f, inside.Particles[0].Position.Z, 1e-3f);
            Assert.AreEqual(3f, outside.Particles[0].Position.X, 1e-4f);
        }

        [Test]
        public void FastParticleStopsOnTheNearSideOfASphere() {
            PhysicsWorld world = World(2);
            world.Collision.AddSphere(new CollisionSphere { Center = Vec3.Zero, Radius = 1f });
            DeformableBody body = Point(new Vec3(0f, 2f, 0f), new Vec3(0f, -200f, 0f));
            world.AddBody(body);
            world.Step(0.02f);

            Assert.AreEqual(0f, body.Particles[0].Position.X, 1e-2f);
            Assert.AreEqual(1f, body.Particles[0].Position.Y, 2e-2f);
            Assert.AreEqual(0f, body.Particles[0].Position.Z, 1e-2f);
        }

        [Test]
        public void CapsulePushesAnInteriorPointOutRadially() {
            PhysicsWorld world = World(1);
            world.Collision.AddCapsule(new CollisionCapsule {
                Start = new Vec3(0f, -1f, 0f),
                End = new Vec3(0f, 1f, 0f),
                Radius = 0.5f
            });
            DeformableBody body = Point(new Vec3(0.2f, 0f, 0f), Vec3.Zero);
            world.AddBody(body);
            world.Step(0.02f);

            Assert.AreEqual(0.5f, body.Particles[0].Position.X, 1e-3f);
            Assert.AreEqual(0f, body.Particles[0].Position.Y, 1e-3f);
            Assert.AreEqual(0f, body.Particles[0].Position.Z, 1e-3f);
        }

        [Test]
        public void OrientedBoxMatchesAnAxisAlignedExit() {
            PhysicsWorld world = World(1);
            world.Collision.AddOrientedBox(new CollisionOrientedBox {
                Center = Vec3.Zero,
                AxisX = new Vec3(1f, 0f, 0f),
                AxisY = new Vec3(0f, 1f, 0f),
                AxisZ = new Vec3(0f, 0f, 1f)
            });
            DeformableBody body = Point(new Vec3(0f, 0.7f, 0.2f), Vec3.Zero);
            world.AddBody(body);
            world.Step(0.02f);

            Assert.AreEqual(0f, body.Particles[0].Position.X, 1e-3f);
            Assert.AreEqual(1f, body.Particles[0].Position.Y, 1e-3f);
            Assert.AreEqual(0.2f, body.Particles[0].Position.Z, 1e-3f);
        }

        [Test]
        public void SoftSphereStaysAboveAWideStaticFloor() {
            var floor = new CollisionOrientedBox {
                Center = new Vec3(0f, -0.15f, 0f),
                AxisX = new Vec3(6f, 0f, 0f),
                AxisY = new Vec3(0f, 0.15f, 0f),
                AxisZ = new Vec3(0f, 0f, 5f)
            };
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 0.28f;
            material.Compliance = 0.002f;
            material.Friction = 0.65f;
            material.Restitution = 0.05f;
            DeformableBody lower = SphereGeometry.Create(0.22f, 6, 4, material, new Vec3(0f, 0.8f, 0f));
            DeformableBody upper = SphereGeometry.Create(0.22f, 6, 4, material, new Vec3(0.05f, 1.5f, 0f));
            lower.AddConstraint(new ShapeMatchingConstraint(lower, 0.003f));
            lower.AddConstraint(new VolumeConstraint(lower, 0.002f));
            upper.AddConstraint(new ShapeMatchingConstraint(upper, 0.003f));
            upper.AddConstraint(new VolumeConstraint(upper, 0.002f));
            var world = new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 1f / 60f,
                SolverIterations = 4,
                MaxStepsPerFrame = 1
            });
            world.Collision.AddOrientedBox(floor);
            world.AddBody(lower);
            world.AddBody(upper);
            for (int step = 0; step < 180; step++) {
                world.Tick(1f / 60f);
                AssertAboveFloor(lower);
                AssertAboveFloor(upper);
            }
        }

        [Test]
        public void RotatedBoxPushesThroughTheNearestFace() {
            float s = 0.70710678f;
            PhysicsWorld world = World(2);
            world.Collision.AddOrientedBox(new CollisionOrientedBox {
                Center = Vec3.Zero,
                AxisX = new Vec3(s, s, 0f),
                AxisY = new Vec3(-s, s, 0f),
                AxisZ = new Vec3(0f, 0f, 1f)
            });
            DeformableBody body = Point(new Vec3(s * 0.6f, s * 0.6f, 0f), Vec3.Zero);
            world.AddBody(body);
            world.Step(0.02f);

            Assert.AreEqual(s, body.Particles[0].Position.X, 2e-2f);
            Assert.AreEqual(s, body.Particles[0].Position.Y, 2e-2f);
            Assert.AreEqual(0f, body.Particles[0].Position.Z, 1e-3f);
        }

        [Test]
        public void TriangleFloorStopsAPointFromBelowAndFromAFastFall() {
            var floor = new CollisionTriangle {
                A = new Vec3(-5f, 0f, -5f),
                B = new Vec3(0f, 0f, 5f),
                C = new Vec3(5f, 0f, -5f),
                Closed = 1
            };
            PhysicsWorld sunk = World(1);
            sunk.Collision.AddTriangle(floor);
            DeformableBody below = Point(new Vec3(0f, -0.25f, -1f), Vec3.Zero);
            sunk.AddBody(below);
            sunk.Step(0.02f);
            Assert.AreEqual(0f, below.Particles[0].Position.Y, 1e-3f);

            PhysicsWorld fast = World(2);
            fast.Collision.AddTriangle(floor);
            DeformableBody falling = Point(new Vec3(0f, 1.5f, -1f), new Vec3(0f, -200f, 0f));
            fast.AddBody(falling);
            fast.Step(0.02f);
            Assert.AreEqual(0f, falling.Particles[0].Position.Y, 2e-2f);
            Assert.Greater(falling.Particles[0].Position.Y, -0.02f);
        }

        [Test]
        public void MovingBoxCarriesAPointUp() {
            var box = new CollisionOrientedBox {
                Center = new Vec3(0f, -1f, 0f),
                AxisX = new Vec3(1.5f, 0f, 0f),
                AxisY = new Vec3(0f, 1f, 0f),
                AxisZ = new Vec3(0f, 0f, 1.5f)
            };
            PhysicsWorld world = World(2);
            world.Collision.AddOrientedBox(box);
            DeformableBody body = Point(new Vec3(0f, -0.05f, 0f), Vec3.Zero);
            world.AddBody(body);
            world.Step(0.02f);
            Assert.AreEqual(0f, body.Particles[0].Position.Y, 1e-3f);

            world.Collision.ClearSolids();
            box.Center = new Vec3(0f, -0.95f, 0f);
            box.Motion = new Vec3(0f, 0.05f, 0f);
            world.Collision.AddOrientedBox(box);
            world.Step(0.02f);
            Assert.Greater(body.Particles[0].Position.Y, 0.04f);
            Assert.Less(body.Particles[0].Position.Y, 0.08f);
        }

        [Test]
        public void SlidingBoxCarriesAPointSideways() {
            var box = new CollisionOrientedBox {
                Center = new Vec3(0f, -1f, 0f),
                AxisX = new Vec3(1.5f, 0f, 0f),
                AxisY = new Vec3(0f, 1f, 0f),
                AxisZ = new Vec3(0f, 0f, 1.5f)
            };
            PhysicsWorld world = World(2);
            world.Collision.AddOrientedBox(box);
            DeformableBody body = Point(new Vec3(0f, -0.05f, 0f), Vec3.Zero);
            world.AddBody(body);
            world.Step(0.02f);
            Assert.AreEqual(0f, body.Particles[0].Position.X, 1e-3f);
            Assert.AreEqual(0f, body.Particles[0].Position.Y, 1e-3f);

            Particle sunk = body.Particles[0];
            sunk.Position = new Vec3(0f, -0.01f, 0f);
            sunk.Previous = sunk.Position;
            sunk.Velocity = Vec3.Zero;
            body.Particles[0] = sunk;

            world.Collision.ClearSolids();
            box.Center = new Vec3(0.08f, -1f, 0f);
            box.Motion = new Vec3(0.08f, 0f, 0f);
            box.Pose = new ColliderPose {
                HasBasis = 1,
                Origin = new Vec3(0.08f, -1f, 0f),
                X = new Vec3(1f, 0f, 0f),
                Y = new Vec3(0f, 1f, 0f),
                Z = new Vec3(0f, 0f, 1f),
                PreviousOrigin = new Vec3(0f, -1f, 0f),
                PreviousX = new Vec3(1f, 0f, 0f),
                PreviousY = new Vec3(0f, 1f, 0f),
                PreviousZ = new Vec3(0f, 0f, 1f)
            };
            world.Collision.AddOrientedBox(box);
            world.Step(0.02f);

            Assert.Greater(body.Particles[0].Position.X, 0.05f, "x " + body.Particles[0].Position.X);
            Assert.Less(body.Particles[0].Position.X, 0.12f, "x " + body.Particles[0].Position.X);
            Assert.AreEqual(0f, body.Particles[0].Position.Y, 0.03f);
            Assert.Less(System.Math.Abs(body.Particles[0].Velocity.X), 1f);
        }

        [Test]
        public void OpenTriangleCatchesAFallFromEitherSide() {
            var flipped = new CollisionTriangle {
                A = new Vec3(-5f, 0f, -5f),
                B = new Vec3(5f, 0f, -5f),
                C = new Vec3(0f, 0f, 5f)
            };
            PhysicsWorld world = World(2);
            world.Collision.AddTriangle(flipped);
            DeformableBody falling = Point(new Vec3(0f, 1.2f, -1f), new Vec3(0f, -80f, 0f));
            DeformableBody above = Point(new Vec3(1f, 1.5f, -1f), Vec3.Zero);
            world.AddBody(falling);
            world.AddBody(above);
            world.Step(0.02f);

            Assert.AreEqual(0f, falling.Particles[0].Position.Y, 3e-2f);
            Assert.AreEqual(1.5f, above.Particles[0].Position.Y, 1e-3f);
        }

        [Test]
        public void RisingBoxDoesNotPullAPointUpThroughItself() {
            var box = new CollisionOrientedBox {
                Center = new Vec3(0f, 0.7f, 0f),
                AxisX = new Vec3(1f, 0f, 0f),
                AxisY = new Vec3(0f, 0.1f, 0f),
                AxisZ = new Vec3(0f, 0f, 1f),
                Motion = new Vec3(0f, 0.8f, 0f)
            };
            PhysicsWorld world = World(2);
            world.Collision.AddOrientedBox(box);
            DeformableBody body = Point(new Vec3(0f, 0.01f, 0f), Vec3.Zero);
            world.AddBody(body);
            world.Step(0.02f);

            Assert.Greater(body.Particles[0].Position.Y, -0.02f);
            Assert.Less(body.Particles[0].Position.Y, 0.08f);
        }

        [Test]
        public void DescendingBoxDoesNotPullARiderThroughTheBottom() {
            var box = new CollisionOrientedBox {
                Center = new Vec3(0f, -0.2f, 0f),
                AxisX = new Vec3(1f, 0f, 0f),
                AxisY = new Vec3(0f, 0.1f, 0f),
                AxisZ = new Vec3(0f, 0f, 1f),
                Motion = new Vec3(0f, -0.4f, 0f)
            };
            PhysicsWorld world = World(2);
            world.Collision.AddOrientedBox(box);
            DeformableBody body = Point(new Vec3(0f, 0.16f, 0f), Vec3.Zero);
            world.AddBody(body);
            world.Step(0.02f);

            Assert.AreEqual(0.16f, body.Particles[0].Position.Y, 0.02f);
        }

        [Test]
        public void ThinWallPushesBackThroughTheEnteredFace() {
            PhysicsWorld world = World(2);
            world.Collision.AddOrientedBox(new CollisionOrientedBox {
                Center = Vec3.Zero,
                AxisX = new Vec3(0.075f, 0f, 0f),
                AxisY = new Vec3(0f, 1f, 0f),
                AxisZ = new Vec3(0f, 0f, 1f)
            });
            DeformableBody body = Point(new Vec3(-0.2f, 0f, 0f), new Vec3(12.5f, 0f, 0f));
            world.AddBody(body);
            world.Step(0.02f);

            Assert.Less(body.Particles[0].Position.X, -0.05f);
            Assert.Greater(body.Particles[0].Position.X, -0.1f);
        }

        [Test]
        public void ClosedFaceDoesNotPullADistantPoint() {
            PhysicsWorld world = World(1);
            world.Collision.AddTriangle(new CollisionTriangle {
                A = new Vec3(-0.4f, 0f, -0.4f),
                B = new Vec3(0f, 0f, 0.4f),
                C = new Vec3(0.4f, 0f, -0.4f),
                Closed = 1
            });
            DeformableBody body = Point(new Vec3(0f, -1.2f, 0f), Vec3.Zero);
            world.AddBody(body);
            world.Step(0.02f);

            Assert.AreEqual(-1.2f, body.Particles[0].Position.Y, 1e-3f);
        }

        [Test]
        public void SpinningBoxDragsAPointAcrossItsTop() {
            float s = 0.70710678f;
            var box = new CollisionOrientedBox {
                Center = new Vec3(0f, -0.1f, 0f),
                AxisX = new Vec3(1.2f, 0f, 0f),
                AxisY = new Vec3(0f, 0.1f, 0f),
                AxisZ = new Vec3(0f, 0f, 1.2f),
                Pose = new ColliderPose {
                    HasBasis = 1,
                    Origin = new Vec3(0f, -0.1f, 0f),
                    X = new Vec3(1f, 0f, 0f),
                    Y = new Vec3(0f, 1f, 0f),
                    Z = new Vec3(0f, 0f, 1f),
                    PreviousOrigin = new Vec3(0f, -0.1f, 0f),
                    PreviousX = new Vec3(s, 0f, -s),
                    PreviousY = new Vec3(0f, 1f, 0f),
                    PreviousZ = new Vec3(s, 0f, s)
                }
            };
            PhysicsWorld world = World(2);
            world.Collision.AddOrientedBox(box);
            DeformableBody body = Point(new Vec3(0.5f, -0.02f, 0f), Vec3.Zero);
            world.AddBody(body);
            world.Step(0.02f);

            Assert.Greater(System.Math.Abs(body.Particles[0].Position.Z), 0.15f);
            float x = body.Particles[0].Position.X;
            float z = body.Particles[0].Position.Z;
            float radius = (float)System.Math.Sqrt(x * x + z * z);
            Assert.AreEqual(0.5f, radius, 0.04f);
            float vx = body.Particles[0].Velocity.X;
            float vz = body.Particles[0].Velocity.Z;
            Assert.Less((float)System.Math.Sqrt(vx * vx + vz * vz), 1f);
        }

        [Test]
        public void OpenSlopeDoesNotPullADistantPointThroughItself() {
            var slope = new CollisionTriangle {
                A = new Vec3(0f, 0.05f, -0.75f),
                B = new Vec3(-0.9f, 0.5f, -0.75f),
                C = new Vec3(-0.9f, 0.5f, 0.75f)
            };
            PhysicsWorld world = World(2);
            world.Collision.AddTriangle(slope);
            DeformableBody body = Point(new Vec3(-0.7f, 0.15f, 0f), Vec3.Zero);
            world.AddBody(body);
            world.Step(0.02f);

            Assert.AreEqual(-0.7f, body.Particles[0].Position.X, 0.02f);
            Assert.AreEqual(0.15f, body.Particles[0].Position.Y, 0.02f);
        }

        [Test]
        public void OpenSlopeBlocksARiseFromUnderneath() {
            var slope = new CollisionTriangle {
                A = new Vec3(0f, 0.05f, -0.75f),
                B = new Vec3(-0.9f, 0.5f, -0.75f),
                C = new Vec3(-0.9f, 0.5f, 0.75f)
            };
            PhysicsWorld world = World(4);
            world.Collision.AddTriangle(slope);
            DeformableBody body = Point(new Vec3(-0.45f, 0.02f, 0f), new Vec3(0f, 12f, 0f));
            world.AddBody(body);
            world.Step(1f / 60f);
            world.Step(1f / 60f);

            Assert.Less(body.Particles[0].Position.Y, 0.34f);
        }

        [Test]
        public void OpenSlopeBlocksASphereRisingFromUnderneath() {
            var slope = new CollisionTriangle {
                A = new Vec3(0f, 0.05f, -0.75f),
                B = new Vec3(-0.9f, 0.5f, -0.75f),
                C = new Vec3(-0.9f, 0.5f, 0.75f)
            };
            var slopeFront = new CollisionTriangle {
                A = new Vec3(0f, 0.05f, -0.75f),
                B = new Vec3(-0.9f, 0.5f, 0.75f),
                C = new Vec3(0f, 0.05f, 0.75f)
            };
            PhysicsWorld world = World(4);
            world.Collision.AddTriangle(slope);
            world.Collision.AddTriangle(slopeFront);
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = Vec3.Zero;
            material.Damping = 0f;
            material.Friction = 0f;
            material.Restitution = 0f;
            DeformableBody body = SphereGeometry.Create(0.18f, 4, 2, material, new Vec3(-0.45f, 0.02f, 0f));
            body.AddConstraint(new ShapeMatchingConstraint(body, 0.0004f));
            body.AddConstraint(new VolumeConstraint(body, 0.0004f));
            for (int i = 0; i < body.Particles.Length; i++) {
                Particle particle = body.Particles[i];
                particle.Velocity = new Vec3(0f, 10f, 0f);
                body.Particles[i] = particle;
            }

            world.AddBody(body);
            for (int step = 0; step < 6; step++) world.Step(1f / 60f);

            Vec3 center = body.CenterOfMass();
            Vec3 edgeU = slope.B - slope.A;
            Vec3 edgeV = slope.C - slope.A;
            Vec3 raw = Vec3.Cross(edgeU, edgeV);
            Vec3 normal = raw * (1f / raw.Length);
            float side = Vec3.Dot(normal, center) - Vec3.Dot(normal, slope.A);
            Assert.Less(side, 0.04f);
        }

        [Test]
        public void SpinningBoxTurnsAParticleThatIsNotTouchingIt() {
            float s = 0.70710678f;
            var box = new CollisionOrientedBox {
                Center = new Vec3(0f, -0.1f, 0f),
                AxisX = new Vec3(1.2f, 0f, 0f),
                AxisY = new Vec3(0f, 0.1f, 0f),
                AxisZ = new Vec3(0f, 0f, 1.2f),
                Pose = new ColliderPose {
                    HasBasis = 1,
                    Origin = new Vec3(0f, -0.1f, 0f),
                    X = new Vec3(1f, 0f, 0f),
                    Y = new Vec3(0f, 1f, 0f),
                    Z = new Vec3(0f, 0f, 1f),
                    PreviousOrigin = new Vec3(0f, -0.1f, 0f),
                    PreviousX = new Vec3(s, 0f, -s),
                    PreviousY = new Vec3(0f, 1f, 0f),
                    PreviousZ = new Vec3(s, 0f, s)
                }
            };
            PhysicsWorld world = World(2);
            world.Collision.AddOrientedBox(box);
            DeformableBody body = Point(new Vec3(0.5f, -0.02f, 0f), Vec3.Zero);
            var raised = new Vec3(0.5f, 0.45f, 0f);
            body.Particles = new[] {
                body.Particles[0],
                new Particle {
                    Position = raised,
                    Previous = raised,
                    InverseMass = 1f
                }
            };
            body.RestPositions = new[] { body.Particles[0].Position, raised };
            world.AddBody(body);
            world.Step(0.02f);

            Assert.Greater(System.Math.Abs(body.Particles[0].Position.Z), 0.15f);
            Assert.Greater(System.Math.Abs(body.Particles[1].Position.Z), 0.15f);
        }

        static void AssertAboveFloor(DeformableBody body) {
            for (int i = 0; i < body.Particles.Length; i++) {
                Assert.IsFalse(body.Particles[i].Position.HasNaN());
                Assert.GreaterOrEqual(body.Particles[i].Position.Y, -0.002f);
            }
        }

        static PhysicsWorld World(int iterations) {
            return new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 0.02f,
                SolverIterations = iterations,
                MaxStepsPerFrame = 1
            });
        }

        static DeformableBody Point(Vec3 position, Vec3 velocity) {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = Vec3.Zero;
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
