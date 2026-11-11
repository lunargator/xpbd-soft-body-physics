using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;
using SoftBodyPhysics.Unity;

namespace SoftBodyPhysics.Samples {

    public static class DeformablePhysicsTestSetup {
        public const float Radius = 0.55f;
        public const int Segments = 6;
        public const int Stacks = 4;
        public const float BlueShape = 0.0016f;
        public const float BlueVolume = 0.00018f;
        public const float AmberShape = 0.0024f;
        public const float AmberVolume = 0.0004f;

        public static readonly Vec3 WallCenter = new Vec3(-1.45f, 1.3f, -6f);
        public static readonly Vec3 WallSize = new Vec3(0.5f, 2.6f, 2.4f);

        public static PhysicsSettings Settings() {
            return new PhysicsSettings {
                FixedDeltaTime = 0.02f,
                SolverIterations = 8,
                MaxStepsPerFrame = 8
            };
        }

        public static PhysicsWorld CreateWorld() {
            PhysicsWorld world = new PhysicsWorld(Settings());
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            world.Collision.AddBox(CollisionBox.FromCenterSize(WallCenter, WallSize));
            return world;
        }

        public static float WallFaceX() { return WallCenter.X + WallSize.X * 0.5f; }

        public static BodyMaterial BlueMaterial() {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 2f;
            material.Compliance = 0.0003f;
            material.Friction = 0.55f;
            material.Restitution = 0.05f;
            return material;
        }

        public static BodyMaterial AmberMaterial() {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 1.1f;
            material.Compliance = 0.0008f;
            material.Friction = 0.3f;
            material.Restitution = 0.35f;
            return material;
        }

        public static DeformableBody CreateSphere(BodyMaterial material, Vec3 center, float shapeCompliance, float volumeCompliance) {
            DeformableBody body = SphereGeometry.Create(Radius, Segments, Stacks, material, center);
            body.AddConstraint(new ShapeMatchingConstraint(body, shapeCompliance));
            body.AddConstraint(new VolumeConstraint(body, volumeCompliance));
            return body;
        }
    }
}
