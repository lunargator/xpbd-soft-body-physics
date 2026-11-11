namespace SoftBodyPhysics.Core {
    public struct BodyMaterial {
        public Vec3 Gravity;
        public float Damping;
        public float Compliance;
        public float Friction;
        public float Restitution;

        public static BodyMaterial Default => new BodyMaterial {
            Gravity = new Vec3(0f, -9.81f, 0f),
            Damping = 0.05f,
            Compliance = 0.0001f,
            Friction = 0.3f,
            Restitution = 0.05f
        };
    }
}
