namespace SoftBodyPhysics.Core {
    public struct Particle {
        public Vec3 Position;
        public Vec3 Previous;
        public Vec3 Velocity;
        public float InverseMass;
        public Vec3 Force;
    }
}
