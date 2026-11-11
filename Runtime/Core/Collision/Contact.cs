using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Collision {
    public struct Contact {
        public int BodyIndex;
        public int ParticleIndex;
        public int ShapeKind;
        public int ShapeIndex;
        public int OtherBodyIndex;
        public int OtherParticleIndex;
        public int ThirdParticleIndex;
        public Vec3 Normal;
        public float Offset;
        public Vec3 Anchor;
        public float IncomingNormalSpeed;
        public float Friction;
        public float Restitution;
        public float Lambda;
        public int Seen;
        public int Contained;
    }
}
