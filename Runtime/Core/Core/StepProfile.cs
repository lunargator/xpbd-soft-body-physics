namespace SoftBodyPhysics.Core {

    public struct StepProfile {
        public float PredictMilliseconds;
        public float ConstraintMilliseconds;
        public float CollisionMilliseconds;
        public float VelocityMilliseconds;
        public int BodyCount;
        public int ParticleCount;

        public float TotalMilliseconds =>
            PredictMilliseconds + ConstraintMilliseconds + CollisionMilliseconds + VelocityMilliseconds;
    }
}
