using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Constraints {

    public sealed class AttachmentConstraint : IConstraint {
        public int ParticleIndex;
        public Vec3 Target;
        public float Compliance;
        Vec3 lambda;

        public AttachmentConstraint(int particleIndex, Vec3 target, float compliance) {
            ParticleIndex = particleIndex;
            Target = target;
            Compliance = compliance < 0f ? 0f : compliance;
        }

        public void ResetLambda() { lambda = Vec3.Zero; }

        public void Solve(DeformableBody body, float stepSeconds) {
            if(body == null || stepSeconds <= 0f) return;

            Particle[] particles = body.Particles;
            if(ParticleIndex < 0 || ParticleIndex >= particles.Length) return;

            Particle particle = particles[ParticleIndex];
            if(particle.InverseMass <= 0f) return;

            float alpha = Compliance / (stepSeconds * stepSeconds);
            float denominator = particle.InverseMass + alpha;
            if(denominator <= 0f) return;

            Vec3 error = particle.Position - Target;
            var delta = new Vec3(
                (-error.X - alpha * lambda.X) / denominator,
                (-error.Y - alpha * lambda.Y) / denominator,
                (-error.Z - alpha * lambda.Z) / denominator);
            lambda += delta;
            particle.Position += delta * particle.InverseMass;
            particles[ParticleIndex] = particle;
        }
    }
}
