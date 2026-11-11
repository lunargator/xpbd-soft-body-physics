using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Constraints {

    public sealed class DistanceConstraint : IConstraint {
        public int IndexA;
        public int IndexB;
        public float RestLength;
        public float Compliance;
        float lambda;

        public DistanceConstraint(int indexA, int indexB, float restLength, float compliance) {
            IndexA = indexA;
            IndexB = indexB;
            RestLength = restLength < 0f ? 0f : restLength;
            Compliance = compliance < 0f ? 0f : compliance;
        }

        public static DistanceConstraint Create(DeformableBody body, int indexA, int indexB) {
            return Create(body, indexA, indexB, body.Material.Compliance);
        }

        public static DistanceConstraint Create(DeformableBody body, int indexA, int indexB, float compliance) {
            Vec3 delta = body.Particles[indexA].Position - body.Particles[indexB].Position;
            return new DistanceConstraint(indexA, indexB, delta.Length, compliance);
        }

        public void ResetLambda() { lambda = 0f; }

        public void Solve(DeformableBody body, float stepSeconds) {
            Particle[] particles = body.Particles;
            if(IndexA == IndexB
                || IndexA < 0
                || IndexB < 0
                || IndexA >= particles.Length
                || IndexB >= particles.Length
                || stepSeconds <= 0f)
                return;

            Particle a = particles[IndexA];
            Particle b = particles[IndexB];
            float inverseMass = a.InverseMass + b.InverseMass;
            if(inverseMass <= 0f) return;

            Vec3 delta = a.Position - b.Position;
            float length = delta.Length;
            if(length <= 1e-8f) return;

            Vec3 gradient = delta * (1f / length);
            float error = length - RestLength;
            float alpha = Compliance / (stepSeconds * stepSeconds);
            float deltaLambda = (-error - alpha * lambda) / (inverseMass + alpha);
            lambda += deltaLambda;

            a.Position += gradient * (a.InverseMass * deltaLambda);
            b.Position -= gradient * (b.InverseMass * deltaLambda);
            particles[IndexA] = a;
            particles[IndexB] = b;
        }
    }
}
