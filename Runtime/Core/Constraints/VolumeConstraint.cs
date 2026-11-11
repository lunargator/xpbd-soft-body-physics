using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Constraints {

    public sealed class VolumeConstraint : IConstraint {
        public float Compliance;
        public float RestVolume;
        readonly Vec3[] gradient;
        float lambda;

        public VolumeConstraint(DeformableBody body, float compliance) {
            Compliance = compliance < 0f ? 0f : compliance;
            RestVolume = SignedVolume(body.RestPositions, body.Triangles);
            gradient = new Vec3[body.Particles.Length];
        }

        public static float SignedVolume(Vec3[] positions, int[] triangles) {
            double sum = 0.0;
            for (int i = 0; i + 2 < triangles.Length; i += 3) {
                Vec3 a = positions[triangles[i]];
                Vec3 b = positions[triangles[i + 1]];
                Vec3 c = positions[triangles[i + 2]];
                sum += (double)a.X * (b.Y * c.Z - b.Z * c.Y)
                    + (double)a.Y * (b.Z * c.X - b.X * c.Z)
                    + (double)a.Z * (b.X * c.Y - b.Y * c.X);
            }

            return (float)(sum / 6.0);
        }

        public static float SignedVolume(Particle[] particles, int[] triangles) {
            double sum = 0.0;
            for (int i = 0; i + 2 < triangles.Length; i += 3) {
                Vec3 a = particles[triangles[i]].Position;
                Vec3 b = particles[triangles[i + 1]].Position;
                Vec3 c = particles[triangles[i + 2]].Position;
                sum += (double)a.X * (b.Y * c.Z - b.Z * c.Y)
                    + (double)a.Y * (b.Z * c.X - b.X * c.Z)
                    + (double)a.Z * (b.X * c.Y - b.Y * c.X);
            }

            return (float)(sum / 6.0);
        }

        public static void AccumulateGradient(Vec3[] positions, int[] triangles, Vec3[] gradient) {
            if(positions == null || triangles == null || gradient == null) return;

            const float sixth = 1f / 6f;
            for (int i = 0; i + 2 < triangles.Length; i += 3) {
                int indexA = triangles[i];
                int indexB = triangles[i + 1];
                int indexC = triangles[i + 2];
                if(indexA < 0 || indexB < 0 || indexC < 0
                    || indexA >= positions.Length || indexB >= positions.Length || indexC >= positions.Length
                    || indexA >= gradient.Length || indexB >= gradient.Length || indexC >= gradient.Length)
                    continue;

                AddFace(positions[indexA], positions[indexB], positions[indexC], gradient, indexA, indexB, indexC, sixth);
            }
        }

        public static void AccumulateGradient(Particle[] particles, int[] triangles, Vec3[] gradient) {
            if(particles == null || triangles == null || gradient == null) return;

            const float sixth = 1f / 6f;
            for (int i = 0; i + 2 < triangles.Length; i += 3) {
                int indexA = triangles[i];
                int indexB = triangles[i + 1];
                int indexC = triangles[i + 2];
                if(indexA < 0 || indexB < 0 || indexC < 0
                    || indexA >= particles.Length || indexB >= particles.Length || indexC >= particles.Length
                    || indexA >= gradient.Length || indexB >= gradient.Length || indexC >= gradient.Length)
                    continue;

                AddFace(particles[indexA].Position, particles[indexB].Position, particles[indexC].Position, gradient, indexA, indexB, indexC, sixth);
            }
        }

        static void AddFace(Vec3 a, Vec3 b, Vec3 c, Vec3[] gradient, int indexA, int indexB, int indexC, float sixth) {
            gradient[indexA] += Cross(b, c) * sixth;
            gradient[indexB] += Cross(c, a) * sixth;
            gradient[indexC] += Cross(a, b) * sixth;
        }

        public void ResetLambda() { lambda = 0f; }

        public void Solve(DeformableBody body, float stepSeconds) {
            int[] triangles = body.Triangles;
            Particle[] particles = body.Particles;
            if(stepSeconds <= 0f || triangles.Length < 3 || gradient.Length != particles.Length) return;

            for (int i = 0; i < gradient.Length; i++) gradient[i] = Vec3.Zero;

            AccumulateGradient(particles, triangles, gradient);

            float volume = SignedVolume(particles, triangles);
            if(RestVolume * volume < 0f) return;

            float error = volume - RestVolume;
            float weight = 0f;
            for (int i = 0; i < particles.Length; i++) {
                float inverseMass = particles[i].InverseMass;
                if(inverseMass <= 0f) continue;
                weight += inverseMass * gradient[i].SqrLength;
            }

            float alpha = Compliance / (stepSeconds * stepSeconds);
            float denominator = weight + alpha;
            if(denominator <= 1e-12f) return;

            float deltaLambda = (-error - alpha * lambda) / denominator;
            lambda += deltaLambda;
            for (int i = 0; i < particles.Length; i++) {
                Particle particle = particles[i];
                if(particle.InverseMass <= 0f) continue;
                particle.Position += gradient[i] * (particle.InverseMass * deltaLambda);
                particles[i] = particle;
            }
        }

        static Vec3 Cross(Vec3 u, Vec3 v) {
            return new Vec3(
                u.Y * v.Z - u.Z * v.Y,
                u.Z * v.X - u.X * v.Z,
                u.X * v.Y - u.Y * v.X);
        }
    }
}
