using System;
using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Constraints {

    public sealed class ShapeMatchingConstraint : IConstraint {
        public float Compliance;
        readonly Vec3[] goals;
        readonly Vec3[] lambda;
        bool goalsReady;

        public ShapeMatchingConstraint(DeformableBody body, float compliance) {
            Compliance = compliance < 0f ? 0f : compliance;
            int count = body.Particles.Length;
            goals = new Vec3[count];
            lambda = new Vec3[count];
        }

        public void ResetLambda() {
            for (int i = 0; i < lambda.Length; i++) lambda[i] = Vec3.Zero;
            goalsReady = false;
        }

        public void Solve(DeformableBody body, float stepSeconds) {
            if(stepSeconds <= 0f || goals.Length != body.Particles.Length) return;
            if(!goalsReady) {
                ComputeGoals(body);
                goalsReady = true;
            }

            float alpha = Compliance / (stepSeconds * stepSeconds);
            Particle[] particles = body.Particles;
            for (int i = 0; i < particles.Length; i++) {
                Particle particle = particles[i];
                if(particle.InverseMass <= 0f) continue;

                Vec3 error = particle.Position - goals[i];
                float denominator = particle.InverseMass + alpha;
                var delta = new Vec3(
                    (-error.X - alpha * lambda[i].X) / denominator,
                    (-error.Y - alpha * lambda[i].Y) / denominator,
                    (-error.Z - alpha * lambda[i].Z) / denominator);
                lambda[i] += delta;
                particle.Position += delta * particle.InverseMass;
                particles[i] = particle;
            }
        }

        void ComputeGoals(DeformableBody body) {
            Particle[] particles = body.Particles;
            Vec3[] rest = body.RestPositions;
            Vec3 restCenter = Vec3.Zero;
            Vec3 center = Vec3.Zero;
            float mass = 0f;
            for (int i = 0; i < particles.Length; i++) {
                float inverseMass = particles[i].InverseMass;
                if(inverseMass <= 0f) continue;
                float particleMass = 1f / inverseMass;
                restCenter += rest[i] * particleMass;
                center += particles[i].Position * particleMass;
                mass += particleMass;
            }

            if(mass <= 0f) {
                for (int i = 0; i < goals.Length; i++) goals[i] = particles[i].Position;
                return;
            }

            float inverse = 1f / mass;
            restCenter *= inverse;
            center *= inverse;

            Mat3 covariance = default;
            for (int i = 0; i < particles.Length; i++) {
                float inverseMass = particles[i].InverseMass;
                if(inverseMass <= 0f) continue;
                float particleMass = 1f / inverseMass;
                Vec3 current = particles[i].Position - center;
                Vec3 reference = rest[i] - restCenter;
                covariance.m00 += current.X * reference.X * particleMass;
                covariance.m01 += current.X * reference.Y * particleMass;
                covariance.m02 += current.X * reference.Z * particleMass;
                covariance.m10 += current.Y * reference.X * particleMass;
                covariance.m11 += current.Y * reference.Y * particleMass;
                covariance.m12 += current.Y * reference.Z * particleMass;
                covariance.m20 += current.Z * reference.X * particleMass;
                covariance.m21 += current.Z * reference.Y * particleMass;
                covariance.m22 += current.Z * reference.Z * particleMass;
            }

            Mat3 rotation = Mat3.PolarRotation(covariance);
            for (int i = 0; i < particles.Length; i++) goals[i] = center + rotation.Multiply(rest[i] - restCenter);
        }

        struct Mat3 {
            public float m00, m01, m02;
            public float m10, m11, m12;
            public float m20, m21, m22;

            public static Mat3 Identity => new Mat3 { m00 = 1f, m11 = 1f, m22 = 1f };

            public Vec3 Multiply(Vec3 value) {
                return new Vec3(
                    m00 * value.X + m01 * value.Y + m02 * value.Z,
                    m10 * value.X + m11 * value.Y + m12 * value.Z,
                    m20 * value.X + m21 * value.Y + m22 * value.Z);
            }

            public static Mat3 PolarRotation(Mat3 covariance) {
                float scale = covariance.m00 * covariance.m00 + covariance.m01 * covariance.m01 + covariance.m02 * covariance.m02
                    + covariance.m10 * covariance.m10 + covariance.m11 * covariance.m11 + covariance.m12 * covariance.m12
                    + covariance.m20 * covariance.m20 + covariance.m21 * covariance.m21 + covariance.m22 * covariance.m22;
                if(scale < 1e-12f) return Identity;

                Mat3 rotation = covariance.Multiply(1f / (float)Math.Sqrt(scale));
                for (int iteration = 0; iteration < 8; iteration++) {
                    if(!rotation.TryInverse(out Mat3 inverse)) return Identity;
                    rotation = Add(rotation, inverse.Transposed()).Multiply(0.5f);
                }

                if(rotation.Determinant() < 0f) {
                    rotation.m02 = -rotation.m02;
                    rotation.m12 = -rotation.m12;
                    rotation.m22 = -rotation.m22;
                }

                return rotation;
            }

            Mat3 Multiply(float scale) {
                return new Mat3 {
                    m00 = m00 * scale, m01 = m01 * scale, m02 = m02 * scale,
                    m10 = m10 * scale, m11 = m11 * scale, m12 = m12 * scale,
                    m20 = m20 * scale, m21 = m21 * scale, m22 = m22 * scale
                };
            }

            static Mat3 Add(Mat3 a, Mat3 b) {
                return new Mat3 {
                    m00 = a.m00 + b.m00, m01 = a.m01 + b.m01, m02 = a.m02 + b.m02,
                    m10 = a.m10 + b.m10, m11 = a.m11 + b.m11, m12 = a.m12 + b.m12,
                    m20 = a.m20 + b.m20, m21 = a.m21 + b.m21, m22 = a.m22 + b.m22
                };
            }

            Mat3 Transposed() {
                return new Mat3 {
                    m00 = m00, m01 = m10, m02 = m20,
                    m10 = m01, m11 = m11, m12 = m21,
                    m20 = m02, m21 = m12, m22 = m22
                };
            }

            float Determinant() {
                return m00 * (m11 * m22 - m12 * m21)
                    - m01 * (m10 * m22 - m12 * m20)
                    + m02 * (m10 * m21 - m11 * m20);
            }

            bool TryInverse(out Mat3 inverse) {
                float c00 = m11 * m22 - m12 * m21;
                float c01 = m12 * m20 - m10 * m22;
                float c02 = m10 * m21 - m11 * m20;
                float c10 = m02 * m21 - m01 * m22;
                float c11 = m00 * m22 - m02 * m20;
                float c12 = m01 * m20 - m00 * m21;
                float c20 = m01 * m12 - m02 * m11;
                float c21 = m02 * m10 - m00 * m12;
                float c22 = m00 * m11 - m01 * m10;
                float determinant = m00 * c00 + m01 * c01 + m02 * c02;
                if(Math.Abs(determinant) < 1e-8f) {
                    inverse = Identity;
                    return false;
                }

                float inverseDeterminant = 1f / determinant;
                inverse = new Mat3 {
                    m00 = c00 * inverseDeterminant, m01 = c10 * inverseDeterminant, m02 = c20 * inverseDeterminant,
                    m10 = c01 * inverseDeterminant, m11 = c11 * inverseDeterminant, m12 = c21 * inverseDeterminant,
                    m20 = c02 * inverseDeterminant, m21 = c12 * inverseDeterminant, m22 = c22 * inverseDeterminant
                };
                return true;
            }
        }
    }
}
