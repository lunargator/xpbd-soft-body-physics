using System.Collections.Generic;
using SoftBodyPhysics.Constraints;

namespace SoftBodyPhysics.Core {

    public sealed class DeformableBody {
        public BodyMaterial Material;
        public Particle[] Particles;
        public Vec3[] RestPositions;
        public int[] Triangles;
        public bool ClosedSurface;
        public bool Convex;
        public readonly List<IConstraint> Constraints = new List<IConstraint>();

        public DeformableBody(int particleCount, BodyMaterial material) {
            Material = material;
            Particles = new Particle[particleCount];
            RestPositions = new Vec3[particleCount];
            Triangles = System.Array.Empty<int>();
        }

        public void AddConstraint(IConstraint constraint) { if(constraint != null) Constraints.Add(constraint); }

        public void ApplyImpulse(int particleIndex, Vec3 impulse) {
            if(particleIndex < 0 || particleIndex >= Particles.Length || impulse.HasNaN()) return;

            Particle particle = Particles[particleIndex];
            if(particle.InverseMass <= 0f) return;

            particle.Velocity += impulse * particle.InverseMass;
            Particles[particleIndex] = particle;
        }

        public void CaptureRestPositions() { for (int i = 0; i < Particles.Length; i++) RestPositions[i] = Particles[i].Position; }

        public Vec3 CenterOfMass() {
            Vec3 weighted = Vec3.Zero;
            float mass = 0f;
            for (int i = 0; i < Particles.Length; i++) {
                float inverseMass = Particles[i].InverseMass;
                if(inverseMass <= 0f) continue;
                float particleMass = 1f / inverseMass;
                weighted += Particles[i].Position * particleMass;
                mass += particleMass;
            }

            if(mass <= 0f) {
                if(Particles.Length == 0) return Vec3.Zero;
                Vec3 average = Vec3.Zero;
                for (int i = 0; i < Particles.Length; i++) average += Particles[i].Position;
                return average * (1f / Particles.Length);
            }

            return weighted * (1f / mass);
        }
    }
}
