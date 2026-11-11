namespace SoftBodyPhysics.Core {

    public static class MassDistribution {
        public static void Equal(float[] inverseMass, float totalMass) {
            if(inverseMass == null) return;

            float each = inverseMass.Length > 0 && totalMass > 1e-8f ? totalMass / inverseMass.Length : 0f;
            float inverse = each <= 1e-8f ? 0f : 1f / each;
            for (int i = 0; i < inverseMass.Length; i++) inverseMass[i] = inverse;
        }

        public static void ByArea(float[] inverseMass, Vec3[] positions, int[] triangles, float totalMass) {
            if(inverseMass == null || inverseMass.Length == 0) return;
            if(totalMass <= 1e-8f) {
                for (int i = 0; i < inverseMass.Length; i++) inverseMass[i] = 0f;
                return;
            }

            if(positions == null || triangles == null || positions.Length != inverseMass.Length || triangles.Length < 3) {
                Equal(inverseMass, totalMass);
                return;
            }

            var area = new float[inverseMass.Length];
            float total = 0f;
            for (int i = 0; i + 2 < triangles.Length; i += 3) {
                int i0 = triangles[i];
                int i1 = triangles[i + 1];
                int i2 = triangles[i + 2];
                if(i0 < 0 || i1 < 0 || i2 < 0 || i0 >= positions.Length || i1 >= positions.Length || i2 >= positions.Length) continue;

                Vec3 edge0 = positions[i1] - positions[i0];
                Vec3 edge1 = positions[i2] - positions[i0];
                Vec3 cross = Vec3.Cross(edge0, edge1);
                float face = 0.5f * cross.Length;
                if(face <= 1e-12f) continue;

                float share = face / 3f;
                area[i0] += share;
                area[i1] += share;
                area[i2] += share;
                total += face;
            }

            if(total <= 1e-12f) {
                Equal(inverseMass, totalMass);
                return;
            }

            float floor = total / inverseMass.Length * 0.25f;
            float weighted = 0f;
            for (int i = 0; i < area.Length; i++) {
                if(area[i] < floor) area[i] = floor;
                weighted += area[i];
            }

            for (int i = 0; i < inverseMass.Length; i++) {
                float particleMass = totalMass * (area[i] / weighted);
                inverseMass[i] = particleMass <= 1e-8f ? 0f : 1f / particleMass;
            }
        }
    }
}
