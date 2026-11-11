using System;
using System.Collections.Generic;
using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Geometry {

    public static class RandomRock {
        public static void Create(int seed, float radius, out Vec3[] vertices, out int[] triangles) {
            if(radius <= 0f) throw new ArgumentOutOfRangeException(nameof(radius), "Rock radius must be positive.");

            const float golden = 1.618034f;
            var verts = new List<Vec3> {
                Unit(-1f, golden, 0f),
                Unit(1f, golden, 0f),
                Unit(-1f, -golden, 0f),
                Unit(1f, -golden, 0f),
                Unit(0f, -1f, golden),
                Unit(0f, 1f, golden),
                Unit(0f, -1f, -golden),
                Unit(0f, 1f, -golden),
                Unit(golden, 0f, -1f),
                Unit(golden, 0f, 1f),
                Unit(-golden, 0f, -1f),
                Unit(-golden, 0f, 1f)
            };

            int[] faces = {
                0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11,
                1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8,
                3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9,
                4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1
            };

            var midpoints = new Dictionary<long, int>();
            var tris = new List<int>(faces.Length * 4);
            for (int i = 0; i < faces.Length; i += 3) {
                int a = faces[i];
                int b = faces[i + 1];
                int c = faces[i + 2];
                int ab = Midpoint(verts, midpoints, a, b);
                int bc = Midpoint(verts, midpoints, b, c);
                int ca = Midpoint(verts, midpoints, c, a);
                tris.Add(a);
                tris.Add(ab);
                tris.Add(ca);
                tris.Add(b);
                tris.Add(bc);
                tris.Add(ab);
                tris.Add(c);
                tris.Add(ca);
                tris.Add(bc);
                tris.Add(ab);
                tris.Add(bc);
                tris.Add(ca);
            }

            const float lump = 0.34f;
            for (int i = 0; i < verts.Count; i++) verts[i] = verts[i] * (radius * (1f + lump * Noise(seed, i)));

            vertices = verts.ToArray();
            triangles = tris.ToArray();
        }

        static int Midpoint(List<Vec3> verts, Dictionary<long, int> midpoints, int a, int b) {
            int low = a < b ? a : b;
            int high = a < b ? b : a;
            long key = ((long)low << 32) | (uint)high;
            if(midpoints.TryGetValue(key, out int index)) return index;

            Vec3 sum = verts[a] + verts[b];
            index = verts.Count;
            verts.Add(sum * (1f / sum.Length));
            midpoints[key] = index;
            return index;
        }

        static Vec3 Unit(float x, float y, float z) {
            float length = (float)Math.Sqrt(x * x + y * y + z * z);
            return new Vec3(x / length, y / length, z / length);
        }

        static float Noise(int seed, int index) {
            unchecked {
                uint n = (uint)(seed * 374761393 + index * 668265263);
                n = (n ^ (n >> 13)) * 1274126177u;
                n ^= n >> 16;
                return (n & 65535) / 65535f * 2f - 1f;
            }
        }
    }
}
