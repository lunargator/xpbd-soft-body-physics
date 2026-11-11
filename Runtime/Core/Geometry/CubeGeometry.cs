using System;
using System.Collections.Generic;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Geometry {

    public static class CubeGeometry {
        public static DeformableBody Create(float size, int cells, BodyMaterial material, Vec3 center) {
            if(size <= 0f) throw new ArgumentOutOfRangeException(nameof(size));
            if(cells < 1) throw new ArgumentOutOfRangeException(nameof(cells));

            int n = cells;
            var lookup = new Dictionary<long, int>();
            var positions = new List<Vec3>();
            for (int z = 0; z <= n; z++) {
                for (int y = 0; y <= n; y++) {
                    for (int x = 0; x <= n; x++) {
                        if(x != 0 && x != n && y != 0 && y != n && z != 0 && z != n) continue;
                        lookup[Key(x, y, z)] = positions.Count;
                        float step = size / n;
                        positions.Add(center + new Vec3(
                            (x - n * 0.5f) * step,
                            (y - n * 0.5f) * step,
                            (z - n * 0.5f) * step));
                    }
                }
            }

            var body = new DeformableBody(positions.Count, material);
            for (int i = 0; i < positions.Count; i++) {
                body.Particles[i] = new Particle {
                    Position = positions[i],
                    Previous = positions[i],
                    InverseMass = 1f
                };
            }

            var triangles = new List<int>(n * n * 12);
            var edges = new HashSet<long>();
            AddFace(body, triangles, edges, material.Compliance, center, lookup, n, 0, true);
            AddFace(body, triangles, edges, material.Compliance, center, lookup, n, 0, false);
            AddFace(body, triangles, edges, material.Compliance, center, lookup, n, 1, true);
            AddFace(body, triangles, edges, material.Compliance, center, lookup, n, 1, false);
            AddFace(body, triangles, edges, material.Compliance, center, lookup, n, 2, true);
            AddFace(body, triangles, edges, material.Compliance, center, lookup, n, 2, false);
            body.Triangles = triangles.ToArray();
            body.ClosedSurface = true;
            body.Convex = true;
            body.CaptureRestPositions();
            return body;
        }

        static long Key(int x, int y, int z) { return x + y * 1000L + z * 1000000L; }

        static void AddFace(
            DeformableBody body,
            List<int> triangles,
            HashSet<long> edges,
            float compliance,
            Vec3 center,
            Dictionary<long, int> lookup,
            int n,
            int axis,
            bool positive) {
            int fixedCoord = positive ? n : 0;
            for (int v = 0; v < n; v++) {
                for (int u = 0; u < n; u++) {
                    int a;
                    int b;
                    int c;
                    int d;
                    if(axis == 0) {
                        a = lookup[Key(fixedCoord, u, v)];
                        b = lookup[Key(fixedCoord, u, v + 1)];
                        c = lookup[Key(fixedCoord, u + 1, v)];
                        d = lookup[Key(fixedCoord, u + 1, v + 1)];
                    } else if(axis == 1) {
                        a = lookup[Key(u, fixedCoord, v)];
                        b = lookup[Key(u + 1, fixedCoord, v)];
                        c = lookup[Key(u, fixedCoord, v + 1)];
                        d = lookup[Key(u + 1, fixedCoord, v + 1)];
                    } else {
                        a = lookup[Key(u, v, fixedCoord)];
                        b = lookup[Key(u, v + 1, fixedCoord)];
                        c = lookup[Key(u + 1, v, fixedCoord)];
                        d = lookup[Key(u + 1, v + 1, fixedCoord)];
                    }

                    if(positive) {
                        AddTriangle(body, triangles, edges, compliance, center, a, c, b);
                        AddTriangle(body, triangles, edges, compliance, center, c, d, b);
                    } else {
                        AddTriangle(body, triangles, edges, compliance, center, a, b, c);
                        AddTriangle(body, triangles, edges, compliance, center, b, d, c);
                    }
                }
            }
        }

        static void AddTriangle(
            DeformableBody body,
            List<int> triangles,
            HashSet<long> edges,
            float compliance,
            Vec3 center,
            int i,
            int j,
            int k) {
            Vec3 a = body.Particles[i].Position;
            Vec3 b = body.Particles[j].Position;
            Vec3 c = body.Particles[k].Position;
            Vec3 normal = Cross(b - a, c - a);
            if(Vec3.Dot(normal, a - center) < 0f) {
                int swap = j;
                j = k;
                k = swap;
            }

            triangles.Add(i);
            triangles.Add(j);
            triangles.Add(k);
            AddEdge(body, edges, compliance, i, j);
            AddEdge(body, edges, compliance, j, k);
            AddEdge(body, edges, compliance, k, i);
        }

        static void AddEdge(DeformableBody body, HashSet<long> edges, float compliance, int i, int j) {
            int min = i < j ? i : j;
            int max = i < j ? j : i;
            long key = ((long)min << 32) | (uint)max;
            if(!edges.Add(key)) return;

            float rest = (body.Particles[min].Position - body.Particles[max].Position).Length;
            body.AddConstraint(new DistanceConstraint(min, max, rest, compliance));
        }

        static Vec3 Cross(Vec3 u, Vec3 v) {
            return new Vec3(
                u.Y * v.Z - u.Z * v.Y,
                u.Z * v.X - u.X * v.Z,
                u.X * v.Y - u.Y * v.X);
        }
    }
}
