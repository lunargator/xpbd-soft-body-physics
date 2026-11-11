using System;
using System.Collections.Generic;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Geometry {

    public static class SphereGeometry {
        public static DeformableBody Create(float radius, int segments, int stacks, BodyMaterial material, Vec3 center) {
            if(radius <= 0f) throw new ArgumentOutOfRangeException(nameof(radius));
            if(segments < 3) throw new ArgumentOutOfRangeException(nameof(segments));
            if(stacks < 2) throw new ArgumentOutOfRangeException(nameof(stacks));

            int rings = stacks - 1;
            int count = rings * segments + 2;
            var body = new DeformableBody(count, material);
            body.Particles[0] = ParticleAt(center + new Vec3(0f, radius, 0f));

            for (int ring = 0; ring < rings; ring++) {
                double theta = Math.PI * (ring + 1) / stacks;
                float y = (float)(Math.Cos(theta) * radius);
                float ringRadius = (float)(Math.Sin(theta) * radius);
                for (int segment = 0; segment < segments; segment++) {
                    double phi = 2.0 * Math.PI * segment / segments;
                    var position = center + new Vec3(
                        (float)(Math.Cos(phi) * ringRadius),
                        y,
                        (float)(Math.Sin(phi) * ringRadius));
                    body.Particles[1 + ring * segments + segment] = ParticleAt(position);
                }
            }

            body.Particles[count - 1] = ParticleAt(center + new Vec3(0f, -radius, 0f));

            var triangles = new List<int>(stacks * segments * 6);
            var edges = new HashSet<long>();
            for (int segment = 0; segment < segments; segment++) {
                int next = (segment + 1) % segments;
                AddTriangle(body, triangles, edges, material.Compliance, center, 0, Ring(0, next, segments), Ring(0, segment, segments));
            }

            for (int ring = 0; ring < rings - 1; ring++) {
                for (int segment = 0; segment < segments; segment++) {
                    int next = (segment + 1) % segments;
                    int a = Ring(ring, segment, segments);
                    int b = Ring(ring, next, segments);
                    int c = Ring(ring + 1, segment, segments);
                    int d = Ring(ring + 1, next, segments);
                    AddTriangle(body, triangles, edges, material.Compliance, center, a, c, b);
                    AddTriangle(body, triangles, edges, material.Compliance, center, b, c, d);
                }
            }

            int south = count - 1;
            int lastRing = rings - 1;
            for (int segment = 0; segment < segments; segment++) {
                int next = (segment + 1) % segments;
                AddTriangle(body, triangles, edges, material.Compliance, center, south, Ring(lastRing, segment, segments), Ring(lastRing, next, segments));
            }

            body.Triangles = triangles.ToArray();
            body.ClosedSurface = true;
            body.Convex = true;
            body.CaptureRestPositions();
            return body;
        }

        static int Ring(int ring, int segment, int segments) { return 1 + ring * segments + segment; }

        static Particle ParticleAt(Vec3 position) {
            return new Particle {
                Position = position,
                Previous = position,
                InverseMass = 1f
            };
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
            if(Vec3.Dot(Cross(b - a, c - a), a - center) < 0f) {
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
