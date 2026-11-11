using System;
using System.Collections.Generic;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Geometry {

    public static class MeshGeometry {
        public sealed class Build {
            public DeformableBody Body;
            public int[] VertexToParticle;
        }

        public static Build Create(Vec3[] vertices, int[] triangles, float cellSize, BodyMaterial material, Vec3 offset) {
            if(vertices == null || vertices.Length == 0) throw new ArgumentException("The mesh has no vertices.");
            if(triangles == null || triangles.Length < 3) throw new ArgumentException("The mesh has no triangles.");
            if(triangles.Length % 3 != 0) throw new ArgumentException("Triangle indices must come in threes.");
            if(cellSize < 0f) throw new ArgumentOutOfRangeException(nameof(cellSize), "Mesh cell size must be zero or positive.");

            for (int i = 0; i < triangles.Length; i++) {
                int index = triangles[i];
                if(index < 0 || index >= vertices.Length) throw new ArgumentException("Triangle index " + i + " is " + index + ", but the mesh has " + vertices.Length + " vertices.");
            }

            float cell = cellSize > 1e-5f ? cellSize : 1e-4f;
            var cells = new Dictionary<Cell, int>();
            var sums = new List<Vec3>();
            var counts = new List<int>();
            var map = new int[vertices.Length];
            for (int i = 0; i < vertices.Length; i++) {
                Vec3 position = vertices[i];
                var cellKey = new Cell(Quantize(position.X, cell), Quantize(position.Y, cell), Quantize(position.Z, cell));
                if(!cells.TryGetValue(cellKey, out int particle)) {
                    particle = sums.Count;
                    cells.Add(cellKey, particle);
                    sums.Add(position);
                    counts.Add(1);
                } else {
                    sums[particle] += position;
                    counts[particle]++;
                }

                map[i] = particle;
            }

            var remapped = new List<int>(triangles.Length);
            for (int i = 0; i + 2 < triangles.Length; i += 3) {
                int a = map[triangles[i]];
                int b = map[triangles[i + 1]];
                int c = map[triangles[i + 2]];
                if(a == b || b == c || c == a) continue;
                remapped.Add(a);
                remapped.Add(b);
                remapped.Add(c);
            }

            if(remapped.Count < 3) throw new ArgumentException("The mesh has no area after welding.");
            if(sums.Count < 3) throw new ArgumentException("The mesh needs at least 3 particles.");

            var positions = new Vec3[sums.Count];
            for (int i = 0; i < sums.Count; i++) positions[i] = sums[i] * (1f / counts[i]) + offset;

            bool closed = IsClosed(remapped);
            if(closed) {
                float volume = VolumeConstraint.SignedVolume(positions, remapped.ToArray());
                if(Math.Abs(volume) < 1e-5f) closed = false;
                else if(volume < 0f) Flip(remapped);
            }

            DeformableBody body = Assemble(positions, remapped.ToArray(), closed, material);
            return new Build {
                Body = body,
                VertexToParticle = map
            };
        }

        public static DeformableBody Assemble(Vec3[] positions, int[] triangles, bool closed, BodyMaterial material) {
            if(positions == null || positions.Length < 3) throw new ArgumentException("The mesh needs at least 3 particles.");
            if(triangles == null || triangles.Length < 3) throw new ArgumentException("The mesh has no triangles.");

            var body = new DeformableBody(positions.Length, material);
            for (int i = 0; i < positions.Length; i++) {
                body.Particles[i] = new Particle {
                    Position = positions[i],
                    Previous = positions[i],
                    InverseMass = 1f
                };
            }

            body.Triangles = (int[])triangles.Clone();
            body.ClosedSurface = closed;
            body.CaptureRestPositions();
            var edges = new HashSet<long>();
            for (int i = 0; i + 2 < body.Triangles.Length; i += 3) {
                AddEdge(body, edges, material.Compliance, body.Triangles[i], body.Triangles[i + 1]);
                AddEdge(body, edges, material.Compliance, body.Triangles[i + 1], body.Triangles[i + 2]);
                AddEdge(body, edges, material.Compliance, body.Triangles[i + 2], body.Triangles[i]);
            }

            return body;
        }

        static bool IsClosed(List<int> triangles) {
            var uses = new Dictionary<long, int>();
            for (int i = 0; i + 2 < triangles.Count; i += 3) {
                Count(uses, triangles[i], triangles[i + 1]);
                Count(uses, triangles[i + 1], triangles[i + 2]);
                Count(uses, triangles[i + 2], triangles[i]);
            }

            foreach (int count in uses.Values) { if(count != 2) return false; }

            return uses.Count > 0;
        }

        static void Count(Dictionary<long, int> uses, int a, int b) {
            long key = EdgeKey(a, b);
            uses.TryGetValue(key, out int count);
            uses[key] = count + 1;
        }

        static void Flip(List<int> triangles) {
            for (int i = 0; i + 2 < triangles.Count; i += 3) {
                int swap = triangles[i + 1];
                triangles[i + 1] = triangles[i + 2];
                triangles[i + 2] = swap;
            }
        }

        static void AddEdge(DeformableBody body, HashSet<long> edges, float compliance, int i, int j) {
            long key = EdgeKey(i, j);
            if(!edges.Add(key)) return;
            float rest = (body.Particles[i].Position - body.Particles[j].Position).Length;
            body.AddConstraint(new DistanceConstraint(i, j, rest, compliance));
        }

        static long EdgeKey(int i, int j) {
            int min = i < j ? i : j;
            int max = i < j ? j : i;
            return ((long)min << 32) | (uint)max;
        }

        static int Quantize(float value, float cell) { return (int)Math.Round(value / cell); }

        struct Cell : IEquatable<Cell> {
            public readonly int X;
            public readonly int Y;
            public readonly int Z;

            public Cell(int x, int y, int z) {
                X = x;
                Y = y;
                Z = z;
            }

            public bool Equals(Cell other) { return X == other.X && Y == other.Y && Z == other.Z; }

            public override bool Equals(object obj) { return obj is Cell other && Equals(other); }

            public override int GetHashCode() { return (X * 73856093) ^ (Y * 19349663) ^ (Z * 83492791); }
        }
    }
}
