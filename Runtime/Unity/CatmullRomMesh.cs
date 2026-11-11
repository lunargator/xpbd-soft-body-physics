using System;
using System.Collections.Generic;
using UnityEngine;

namespace SoftBodyPhysics.Unity {

    public sealed class CatmullRomMesh {
        public const int Divisions = 3;

        struct Knot {
            public byte Kind;
            public int A, B, C, D;
            public float T;
            public int Particle;
        }

        readonly struct WeldKey : IEquatable<WeldKey> {
            readonly byte kind;
            readonly int a, b, c;

            public WeldKey(byte kind, int a, int b, int c) {
                this.kind = kind;
                this.a = a;
                this.b = b;
                this.c = c;
            }

            public bool Equals(WeldKey other) { return kind == other.kind && a == other.a && b == other.b && c == other.c; }

            public override bool Equals(object obj) { return obj is WeldKey other && Equals(other); }

            public override int GetHashCode() { return (((kind * 397) ^ a) * 397 ^ b) * 17 ^ c; }
        }

        Knot[] knots;
        int[] triangles;
        int[] faceControls;

        public int VertexCount => knots == null ? 0 : knots.Length;
        public int[] Triangles => triangles;

        public static Vector3 Curve(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t) {
            float t2 = t * t;
            float t3 = t2 * t;
            return 0.5f * (
                2f * p1
                + (-p0 + p2) * t
                + (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2
                + (-p0 + 3f * p1 - 3f * p2 + p3) * t3);
        }

        public static CatmullRomMesh Build(int[] particleOfVertex, int[] coarseTriangles) {
            if(particleOfVertex == null || particleOfVertex.Length == 0) return null;
            if(coarseTriangles == null || coarseTriangles.Length < 3) return null;

            int coarseCount = particleOfVertex.Length;
            int maxParticle = 0;
            for (int i = 0; i < coarseCount; i++) {
                if(particleOfVertex[i] > maxParticle) maxParticle = particleOfVertex[i];
            }

            var particleVertex = new int[maxParticle + 1];
            for (int i = 0; i < particleVertex.Length; i++) particleVertex[i] = -1;
            for (int i = 0; i < coarseCount; i++) {
                int particle = particleOfVertex[i];
                if(particle >= 0 && particleVertex[particle] < 0) particleVertex[particle] = i;
            }

            var opposites = new Dictionary<ulong, int[]>(coarseTriangles.Length);
            int corners = coarseTriangles.Length - (coarseTriangles.Length % 3);
            for (int i = 0; i < corners; i += 3) {
                int va = coarseTriangles[i];
                int vb = coarseTriangles[i + 1];
                int vc = coarseTriangles[i + 2];
                if(va < 0 || vb < 0 || vc < 0 || va >= coarseCount || vb >= coarseCount || vc >= coarseCount) continue;
                Note(opposites, particleOfVertex[va], particleOfVertex[vb], particleOfVertex[vc]);
                Note(opposites, particleOfVertex[vb], particleOfVertex[vc], particleOfVertex[va]);
                Note(opposites, particleOfVertex[vc], particleOfVertex[va], particleOfVertex[vb]);
            }

            var weld = new Dictionary<WeldKey, int>(coarseCount * 4);
            var built = new List<Knot>(coarseCount * 4);
            var indices = new List<int>(corners * Divisions * Divisions);
            var controls = new List<int>();
            int n = Divisions;
            var grid = new int[(n + 1) * (n + 1)];

            for (int triangle = 0; triangle < corners; triangle += 3) {
                int va = coarseTriangles[triangle];
                int vb = coarseTriangles[triangle + 1];
                int vc = coarseTriangles[triangle + 2];
                if(va < 0 || vb < 0 || vc < 0 || va >= coarseCount || vb >= coarseCount || vc >= coarseCount) continue;

                for (int i = 0; i <= n; i++) {
                    for (int j = 0; j <= n - i; j++) {
                        grid[i * (n + 1) + j] = Place(
                            built, weld, controls, particleOfVertex, particleVertex, opposites,
                            triangle / 3, va, vb, vc, i, j);
                    }
                }

                for (int i = 0; i < n; i++) {
                    for (int j = 0; j < n - i; j++) {
                        int i00 = grid[i * (n + 1) + j];
                        int i10 = grid[(i + 1) * (n + 1) + j];
                        int i01 = grid[i * (n + 1) + j + 1];
                        indices.Add(i00);
                        indices.Add(i10);
                        indices.Add(i01);
                        if(i + j < n - 1) {
                            int i11 = grid[(i + 1) * (n + 1) + j + 1];
                            indices.Add(i10);
                            indices.Add(i11);
                            indices.Add(i01);
                        }
                    }
                }
            }

            if(built.Count == 0 || indices.Count < 3) return null;
            return new CatmullRomMesh {
                knots = built.ToArray(),
                triangles = indices.ToArray(),
                faceControls = controls.ToArray()
            };
        }

        public int[] ParticleMap() {
            var map = new int[knots.Length];
            for (int i = 0; i < knots.Length; i++) map[i] = knots[i].Particle;
            return map;
        }

        public void Evaluate(Vector3[] coarse, Vector3[] fine) {
            if(coarse == null || fine == null) return;
            int count = knots.Length < fine.Length ? knots.Length : fine.Length;
            for (int i = 0; i < count; i++) fine[i] = Position(knots[i], coarse);
        }

        public void EvaluateUv(Vector2[] coarse, Vector2[] fine) {
            if(coarse == null || fine == null) return;
            int count = knots.Length < fine.Length ? knots.Length : fine.Length;
            for (int i = 0; i < count; i++) {
                Knot knot = knots[i];
                if(knot.Kind == 0) fine[i] = SampleUv(coarse, knot.A);
                else if(knot.Kind == 1) fine[i] = Vector2.Lerp(SampleUv(coarse, knot.B), SampleUv(coarse, knot.C), knot.T);
                else fine[i] = (SampleUv(coarse, knot.B) + SampleUv(coarse, knot.C) + SampleUv(coarse, knot.D)) / 3f;
            }
        }

        Vector3 Position(Knot knot, Vector3[] coarse) {
            if(knot.Kind == 0) return Sample(coarse, knot.A);
            if(knot.Kind == 1) return EdgePoint(knot.A, knot.B, knot.C, knot.D, knot.T, coarse);

            Vector3 sum = Vector3.zero;
            int edges = 0;
            for (int edge = 0; edge < 3; edge++) {
                int offset = knot.A + edge * 4;
                if(offset < 0 || offset + 3 >= faceControls.Length) continue;
                sum += EdgePoint(faceControls[offset], faceControls[offset + 1], faceControls[offset + 2], faceControls[offset + 3], 0.5f, coarse);
                edges++;
            }

            return edges == 0 ? Vector3.zero : sum / edges;
        }

        static Vector3 EdgePoint(int n0, int v0, int v1, int n1, float t, Vector3[] coarse) {
            Vector3 a = Sample(coarse, v0);
            Vector3 b = Sample(coarse, v1);
            Vector3 p0 = n0 >= 0 ? Sample(coarse, n0) : (n1 >= 0 ? Reflect(Sample(coarse, n1), a, b) : 2f * a - b);
            Vector3 p3 = n1 >= 0 ? Sample(coarse, n1) : (n0 >= 0 ? Reflect(Sample(coarse, n0), a, b) : 2f * b - a);
            return Curve(p0, a, b, p3, t);
        }

        static Vector3 Sample(Vector3[] coarse, int index) {
            if(index < 0 || index >= coarse.Length) return Vector3.zero;
            return coarse[index];
        }

        static Vector2 SampleUv(Vector2[] coarse, int index) {
            if(index < 0 || index >= coarse.Length) return Vector2.zero;
            return coarse[index];
        }

        static Vector3 Reflect(Vector3 point, Vector3 a, Vector3 b) {
            Vector3 ab = b - a;
            float lengthSq = ab.sqrMagnitude;
            if(lengthSq < 1e-12f) return point;
            Vector3 offset = point - a;
            Vector3 along = ab * (Vector3.Dot(offset, ab) / lengthSq);
            return a + along - (offset - along);
        }

        static int Place(
            List<Knot> built,
            Dictionary<WeldKey, int> weld,
            List<int> controls,
            int[] particleOfVertex,
            int[] particleVertex,
            Dictionary<ulong, int[]> opposites,
            int triangle,
            int a,
            int b,
            int c,
            int i,
            int j) {
            int n = Divisions;
            if(i == 0 && j == 0) return Corner(built, weld, particleOfVertex, a);
            if(i == n && j == 0) return Corner(built, weld, particleOfVertex, b);
            if(i == 0 && j == n) return Corner(built, weld, particleOfVertex, c);
            if(j == 0) return Edge(built, weld, particleOfVertex, particleVertex, opposites, a, b, i);
            if(i == 0) return Edge(built, weld, particleOfVertex, particleVertex, opposites, a, c, j);
            if(i + j == n) return Edge(built, weld, particleOfVertex, particleVertex, opposites, b, c, j);
            return Face(built, weld, controls, particleOfVertex, particleVertex, opposites, triangle, a, b, c);
        }

        static int Corner(List<Knot> built, Dictionary<WeldKey, int> weld, int[] particleOfVertex, int coarse) {
            var key = new WeldKey(0, coarse, 0, 0);
            if(weld.TryGetValue(key, out int found)) return found;
            int index = built.Count;
            weld.Add(key, index);
            int particle = coarse >= 0 && coarse < particleOfVertex.Length ? particleOfVertex[coarse] : -1;
            built.Add(new Knot { Kind = 0, A = coarse, Particle = particle });
            return index;
        }

        static int Edge(
            List<Knot> built,
            Dictionary<WeldKey, int> weld,
            int[] particleOfVertex,
            int[] particleVertex,
            Dictionary<ulong, int[]> opposites,
            int from,
            int to,
            int step) {
            int lo = from < to ? from : to;
            int hi = from < to ? to : from;
            int canonical = from < to ? step : Divisions - step;
            var key = new WeldKey(1, lo, hi, canonical);
            if(weld.TryGetValue(key, out int found)) return found;

            Neighbors(particleOfVertex, particleVertex, opposites, lo, hi, out int n0, out int n1);
            int index = built.Count;
            weld.Add(key, index);
            built.Add(new Knot {
                Kind = 1,
                A = n0,
                B = lo,
                C = hi,
                D = n1,
                T = canonical / (float)Divisions,
                Particle = -1
            });
            return index;
        }

        static int Face(
            List<Knot> built,
            Dictionary<WeldKey, int> weld,
            List<int> controls,
            int[] particleOfVertex,
            int[] particleVertex,
            Dictionary<ulong, int[]> opposites,
            int triangle,
            int a,
            int b,
            int c) {
            var key = new WeldKey(2, triangle, 0, 0);
            if(weld.TryGetValue(key, out int found)) return found;

            int offset = controls.Count;
            StoreEdge(controls, particleOfVertex, particleVertex, opposites, a, b);
            StoreEdge(controls, particleOfVertex, particleVertex, opposites, b, c);
            StoreEdge(controls, particleOfVertex, particleVertex, opposites, c, a);
            int index = built.Count;
            weld.Add(key, index);
            built.Add(new Knot { Kind = 2, A = offset, B = a, C = b, D = c, Particle = -1 });
            return index;
        }

        static void StoreEdge(
            List<int> controls,
            int[] particleOfVertex,
            int[] particleVertex,
            Dictionary<ulong, int[]> opposites,
            int from,
            int to) {
            Neighbors(particleOfVertex, particleVertex, opposites, from, to, out int n0, out int n1);
            controls.Add(n0);
            controls.Add(from);
            controls.Add(to);
            controls.Add(n1);
        }

        static void Neighbors(
            int[] particleOfVertex,
            int[] particleVertex,
            Dictionary<ulong, int[]> opposites,
            int from,
            int to,
            out int n0,
            out int n1) {
            n0 = -1;
            n1 = -1;
            if(from < 0 || to < 0 || from >= particleOfVertex.Length || to >= particleOfVertex.Length) return;
            int a = particleOfVertex[from];
            int b = particleOfVertex[to];
            if(a < 0 || b < 0 || a == b) return;
            if(!opposites.TryGetValue(Pair(a, b), out int[] found)) return;
            n0 = Coarse(particleVertex, found[0], a, b);
            n1 = Coarse(particleVertex, found.Length > 1 ? found[1] : -1, a, b);
        }

        static int Coarse(int[] particleVertex, int particle, int endA, int endB) {
            if(particle < 0 || particle == endA || particle == endB || particle >= particleVertex.Length) return -1;
            return particleVertex[particle];
        }

        static void Note(Dictionary<ulong, int[]> opposites, int a, int b, int opposite) {
            if(a < 0 || b < 0 || a == b || opposite < 0 || opposite == a || opposite == b) return;
            ulong key = Pair(a, b);
            if(!opposites.TryGetValue(key, out int[] found)) {
                opposites.Add(key, new[] { opposite, -1 });
                return;
            }

            if(found[0] == opposite || found[1] == opposite) return;
            if(found[1] < 0) found[1] = opposite;
        }

        static ulong Pair(int a, int b) {
            uint lo = (uint)(a < b ? a : b);
            uint hi = (uint)(a < b ? b : a);
            return ((ulong)lo << 32) | hi;
        }
    }
}
