using System;
using System.Collections.Generic;
using SoftBodyPhysics.Core;
using UnityEngine;
using UnityEngine.Rendering;

namespace SoftBodyPhysics.Unity {
    public enum SoftBodyUvLayout {
        Sphere,
        Cube
    }

    public sealed class SoftBodyMesh {
        Mesh runtime;
        Mesh source;
        Vector3[] vertices;
        Vector3[] restOffsets;
        Vector3[] normals;
        Vector2[] uvs;
        int[] particleOfVertex;
        int[] normalTriangles;
        int submeshCount = 1;
        CatmullRomMesh spline;
        Vector3[] coarsePositions;
        Vector3[] coarseRest;
        int[] coarseParticles;

        public Mesh RuntimeMesh => runtime;
        public Mesh SourceMesh => source;
        public int SubmeshCount => submeshCount;
        public int VertexCount => vertices == null ? 0 : vertices.Length;
        public float VertexMilliseconds { get; private set; }
        public float NormalMilliseconds { get; private set; }
        public float UploadMilliseconds { get; private set; }
        public float TopologyMilliseconds { get; private set; }
        public float BuildTopologyMilliseconds { get; private set; }

        public int ParticleOf(int vertex) {
            if(particleOfVertex == null || vertex < 0 || vertex >= particleOfVertex.Length) return -1;
            return particleOfVertex[vertex];
        }

        public void BuildBody(DeformableBody body, SoftBodyUvLayout layout, Transform space, bool smoothSurface = false) {
            if(body == null) throw new ArgumentNullException(nameof(body));
            if(body.Particles == null || body.Particles.Length == 0) throw new ArgumentException("The body has no particles.", nameof(body));

            ClearSpline();
            source = null;
            int[] triangles = body.Triangles ?? Array.Empty<int>();
            if(triangles.Length >= 3) BuildCorners(body, layout, space, triangles);
            else BuildPoints(body, space);
            if(smoothSurface) ApplySpline();

            Upload(hasColors: false, colors: null);
            Publish(body, space);
        }

        public void BuildSource(DeformableBody body, Mesh sourceMesh, int[] vertexToParticle, Transform space, bool smoothSurface = false) {
            if(body == null) throw new ArgumentNullException(nameof(body));
            if(sourceMesh == null) throw new ArgumentNullException(nameof(sourceMesh));
            if(body.Particles == null || body.Particles.Length == 0) throw new ArgumentException("The body has no particles.", nameof(body));

            ClearSpline();
            int count = sourceMesh.vertexCount;
            if(count == 0) throw new ArgumentException("Display mesh has no vertices.", nameof(sourceMesh));

            int[] map = ResolveMap(body, sourceMesh, vertexToParticle, count);
            source = sourceMesh;
            vertices = sourceMesh.vertices;
            particleOfVertex = map;
            restOffsets = new Vector3[count];
            FillOffsets(body, WorldFromLocal(space, vertices));
            normals = new Vector3[count];

            Vector2[] sourceUv = sourceMesh.uv;
            if(sourceUv != null && sourceUv.Length == count) uvs = sourceUv;
            else uvs = ProjectSphere(body, vertices, space);

            Color[] colors = sourceMesh.colors;
            bool hasColors = colors != null && colors.Length == count;

            EnsureRuntime();
            runtime.Clear();
            runtime.MarkDynamic();
            runtime.indexFormat = count > 65535 ? IndexFormat.UInt32 : IndexFormat.UInt16;
            runtime.vertices = vertices;
            runtime.uv = uvs;
            if(hasColors) runtime.colors = colors;

            Vector2[] uv2 = sourceMesh.uv2;
            if(uv2 != null && uv2.Length == count) runtime.uv2 = uv2;

            int submeshes = sourceMesh.subMeshCount < 1 ? 1 : sourceMesh.subMeshCount;
            if(smoothSurface && submeshes == 1 && !hasColors) {
                normalTriangles = sourceMesh.GetTriangles(0);
                submeshCount = 1;
                ApplySpline();
                Upload(false, null);
                Publish(body, space);
                return;
            }

            submeshCount = submeshes;
            runtime.subMeshCount = submeshes;
            var combined = new List<int>();
            long topology = System.Diagnostics.Stopwatch.GetTimestamp();
            for (int submesh = 0; submesh < submeshes; submesh++) {
                int[] tris = sourceMesh.GetTriangles(submesh);
                runtime.SetTriangles(tris, submesh, false);
                for (int i = 0; i < tris.Length; i++) combined.Add(tris[i]);
            }

            BuildTopologyMilliseconds += MillisecondsSince(ref topology);

            normalTriangles = combined.ToArray();
            Publish(body, space);
        }

        public void Publish(DeformableBody body, Transform space) {
            VertexMilliseconds = 0f;
            NormalMilliseconds = 0f;
            UploadMilliseconds = 0f;
            TopologyMilliseconds = 0f;
            if(body == null || runtime == null || vertices == null || vertices.Length == 0) return;

            long mark = System.Diagnostics.Stopwatch.GetTimestamp();
            if(spline != null && coarsePositions != null) {
                PublishSpline(body, space);
                VertexMilliseconds = MillisecondsSince(ref mark);
                WriteNormalsAndBounds();
                NormalMilliseconds = MillisecondsSince(ref mark);
                runtime.vertices = vertices;
                runtime.normals = normals;
                runtime.bounds = BoundsOf(vertices);
                UploadMilliseconds = MillisecondsSince(ref mark);
                return;
            }

            Particle[] particles = body.Particles;
            for (int i = 0; i < vertices.Length; i++) {
                int particleIndex = particleOfVertex[i];
                if(particleIndex < 0 || particleIndex >= particles.Length) continue;
                Vec3 position = particles[particleIndex].Position;
                var world = new Vector3(position.X, position.Y, position.Z) + restOffsets[i];
                vertices[i] = space != null ? space.InverseTransformPoint(world) : world;
            }

            VertexMilliseconds = MillisecondsSince(ref mark);
            WriteNormalsAndBounds();
            NormalMilliseconds = MillisecondsSince(ref mark);
            runtime.vertices = vertices;
            runtime.normals = normals;
            runtime.bounds = BoundsOf(vertices);
            UploadMilliseconds = MillisecondsSince(ref mark);
        }

        static float MillisecondsSince(ref long mark) {
            long now = System.Diagnostics.Stopwatch.GetTimestamp();
            float milliseconds = (float)((now - mark) * 1000.0 / System.Diagnostics.Stopwatch.Frequency);
            mark = now;
            return milliseconds;
        }

        public void Release() {
            if(runtime != null) {
                if(Application.isPlaying) UnityEngine.Object.Destroy(runtime);
                else UnityEngine.Object.DestroyImmediate(runtime);
            }

            runtime = null;
            source = null;
            vertices = null;
            restOffsets = null;
            normals = null;
            uvs = null;
            particleOfVertex = null;
            normalTriangles = null;
            ClearSpline();
        }

        void ClearSpline() {
            spline = null;
            coarsePositions = null;
            coarseRest = null;
            coarseParticles = null;
        }

        void ApplySpline() {
            if(vertices == null || vertices.Length == 0 || normalTriangles == null || normalTriangles.Length < 3) return;
            coarsePositions = (Vector3[])vertices.Clone();
            coarseRest = restOffsets == null ? new Vector3[vertices.Length] : (Vector3[])restOffsets.Clone();
            coarseParticles = particleOfVertex == null ? new int[vertices.Length] : (int[])particleOfVertex.Clone();
            Vector2[] coarseUv = uvs;
            spline = CatmullRomMesh.Build(coarseParticles, normalTriangles);
            if(spline == null) {
                ClearSpline();
                return;
            }

            vertices = new Vector3[spline.VertexCount];
            uvs = new Vector2[spline.VertexCount];
            particleOfVertex = spline.ParticleMap();
            restOffsets = new Vector3[spline.VertexCount];
            normals = new Vector3[spline.VertexCount];
            normalTriangles = spline.Triangles;
            submeshCount = 1;
            spline.Evaluate(coarsePositions, vertices);
            KeepInsideParticles(coarsePositions, vertices);
            spline.EvaluateUv(coarseUv, uvs);
        }

        void PublishSpline(DeformableBody body, Transform space) {
            Particle[] particles = body.Particles;
            int count = coarsePositions.Length;
            for (int i = 0; i < count; i++) {
                int particleIndex = coarseParticles[i];
                if(particleIndex < 0 || particleIndex >= particles.Length) continue;
                Vec3 position = particles[particleIndex].Position;
                var world = new Vector3(position.X, position.Y, position.Z) + coarseRest[i];
                coarsePositions[i] = space != null ? space.InverseTransformPoint(world) : world;
            }

            spline.Evaluate(coarsePositions, vertices);
            KeepInsideParticles(coarsePositions, vertices);
        }

        static void KeepInsideParticles(Vector3[] coarse, Vector3[] vertices) {
            if(coarse == null || coarse.Length == 0 || vertices == null) return;
            Vector3 center = Vector3.zero;
            for (int i = 0; i < coarse.Length; i++) center += coarse[i];
            center /= coarse.Length;
            for (int v = 0; v < vertices.Length; v++) {
                Vector3 offset = vertices[v] - center;
                float distance = offset.magnitude;
                if(distance < 1e-6f) continue;
                Vector3 direction = offset / distance;
                float support = 0f;
                for (int i = 0; i < coarse.Length; i++) {
                    float along = Vector3.Dot(coarse[i] - center, direction);
                    if(along > support) support = along;
                }

                if(distance > support + 1e-5f) vertices[v] = center + direction * support;
            }
        }

        void BuildCorners(DeformableBody body, SoftBodyUvLayout layout, Transform space, int[] triangles) {
            int corners = triangles.Length - (triangles.Length % 3);
            var cornerParticle = new int[corners];
            var cornerUv = new Vector2[corners];
            var cornerRest = new Vector3[corners];
            Vector3 center = CenterOf(body);
            RestBounds(body, out Vector3 min, out Vector3 max);

            for (int corner = 0; corner < corners; corner += 3) {
                int particleA = triangles[corner];
                int particleB = triangles[corner + 1];
                int particleC = triangles[corner + 2];
                Vector3 restA = RestVector(body, particleA);
                Vector3 restB = RestVector(body, particleB);
                Vector3 restC = RestVector(body, particleC);
                Vector3 localA = RestLocal(body, space, particleA);
                Vector3 localB = RestLocal(body, space, particleB);
                Vector3 localC = RestLocal(body, space, particleC);
                Vector2 uvA;
                Vector2 uvB;
                Vector2 uvC;
                if(layout == SoftBodyUvLayout.Cube) {
                    int axis = DominantAxis(Vector3.Cross(restB - restA, restC - restA));
                    uvA = CubeUv(restA, min, max, axis);
                    uvB = CubeUv(restB, min, max, axis);
                    uvC = CubeUv(restC, min, max, axis);
                } else {
                    uvA = SphereUv(RestVector(body, particleA), center);
                    uvB = SphereUv(RestVector(body, particleB), center);
                    uvC = SphereUv(RestVector(body, particleC), center);
                    CloseSeam(ref uvA, ref uvB, ref uvC);
                }

                cornerParticle[corner] = particleA;
                cornerParticle[corner + 1] = particleB;
                cornerParticle[corner + 2] = particleC;
                cornerUv[corner] = uvA;
                cornerUv[corner + 1] = uvB;
                cornerUv[corner + 2] = uvC;
                cornerRest[corner] = localA;
                cornerRest[corner + 1] = localB;
                cornerRest[corner + 2] = localC;
            }

            Weld(body, space, cornerParticle, cornerUv, cornerRest);
        }

        void Weld(DeformableBody body, Transform space, int[] cornerParticle, Vector2[] cornerUv, Vector3[] cornerRest) {
            var weld = new Dictionary<WeldKey, int>(cornerParticle.Length);
            var particle = new List<int>();
            var uv = new List<Vector2>();
            var rest = new List<Vector3>();
            normalTriangles = new int[cornerParticle.Length];

            for (int i = 0; i < cornerParticle.Length; i++) {
                var key = new WeldKey(cornerParticle[i], Quantize(cornerUv[i].x), Quantize(cornerUv[i].y));
                if(!weld.TryGetValue(key, out int index)) {
                    index = particle.Count;
                    weld.Add(key, index);
                    particle.Add(cornerParticle[i]);
                    uv.Add(cornerUv[i]);
                    rest.Add(cornerRest[i]);
                }

                normalTriangles[i] = index;
            }

            vertices = rest.ToArray();
            uvs = uv.ToArray();
            particleOfVertex = particle.ToArray();
            restOffsets = new Vector3[vertices.Length];
            FillOffsets(body, WorldFromLocal(space, vertices));
            normals = new Vector3[vertices.Length];
            submeshCount = 1;
        }

        void BuildPoints(DeformableBody body, Transform space) {
            int count = body.Particles.Length;
            vertices = new Vector3[count];
            uvs = new Vector2[count];
            particleOfVertex = new int[count];
            Vector3 center = CenterOf(body);
            for (int i = 0; i < count; i++) {
                particleOfVertex[i] = i;
                vertices[i] = RestLocal(body, space, i);
                uvs[i] = SphereUv(RestVector(body, i), center);
            }

            restOffsets = new Vector3[count];
            FillOffsets(body, WorldFromLocal(space, vertices));
            normals = new Vector3[count];
            normalTriangles = Array.Empty<int>();
            submeshCount = 1;
        }

        void Upload(bool hasColors, Color[] colors) {
            EnsureRuntime();
            runtime.Clear();
            runtime.MarkDynamic();
            runtime.indexFormat = vertices.Length > 65535 ? IndexFormat.UInt32 : IndexFormat.UInt16;
            runtime.subMeshCount = 1;
            runtime.vertices = vertices;
            runtime.uv = uvs;
            long topology = System.Diagnostics.Stopwatch.GetTimestamp();
            runtime.SetTriangles(normalTriangles, 0, false);
            BuildTopologyMilliseconds += MillisecondsSince(ref topology);
            if(hasColors && colors != null && colors.Length == vertices.Length) runtime.colors = colors;
        }

        void EnsureRuntime() {
            if(runtime != null) return;
            runtime = new Mesh { name = "Soft Body Runtime", hideFlags = HideFlags.DontSave };
            runtime.MarkDynamic();
        }

        void WriteNormalsAndBounds() {
            if(normals == null || normals.Length != vertices.Length) normals = new Vector3[vertices.Length];
            else { for (int i = 0; i < normals.Length; i++) normals[i] = Vector3.zero; }

            if(normalTriangles != null) {
                for (int i = 0; i + 2 < normalTriangles.Length; i += 3) {
                    int i0 = normalTriangles[i];
                    int i1 = normalTriangles[i + 1];
                    int i2 = normalTriangles[i + 2];
                    if(i0 < 0 || i1 < 0 || i2 < 0 || i0 >= vertices.Length || i1 >= vertices.Length || i2 >= vertices.Length) continue;
                    Vector3 cross = Vector3.Cross(vertices[i1] - vertices[i0], vertices[i2] - vertices[i0]);
                    normals[i0] += cross;
                    normals[i1] += cross;
                    normals[i2] += cross;
                }
            }

            for (int i = 0; i < normals.Length; i++) {
                float magnitude = normals[i].magnitude;
                normals[i] = magnitude < 1e-8f ? Vector3.up : normals[i] / magnitude;
            }
        }

        static int[] ResolveMap(DeformableBody body, Mesh sourceMesh, int[] vertexToParticle, int count) {
            if(vertexToParticle == null) {
                if(count != body.Particles.Length)
                    throw new ArgumentException(
                        "A vertex-to-particle map is required when the mesh has " + count
                        + " vertices and the body has " + body.Particles.Length + " particles.",
                        nameof(vertexToParticle));
                vertexToParticle = new int[count];
                for (int i = 0; i < count; i++) vertexToParticle[i] = i;
            }

            if(vertexToParticle.Length != count)
                throw new ArgumentException(
                    "Vertex map length " + vertexToParticle.Length + " does not match the mesh vertex count " + count + ".",
                    nameof(vertexToParticle));

            var copy = new int[count];
            int particles = body.Particles.Length;
            for (int i = 0; i < count; i++) {
                int index = vertexToParticle[i];
                if(index < 0 || index >= particles)
                    throw new ArgumentException(
                        "Vertex map index " + i + " is " + index + ", but the body has " + particles + " particles.",
                        nameof(vertexToParticle));
                copy[i] = index;
            }

            return copy;
        }

        void FillOffsets(DeformableBody body, Vector3[] restWorld) {
            for (int i = 0; i < restOffsets.Length; i++) {
                int particleIndex = particleOfVertex[i];
                Vec3 rest = RestOf(body, particleIndex);
                var particleWorld = new Vector3(rest.X, rest.Y, rest.Z);
                restOffsets[i] = restWorld[i] - particleWorld;
            }
        }

        static Vector3[] WorldFromLocal(Transform space, Vector3[] local) {
            var world = new Vector3[local.Length];
            for (int i = 0; i < local.Length; i++) world[i] = space != null ? space.TransformPoint(local[i]) : local[i];
            return world;
        }

        static Vector3 RestLocal(DeformableBody body, Transform space, int particleIndex) {
            Vec3 rest = RestOf(body, particleIndex);
            var world = new Vector3(rest.X, rest.Y, rest.Z);
            return space != null ? space.InverseTransformPoint(world) : world;
        }

        static Vec3 RestOf(DeformableBody body, int particleIndex) {
            if(body.RestPositions != null && particleIndex < body.RestPositions.Length) return body.RestPositions[particleIndex];
            return body.Particles[particleIndex].Position;
        }

        static Vector3 RestVector(DeformableBody body, int particleIndex) {
            Vec3 rest = RestOf(body, particleIndex);
            return new Vector3(rest.X, rest.Y, rest.Z);
        }

        static Vector3 CenterOf(DeformableBody body) {
            Vector3 center = Vector3.zero;
            int count = body.Particles.Length;
            for (int i = 0; i < count; i++) center += RestVector(body, i);
            return count > 0 ? center / count : Vector3.zero;
        }

        static void RestBounds(DeformableBody body, out Vector3 min, out Vector3 max) {
            min = RestVector(body, 0);
            max = min;
            for (int i = 1; i < body.Particles.Length; i++) {
                Vector3 point = RestVector(body, i);
                min = Vector3.Min(min, point);
                max = Vector3.Max(max, point);
            }
        }

        static Vector2 SphereUv(Vector3 rest, Vector3 center) {
            Vector3 direction = rest - center;
            float length = direction.magnitude;
            if(length < 1e-8f) return new Vector2(0.5f, 0.5f);
            float u = Mathf.Atan2(direction.x, direction.z) / (Mathf.PI * 2f) + 0.5f;
            float v = Mathf.Asin(Mathf.Clamp(direction.y / length, -1f, 1f)) / Mathf.PI + 0.5f;
            return new Vector2(u, v);
        }

        static void CloseSeam(ref Vector2 a, ref Vector2 b, ref Vector2 c) {
            float min = Mathf.Min(a.x, Mathf.Min(b.x, c.x));
            float max = Mathf.Max(a.x, Mathf.Max(b.x, c.x));
            if(max - min <= 0.5f) return;
            if(a.x < 0.5f) a.x += 1f;
            if(b.x < 0.5f) b.x += 1f;
            if(c.x < 0.5f) c.x += 1f;
        }

        static int DominantAxis(Vector3 normal) {
            float ax = Mathf.Abs(normal.x);
            float ay = Mathf.Abs(normal.y);
            float az = Mathf.Abs(normal.z);
            if(ax >= ay && ax >= az) return 0;
            if(ay >= az) return 1;
            return 2;
        }

        static Vector2 CubeUv(Vector3 point, Vector3 min, Vector3 max, int axis) {
            if(axis == 0) return new Vector2(Span(point.z, min.z, max.z), Span(point.y, min.y, max.y));
            if(axis == 1) return new Vector2(Span(point.x, min.x, max.x), Span(point.z, min.z, max.z));
            return new Vector2(Span(point.x, min.x, max.x), Span(point.y, min.y, max.y));
        }

        static float Span(float value, float min, float max) {
            float size = max - min;
            if(size < 1e-8f) return 0f;
            return (value - min) / size;
        }

        static int Quantize(float value) { return Mathf.RoundToInt(value * 1000f); }

        static Bounds BoundsOf(Vector3[] points) {
            Vector3 min = points[0];
            Vector3 max = points[0];
            for (int i = 1; i < points.Length; i++) {
                min = Vector3.Min(min, points[i]);
                max = Vector3.Max(max, points[i]);
            }

            return new Bounds((min + max) * 0.5f, max - min);
        }

        static Vector2[] ProjectSphere(DeformableBody body, Vector3[] localVertices, Transform space) {
            Vector3 center = CenterOf(body);
            var uv = new Vector2[localVertices.Length];
            for (int i = 0; i < localVertices.Length; i++) {
                Vector3 world = space != null ? space.TransformPoint(localVertices[i]) : localVertices[i];
                uv[i] = SphereUv(world, center);
            }

            return uv;
        }

        readonly struct WeldKey : IEquatable<WeldKey> {
            readonly int particle;
            readonly int u;
            readonly int v;

            public WeldKey(int particle, int u, int v) {
                this.particle = particle;
                this.u = u;
                this.v = v;
            }

            public bool Equals(WeldKey other) { return particle == other.particle && u == other.u && v == other.v; }

            public override bool Equals(object obj) { return obj is WeldKey other && Equals(other); }

            public override int GetHashCode() { return (particle * 397) ^ (u * 17) ^ v; }
        }
    }
}
