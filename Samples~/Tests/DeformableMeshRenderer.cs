using SoftBodyPhysics.Core;
using UnityEngine;
using SoftBodyPhysics.Unity;

namespace SoftBodyPhysics.Samples {

    public sealed class DeformableMeshRenderer : MonoBehaviour {
        public const int SplineDivisions = 3;

        Mesh mesh;
        Vector3[] vertices;
        Vector3[] normals;
        int[] indices;
        Vector3[] locals;
        DeformableBody body;
        int segments;
        int stacks;
        bool spline;

        public Mesh SharedMesh => mesh;
        public DeformableBody Body => body;

        public void Bind(DeformableBody source, Material material) { Bind(source, material, 0, 0); }

        public void Bind(DeformableBody source, Material material, int sphereSegments, int sphereStacks) {
            body = source;
            segments = sphereSegments;
            stacks = sphereStacks;
            int particles = source == null ? 0 : source.Particles.Length;
            spline = sphereSegments >= 3 && sphereStacks >= 2 && particles == (sphereStacks - 1) * sphereSegments + 2;
            if(mesh == null) {
                mesh = new Mesh { name = name + " Surface" };
                mesh.MarkDynamic();
                mesh.hideFlags = HideFlags.DontSave;
            }

            mesh.Clear();
            mesh.MarkDynamic();
            if(spline) {
                int columns = sphereSegments * SplineDivisions;
                int rows = sphereStacks * SplineDivisions;
                vertices = new Vector3[2 + (rows - 1) * columns];
                locals = new Vector3[particles];
                indices = SplineTriangles(columns, rows);
                mesh.vertices = vertices;
                mesh.triangles = indices;
            } else {
                vertices = new Vector3[particles];
                locals = null;
                indices = source == null || source.Triangles == null ? System.Array.Empty<int>() : source.Triangles;
                mesh.vertices = vertices;
                mesh.triangles = indices;
            }

            MeshFilter filter = GetComponent<MeshFilter>();
            if(filter == null) filter = gameObject.AddComponent<MeshFilter>();
            filter.sharedMesh = mesh;

            MeshRenderer renderer = GetComponent<MeshRenderer>();
            if(renderer == null) renderer = gameObject.AddComponent<MeshRenderer>();
            renderer.sharedMaterial = material;
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.On;
            renderer.receiveShadows = true;
            Publish();
        }

        public void Publish() {
            if(body == null || mesh == null || vertices == null) return;

            if(spline) PublishSpline();
            else PublishParticles();

            ApplySurface();
        }

        void ApplySurface() {
            if(normals == null || normals.Length != vertices.Length) normals = new Vector3[vertices.Length];
            else { for (int i = 0; i < normals.Length; i++) normals[i] = Vector3.zero; }

            if(indices != null) {
                for (int i = 0; i + 2 < indices.Length; i += 3) {
                    int i0 = indices[i];
                    int i1 = indices[i + 1];
                    int i2 = indices[i + 2];
                    if(i0 < 0 || i1 < 0 || i2 < 0 || i0 >= vertices.Length || i1 >= vertices.Length || i2 >= vertices.Length) continue;
                    Vector3 cross = Vector3.Cross(vertices[i1] - vertices[i0], vertices[i2] - vertices[i0]);
                    normals[i0] += cross;
                    normals[i1] += cross;
                    normals[i2] += cross;
                }
            }

            Vector3 min = vertices[0];
            Vector3 max = vertices[0];
            for (int i = 0; i < vertices.Length; i++) {
                float mag = normals[i].magnitude;
                normals[i] = mag < 1e-8f ? Vector3.up : normals[i] / mag;
                Vector3 point = vertices[i];
                min = Vector3.Min(min, point);
                max = Vector3.Max(max, point);
            }

            mesh.vertices = vertices;
            mesh.normals = normals;
            mesh.bounds = new Bounds((min + max) * 0.5f, max - min);
        }

        void PublishParticles() {
            Particle[] particles = body.Particles;
            int count = particles.Length < vertices.Length ? particles.Length : vertices.Length;
            for (int i = 0; i < count; i++) {
                Vec3 position = particles[i].Position;
                vertices[i] = transform.InverseTransformPoint(new Vector3(position.X, position.Y, position.Z));
            }
        }

        void PublishSpline() {
            Particle[] particles = body.Particles;
            int count = particles.Length < locals.Length ? particles.Length : locals.Length;
            for (int i = 0; i < count; i++) {
                Vec3 position = particles[i].Position;
                locals[i] = transform.InverseTransformPoint(new Vector3(position.X, position.Y, position.Z));
            }

            int columns = segments * SplineDivisions;
            int rows = stacks * SplineDivisions;
            vertices[0] = Surface(0, 0f, 0, 0f);
            int cursor = 1;
            for (int row = 1; row < rows; row++) {
                int stack = row / SplineDivisions;
                float tv = (row % SplineDivisions) / (float)SplineDivisions;
                for (int column = 0; column < columns; column++) {
                    int segment = column / SplineDivisions;
                    float tu = (column % SplineDivisions) / (float)SplineDivisions;
                    vertices[cursor++] = Surface(segment, tu, stack, tv);
                }
            }

            vertices[vertices.Length - 1] = Surface(0, 0f, stacks, 0f);
        }

        Vector3 Surface(int segment, float tu, int stack, float tv) {
            return CatmullRom(
                AlongRing(stack - 1, segment, tu),
                AlongRing(stack, segment, tu),
                AlongRing(stack + 1, segment, tu),
                AlongRing(stack + 2, segment, tu),
                tv);
        }

        Vector3 AlongRing(int stack, int segment, float t) {
            return CatmullRom(
                Sample(stack, segment - 1),
                Sample(stack, segment),
                Sample(stack, segment + 1),
                Sample(stack, segment + 2),
                t);
        }

        Vector3 Sample(int stack, int segment) {
            if(stack < 0) return MirrorAcrossPole(Point(0, 0), Point(1, segment));
            if(stack > stacks) return MirrorAcrossPole(Point(stacks, 0), Point(stacks - 1, segment));
            return Point(stack, segment);
        }

        Vector3 MirrorAcrossPole(Vector3 pole, Vector3 ring) {
            Vector3 center = (Point(0, 0) + Point(stacks, 0)) * 0.5f;
            Vector3 axis = pole - center;
            float lengthSq = axis.sqrMagnitude;
            if(lengthSq < 1e-8f) return ring;

            Vector3 offset = ring - center;
            Vector3 parallel = axis * (Vector3.Dot(offset, axis) / lengthSq);
            return center + parallel - (offset - parallel);
        }

        Vector3 Point(int stack, int segment) {
            if(stack <= 0) return locals[0];
            if(stack >= stacks) return locals[locals.Length - 1];

            int wrapped = segment % segments;
            if(wrapped < 0) wrapped += segments;
            return locals[1 + (stack - 1) * segments + wrapped];
        }

        static int[] SplineTriangles(int columns, int rows) {
            int middle = rows - 2;
            if(middle < 0) middle = 0;
            var triangles = new int[columns * (2 + middle * 2) * 3];
            int cursor = 0;
            int south = 1 + (rows - 1) * columns;
            for (int column = 0; column < columns; column++) {
                int next = (column + 1) % columns;
                triangles[cursor++] = 0;
                triangles[cursor++] = 1 + column;
                triangles[cursor++] = 1 + next;
                int last = 1 + (rows - 2) * columns;
                triangles[cursor++] = south;
                triangles[cursor++] = last + next;
                triangles[cursor++] = last + column;
            }

            for (int row = 1; row < rows - 1; row++) {
                int current = 1 + (row - 1) * columns;
                int below = current + columns;
                for (int column = 0; column < columns; column++) {
                    int next = (column + 1) % columns;
                    int i00 = current + column;
                    int i10 = current + next;
                    int i01 = below + column;
                    int i11 = below + next;
                    triangles[cursor++] = i00;
                    triangles[cursor++] = i10;
                    triangles[cursor++] = i01;
                    triangles[cursor++] = i10;
                    triangles[cursor++] = i11;
                    triangles[cursor++] = i01;
                }
            }

            return triangles;
        }

        static Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t) {
            float t2 = t * t;
            float t3 = t2 * t;
            return 0.5f * (
                2f * p1
                + (-p0 + p2) * t
                + (2f * p0 - 5f * p1 + 4f * p2 - p3) * t2
                + (-p0 + 3f * p1 - 3f * p2 + p3) * t3);
        }

        public void Release() {
            if(mesh == null) return;
            if(Application.isPlaying) Destroy(mesh);
            else DestroyImmediate(mesh);
            mesh = null;
        }

        void OnDestroy() { Release(); }
    }
}
