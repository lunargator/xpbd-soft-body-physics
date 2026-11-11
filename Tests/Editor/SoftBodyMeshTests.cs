using System;
using NUnit.Framework;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Unity;
using UnityEngine;

namespace SoftBodyPhysics.Tests {
    public class SoftBodyMeshTests {
        [Test]
        public void SphereAndCubeKeepUvsNormalsAndBounds() {
            SoftBody sphere = Create(SoftBody.ShapeKind.Sphere);
            SoftBody cube = Create(SoftBody.ShapeKind.Cube);
            try {
                sphere.Initialize();
                cube.Initialize();
                AssertLitSurface(sphere, 0.45f);
                AssertLitSurface(cube, 0.15f);
                Assert.GreaterOrEqual(Spread(Uvs(cube)).x, 0.5f);
                Assert.GreaterOrEqual(Spread(Uvs(sphere)).x, 0.5f);
            } finally {
                UnityEngine.Object.DestroyImmediate(sphere.gameObject);
                UnityEngine.Object.DestroyImmediate(cube.gameObject);
                DestroySimulations();
            }
        }

        [Test]
        public void ImportedMeshKeepsItsUvsAndReusesTheBake() {
            SoftBody body = Create(SoftBody.ShapeKind.Mesh);
            Mesh source = TetraMesh();
            Vector3 sourceStart = source.vertices[0];
            body.sourceMesh = source;
            body.meshCellSize = 0f;
            try {
                body.BakeMesh();
                Assert.AreEqual(4, body.bakedLocalPositions.Length);
                body.Initialize();
                SoftBodySurface surface = body.GetComponentInChildren<SoftBodySurface>();
                Assert.AreSame(source, surface.SourceMesh);
                Assert.AreEqual(source.uv.Length, surface.SharedMesh.uv.Length);
                Assert.AreEqual(sourceStart, source.vertices[0]);
                Assert.AreEqual(4, body.Body.Particles.Length);
                int particles = body.Body.Particles.Length;
                body.Initialize();
                Assert.AreEqual(particles, body.Body.Particles.Length);
                Assert.AreEqual(source.uv[1], surface.SharedMesh.uv[1]);
            } finally {
                UnityEngine.Object.DestroyImmediate(source);
                UnityEngine.Object.DestroyImmediate(body.gameObject);
                DestroySimulations();
            }
        }

        [Test]
        public void DisplayMeshKeepsItsMaterialAndTheSourceStaysPut() {
            SoftBody body = Create(SoftBody.ShapeKind.Sphere);
            Mesh source = Quad();
            Vector3 sourceStart = source.vertices[0];
            var texture = new Texture2D(4, 4);
            texture.hideFlags = HideFlags.HideAndDontSave;
            Material painted = Paint(texture);
            Material second = Paint(texture);
            try {
                body.Initialize();
                body.SetDisplayMesh(source, new[] { 0, 0, 2, 3 }, new[] { painted, second });

                SoftBodySurface surface = body.GetComponentInChildren<SoftBodySurface>();
                Mesh runtime = surface.SharedMesh;
                Assert.AreNotSame(source, runtime);
                Assert.AreEqual(2, runtime.subMeshCount);
                Assert.AreEqual(4, runtime.uv.Length);
                Assert.AreEqual(0f, runtime.uv[0].x, 1e-4f);
                Assert.AreEqual(1f, runtime.uv[3].x, 1e-4f);
                Assert.AreEqual(Color.red, runtime.colors[0]);
                Assert.AreEqual(Color.green, runtime.colors[1]);
                Assert.AreSame(texture, surface.GetComponent<MeshRenderer>().sharedMaterials[0].mainTexture);
                Assert.AreSame(second, surface.GetComponent<MeshRenderer>().sharedMaterials[1]);
                Assert.AreEqual(sourceStart.x, source.vertices[0].x, 1e-5f);

                Vector3 rested = runtime.vertices[1];
                Particle moved = body.Body.Particles[0];
                moved.Position.Y += 1f;
                body.Body.Particles[0] = moved;
                surface.Publish();

                Vector3 deformed = runtime.vertices[1];
                Assert.AreEqual(rested.x, deformed.x, 1e-4f);
                Assert.AreEqual(rested.y + 1f, deformed.y, 1e-4f);
                Assert.AreEqual(1f, runtime.uv[1].x, 1e-4f);
                Assert.AreEqual(1f, runtime.normals[1].magnitude, 1e-3f);
                Assert.IsTrue(runtime.bounds.Contains(runtime.vertices[1]));
                Assert.AreEqual(sourceStart.x, source.vertices[0].x, 1e-5f);

                body.ResetSimulation();
                Assert.AreEqual(rested.y, runtime.vertices[1].y, 1e-4f);
                Assert.AreEqual(sourceStart.y, source.vertices[0].y, 1e-5f);
            } finally {
                if(body != null) UnityEngine.Object.DestroyImmediate(body.gameObject);
                UnityEngine.Object.DestroyImmediate(source);
                UnityEngine.Object.DestroyImmediate(painted);
                UnityEngine.Object.DestroyImmediate(second);
                UnityEngine.Object.DestroyImmediate(texture);
                DestroySimulations();
            }
        }

        [Test]
        public void ABadVertexMapExplainsItself() {
            SoftBody body = Create(SoftBody.ShapeKind.Sphere);
            Mesh source = Quad();
            try {
                body.Initialize();
                ArgumentException error = Assert.Throws<ArgumentException>(() => body.SetDisplayMesh(source, new[] { 0, 1 }, null));
                StringAssert.Contains("does not match", error.Message);

                Mesh empty = new Mesh();
                try {
                    error = Assert.Throws<ArgumentException>(() => body.SetDisplayMesh(empty, null, null));
                    StringAssert.Contains("no vertices", error.Message);
                } finally { UnityEngine.Object.DestroyImmediate(empty); }
            } finally {
                UnityEngine.Object.DestroyImmediate(body.gameObject);
                UnityEngine.Object.DestroyImmediate(source);
                DestroySimulations();
            }
        }

        [Test]
        public void RenderIntervalSkipsFramesBetweenPublishes() {
            SoftBody body = Create(SoftBody.ShapeKind.Sphere);
            try {
                body.renderInterval = 2;
                body.Initialize();
                SoftBodySurface surface = body.GetComponentInChildren<SoftBodySurface>();
                Mesh mesh = surface.SharedMesh;
                int particle = surface.ParticleOf(0);
                Vector3 before = mesh.vertices[0];
                Particle moved = body.Body.Particles[particle];
                moved.Position.X += 0.4f;
                body.Body.Particles[particle] = moved;

                Assert.IsFalse(surface.PublishIfDue(1, body.renderInterval));
                Assert.AreEqual(before.x, mesh.vertices[0].x, 1e-4f);
                Assert.IsTrue(surface.PublishIfDue(2, body.renderInterval));
                Assert.AreEqual(before.x + 0.4f, mesh.vertices[0].x, 1e-4f);
            } finally {
                UnityEngine.Object.DestroyImmediate(body.gameObject);
                DestroySimulations();
            }
        }

        [Test]
        public void SmoothSurfaceKeepsTheParticlesAndAddsVertices() {
            SoftBody body = Create(SoftBody.ShapeKind.Sphere);
            body.smoothSurface = true;
            try {
                body.Initialize();
                int particles = body.Body.Particles.Length;
                SoftBodySurface surface = body.GetComponentInChildren<SoftBodySurface>();
                Mesh mesh = surface.SharedMesh;
                Assert.AreEqual(20, particles);
                Assert.Greater(mesh.vertexCount, particles);
                Assert.AreEqual(mesh.vertexCount, mesh.uv.Length);
                Assert.AreEqual(mesh.vertexCount, mesh.normals.Length);

                Particle north = body.Body.Particles[0];
                north.Position.Y += 0.4f;
                body.Body.Particles[0] = north;
                surface.Publish();

                Vector3 target = new Vector3(north.Position.X, north.Position.Y, north.Position.Z);
                Vector3[] points = mesh.vertices;
                int pole = -1;
                for (int i = 0; i < points.Length; i++) {
                    Assert.AreEqual(1f, mesh.normals[i].magnitude, 1e-2f);
                    if((points[i] - target).sqrMagnitude < 1e-6f) pole = i;
                }

                Assert.GreaterOrEqual(pole, 0);
                Assert.Greater(mesh.normals[pole].y, 0.5f);
                AssertSkinStaysInside(body, points);
                int smoothCount = mesh.vertexCount;
                body.smoothSurface = false;
                body.ApplySettings();
                Assert.Less(surface.SharedMesh.vertexCount, smoothCount);
                Assert.AreEqual(particles, body.Body.Particles.Length);
            } finally {
                UnityEngine.Object.DestroyImmediate(body.gameObject);
                DestroySimulations();
            }
        }

        static void AssertLitSurface(SoftBody body, float alignment) {
            SoftBodySurface surface = body.GetComponentInChildren<SoftBodySurface>();
            Mesh mesh = surface.SharedMesh;
            Assert.IsNull(surface.SourceMesh);
            Assert.AreEqual(HideFlags.DontSave, mesh.hideFlags);
            Assert.GreaterOrEqual(mesh.vertexCount, body.Body.Particles.Length);
            Assert.AreEqual(mesh.vertexCount, mesh.uv.Length);
            Assert.AreEqual(mesh.vertexCount, mesh.normals.Length);
            Vector3[] points = mesh.vertices;
            Vector3[] normals = mesh.normals;
            Vector2[] uvs = mesh.uv;
            Assert.Greater(points.Length, 0);
            for (int i = 0; i < points.Length; i++) {
                Assert.IsFalse(float.IsNaN(uvs[i].x) || float.IsNaN(uvs[i].y));
                Assert.AreEqual(1f, normals[i].magnitude, 1e-3f);
                if(points[i].sqrMagnitude > 0.04f) Assert.Greater(Vector3.Dot(points[i].normalized, normals[i]), alignment);
                Assert.IsTrue(mesh.bounds.Contains(points[i]));
            }
        }

        static Vector2 Spread(Vector2[] uvs) {
            Vector2 min = uvs[0];
            Vector2 max = uvs[0];
            for (int i = 1; i < uvs.Length; i++) {
                min = Vector2.Min(min, uvs[i]);
                max = Vector2.Max(max, uvs[i]);
            }

            return max - min;
        }

        static Vector2[] Uvs(SoftBody body) { return body.GetComponentInChildren<SoftBodySurface>().SharedMesh.uv; }

        static Mesh TetraMesh() {
            var mesh = new Mesh { name = "Imported Tetra" };
            mesh.vertices = new[] {
                Vector3.zero,
                Vector3.right,
                Vector3.up,
                Vector3.forward
            };
            mesh.uv = new[] { Vector2.zero, Vector2.right, Vector2.up, new Vector2(1f, 1f) };
            mesh.triangles = new[] { 0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3 };
            return mesh;
        }

        static Mesh Quad() {
            var mesh = new Mesh { name = "Display Quad" };
            mesh.vertices = new[] {
                Vector3.zero,
                new Vector3(0.25f, 0f, 0f),
                Vector3.forward,
                Vector3.right
            };
            mesh.uv = new[] { Vector2.zero, Vector2.right, Vector2.up, Vector2.one };
            mesh.colors = new[] { Color.red, Color.green, Color.blue, Color.white };
            mesh.subMeshCount = 2;
            mesh.SetTriangles(new[] { 0, 2, 1 }, 0);
            mesh.SetTriangles(new[] { 1, 2, 3 }, 1);
            return mesh;
        }

        static Material Paint(Texture texture) {
            var material = new Material(Shader.Find("Sprites/Default"));
            material.hideFlags = HideFlags.HideAndDontSave;
            material.mainTexture = texture;
            return material;
        }

        static void AssertSkinStaysInside(SoftBody body, Vector3[] points) {
            Transform space = body.transform;
            Particle[] particles = body.Body.Particles;
            var local = new Vector3[particles.Length];
            Vector3 center = Vector3.zero;
            for (int i = 0; i < particles.Length; i++) {
                Vec3 position = particles[i].Position;
                local[i] = space.InverseTransformPoint(new Vector3(position.X, position.Y, position.Z));
                center += local[i];
            }

            center /= particles.Length;
            for (int v = 0; v < points.Length; v++) {
                Vector3 offset = points[v] - center;
                float distance = offset.magnitude;
                if(distance < 1e-4f) continue;
                Vector3 direction = offset / distance;
                float support = 0f;
                for (int i = 0; i < local.Length; i++) {
                    float along = Vector3.Dot(local[i] - center, direction);
                    if(along > support) support = along;
                }

                Assert.LessOrEqual(distance, support + 0.01f);
            }
        }

        static SoftBody Create(SoftBody.ShapeKind shape) {
            var host = new GameObject(shape.ToString());
            SoftBody body = host.AddComponent<SoftBody>();
            body.shape = shape;
            body.sphereSegments = 6;
            body.sphereStacks = 4;
            body.cubeCells = 2;
            body.initializeOnEnable = false;
            return body;
        }

        static void DestroySimulations() {
            SoftBodySimulation[] worlds = UnityEngine.Object.FindObjectsByType<SoftBodySimulation>(FindObjectsInactive.Include);
            for (int i = 0; i < worlds.Length; i++) { if(worlds[i] != null) UnityEngine.Object.DestroyImmediate(worlds[i].gameObject); }
        }
    }
}
