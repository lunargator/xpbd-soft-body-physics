using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;
using SoftBodyPhysics.Unity;
using UnityEngine;

namespace SoftBodyPhysics.Tests {
    public class PublishProfileTests {
        readonly List<Object> owned = new List<Object>();

        [Test]
        public void PublishSplitsVerticesNormalsAndUpload() {
            DeformableBody body = SphereGeometry.Create(0.5f, 6, 4, BodyMaterial.Default, Vec3.Zero);
            var mesh = new SoftBodyMesh();
            try {
                mesh.BuildBody(body, SoftBodyUvLayout.Sphere, null);
                float buildTopology = mesh.BuildTopologyMilliseconds;
                mesh.Publish(body, null);
                string path = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "Temp", "MeshPublish.txt");
                File.WriteAllText(path,
                    "vertices " + mesh.VertexMilliseconds.ToString("0.000")
                    + " normals " + mesh.NormalMilliseconds.ToString("0.000")
                    + " upload " + mesh.UploadMilliseconds.ToString("0.000")
                    + " publishTopology " + mesh.TopologyMilliseconds.ToString("0.000")
                    + " buildTopology " + buildTopology.ToString("0.000")
                    + " count " + mesh.VertexCount);
                Assert.Greater(mesh.VertexCount, 0);
                Assert.AreEqual(0f, mesh.TopologyMilliseconds, 1e-6f);
                Assert.Greater(mesh.VertexMilliseconds + mesh.NormalMilliseconds + mesh.UploadMilliseconds, 0f);
                Assert.IsFalse(float.IsNaN(buildTopology));
            } finally { mesh.Release(); }
        }

        [Test]
        public void AStillMeshColliderIsReadOnce() {
            SoftBodyColliders.Invalidate();
            var host = new GameObject("Cached Floor");
            owned.Add(host);
            var mesh = new Mesh { name = "Cached Quad" };
            owned.Add(mesh);
            mesh.vertices = new[] {
                new Vector3(-1f, 0f, -1f),
                new Vector3(1f, 0f, -1f),
                new Vector3(1f, 0f, 1f),
                new Vector3(-1f, 0f, 1f)
            };
            mesh.triangles = new[] { 0, 2, 1, 0, 3, 2 };
            MeshCollider collider = host.AddComponent<MeshCollider>();
            collider.sharedMesh = mesh;
            var world = new PhysicsWorld(new PhysicsSettings());
            try {
                SoftBodyColliders.Sync(world.Collision, host.transform);
                Assert.AreEqual(1, SoftBodyColliders.LastSceneScans);
                Assert.AreEqual(1, SoftBodyColliders.LastMeshReads);
                int triangles = world.Collision.TriangleCount;
                Assert.AreEqual(2, triangles);

                host.transform.position += new Vector3(0f, 0.2f, 0f);
                SoftBodyColliders.Sync(world.Collision, host.transform);
                Assert.AreEqual(0, SoftBodyColliders.LastSceneScans);
                Assert.AreEqual(0, SoftBodyColliders.LastMeshReads);
                Assert.AreEqual(triangles, world.Collision.TriangleCount);
            } finally {
                SoftBodyColliders.Invalidate();
                for (int i = 0; i < owned.Count; i++) { if(owned[i] != null) Object.DestroyImmediate(owned[i]); }
            }
        }
    }
}
