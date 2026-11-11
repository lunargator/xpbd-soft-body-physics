using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;
using UnityEngine;

namespace SoftBodyPhysics.Samples {

    public static class RockMesh {
        public static Mesh Create(int seed, float radius, string name) {
            RandomRock.Create(seed, radius, out Vec3[] points, out int[] triangles);
            var vertices = new Vector3[points.Length];
            for (int i = 0; i < points.Length; i++) vertices[i] = new Vector3(points[i].X, points[i].Y, points[i].Z);

            var mesh = new Mesh { name = name, hideFlags = HideFlags.DontSave };
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }
    }
}
