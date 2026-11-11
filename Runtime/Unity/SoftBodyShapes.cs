using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;
using UnityEngine;

namespace SoftBodyPhysics.Unity {

    public static class SoftBodyShapes {
        public static DeformableBody Sphere(float radius, int segments, int stacks, BodyMaterial material, Vec3 center) {
            DeformableBody body = SphereGeometry.Create(
                Mathf.Max(radius, 0.01f),
                Mathf.Max(segments, 3),
                Mathf.Max(stacks, 2),
                material,
                center);
            Finish(body, material.Compliance);
            return body;
        }

        public static DeformableBody Cube(float size, int cells, BodyMaterial material, Vec3 center) {
            DeformableBody body = CubeGeometry.Create(
                Mathf.Max(size, 0.01f),
                Mathf.Max(cells, 1),
                material,
                center);
            Finish(body, material.Compliance);
            return body;
        }

        public static DeformableBody Mesh(Vec3[] vertices, int[] triangles, float cellSize, BodyMaterial material, Vec3 offset, out int[] vertexToParticle) {
            MeshGeometry.Build build = MeshGeometry.Create(vertices, triangles, cellSize, material, offset);
            vertexToParticle = build.VertexToParticle;
            FinishSurface(build.Body, material.Compliance);
            return build.Body;
        }

        public static void FinishSurface(DeformableBody body, float compliance) {
            if(body == null) return;
            body.AddConstraint(new ShapeMatchingConstraint(body, compliance * 1.5f));
            if(body.ClosedSurface) body.AddConstraint(new VolumeConstraint(body, compliance));
        }

        public static void Finish(DeformableBody body, float compliance) {
            if(body == null) return;
            body.AddConstraint(new ShapeMatchingConstraint(body, compliance * 1.5f));
            body.AddConstraint(new VolumeConstraint(body, compliance));
        }
    }
}
