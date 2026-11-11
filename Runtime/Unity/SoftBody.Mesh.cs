using System;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;
using UnityEngine;

namespace SoftBodyPhysics.Unity {
    public sealed partial class SoftBody {

        public void BakeMesh() {
            if(sourceMesh == null) throw new InvalidOperationException("Assign a mesh before generating.");

            Vector3[] local = sourceMesh.vertices;
            int[] indices = sourceMesh.triangles;
            var vertices = new Vec3[local.Length];
            for (int i = 0; i < local.Length; i++) vertices[i] = new Vec3(local[i].x, local[i].y, local[i].z);

            MeshGeometry.Build build = MeshGeometry.Create(vertices, indices, meshCellSize, BodyMaterial.Default, Vec3.Zero);
            bakedLocalPositions = new Vector3[build.Body.Particles.Length];
            for (int i = 0; i < bakedLocalPositions.Length; i++) {
                Vec3 position = build.Body.Particles[i].Position;
                bakedLocalPositions[i] = new Vector3(position.X, position.Y, position.Z);
            }

            bakedTriangles = (int[])build.Body.Triangles.Clone();
            bakedVertexMap = (int[])build.VertexToParticle.Clone();
            bakedClosed = build.Body.ClosedSurface;
            bakedSourceVertexCount = local.Length;
            bakedCellSize = meshCellSize;
        }

        public bool HasBakedMesh() {
            return sourceMesh != null
                && bakedLocalPositions != null
                && bakedLocalPositions.Length >= 3
                && bakedTriangles != null
                && bakedTriangles.Length >= 3
                && bakedVertexMap != null
                && bakedVertexMap.Length == sourceMesh.vertexCount
                && bakedSourceVertexCount == sourceMesh.vertexCount
                && Mathf.Approximately(bakedCellSize, meshCellSize);
        }

        DeformableBody CreateMeshBody(BodyMaterial material) {
            if(sourceMesh == null) throw new InvalidOperationException("Assign a mesh before generating.");

            if(HasBakedMesh()) {
                var positions = new Vec3[bakedLocalPositions.Length];
                for (int i = 0; i < bakedLocalPositions.Length; i++) {
                    Vector3 world = transform.TransformPoint(bakedLocalPositions[i]);
                    positions[i] = new Vec3(world.x, world.y, world.z);
                }

                DeformableBody baked = MeshGeometry.Assemble(positions, bakedTriangles, bakedClosed, material);
                SoftBodyShapes.FinishSurface(baked, material.Compliance);
                meshVertexMap = bakedVertexMap;
                return baked;
            }

            Vector3[] local = sourceMesh.vertices;
            var vertices = new Vec3[local.Length];
            for (int i = 0; i < local.Length; i++) {
                Vector3 world = transform.TransformPoint(local[i]);
                vertices[i] = new Vec3(world.x, world.y, world.z);
            }

            return SoftBodyShapes.Mesh(vertices, sourceMesh.triangles, meshCellSize, material, Vec3.Zero, out meshVertexMap);
        }

    }
}
