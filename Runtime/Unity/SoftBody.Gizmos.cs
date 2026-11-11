using System;
using System.Collections.Generic;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;
using UnityEngine;

namespace SoftBodyPhysics.Unity {
    public sealed partial class SoftBody {
        void DrawRestPreview() {
            if(previewStamp != RestStamp() || previewMesh != sourceMesh || previewPosition != transform.position || previewRotation != transform.rotation || previewPoints == null)
                BuildRestPreview();
            if(previewPoints == null) return;

            if(showParticles) {
                Gizmos.color = color;
                for (int i = 0; i < previewPoints.Length; i++) Gizmos.DrawSphere(previewPoints[i], 0.03f);
            }

            if(showConstraints && previewEdges != null) {
                Gizmos.color = new Color(color.r, color.g, color.b, 0.85f);
                for (int i = 0; i + 1 < previewEdges.Length; i += 2)
                    Gizmos.DrawLine(previewPoints[previewEdges[i]], previewPoints[previewEdges[i + 1]]);
            }
        }

        int RestStamp() {
            unchecked {
                int stamp = (int)shape;
                stamp = stamp * 17 + sphereSegments;
                stamp = stamp * 17 + sphereStacks;
                stamp = stamp * 17 + cubeCells;
                stamp = stamp * 17 + (HasBakedMesh() ? bakedLocalPositions.Length : 0);
                stamp = stamp * 17 + Mathf.RoundToInt(radius * 1000f);
                stamp = stamp * 17 + Mathf.RoundToInt(cubeSize * 1000f);
                stamp = stamp * 17 + Mathf.RoundToInt(meshCellSize * 1000f);
                return stamp;
            }
        }

        void BuildRestPreview() {
            previewStamp = RestStamp();
            previewMesh = sourceMesh;
            previewPosition = transform.position;
            previewRotation = transform.rotation;
            previewPoints = null;
            previewEdges = null;
            if(shape == ShapeKind.Mesh) {
                BuildMeshPreview();
                return;
            }

            if(shape == ShapeKind.Sphere && radius <= 0f) return;
            if(shape == ShapeKind.Cube && cubeSize <= 0f) return;

            BodyMaterial material = BodyMaterial.Default;
            DeformableBody preview = shape == ShapeKind.Cube
                ? SoftBodyShapes.Cube(cubeSize, cubeCells < 1 ? 1 : cubeCells, material, Center())
                : SoftBodyShapes.Sphere(radius, sphereSegments < 3 ? 3 : sphereSegments, sphereStacks < 2 ? 2 : sphereStacks, material, Center());
            CopyPreview(preview);
        }

        void BuildMeshPreview() {
            if(HasBakedMesh()) {
                previewPoints = new Vector3[bakedLocalPositions.Length];
                for (int i = 0; i < bakedLocalPositions.Length; i++) previewPoints[i] = transform.TransformPoint(bakedLocalPositions[i]);
                previewEdges = EdgesFromTriangles(bakedTriangles, previewPoints.Length);
                return;
            }

            if(sourceMesh == null || !sourceMesh.isReadable) return;
            Vector3[] local = sourceMesh.vertices;
            previewPoints = new Vector3[local.Length];
            for (int i = 0; i < local.Length; i++) previewPoints[i] = transform.TransformPoint(local[i]);
            previewEdges = EdgesFromTriangles(sourceMesh.triangles, previewPoints.Length);
        }

        void CopyPreview(DeformableBody preview) {
            Particle[] particles = preview.Particles;
            previewPoints = new Vector3[particles.Length];
            for (int i = 0; i < particles.Length; i++) {
                Vec3 position = particles[i].Position;
                previewPoints[i] = new Vector3(position.X, position.Y, position.Z);
            }

            int edges = 0;
            List<IConstraint> constraints = preview.Constraints;
            for (int i = 0; i < constraints.Count; i++) { if(constraints[i] is DistanceConstraint) edges++; }

            previewEdges = new int[edges * 2];
            int cursor = 0;
            for (int i = 0; i < constraints.Count; i++) {
                if(!(constraints[i] is DistanceConstraint distance)) continue;
                previewEdges[cursor++] = distance.IndexA;
                previewEdges[cursor++] = distance.IndexB;
            }
        }

        static int[] EdgesFromTriangles(int[] triangles, int count) {
            if(triangles == null) return null;
            var edges = new List<int>(triangles.Length * 2);
            for (int i = 0; i + 2 < triangles.Length; i += 3) {
                AddEdge(edges, triangles[i], triangles[i + 1], count);
                AddEdge(edges, triangles[i + 1], triangles[i + 2], count);
                AddEdge(edges, triangles[i + 2], triangles[i], count);
            }

            return edges.ToArray();
        }

        static void AddEdge(List<int> edges, int a, int b, int count) {
            if(a < 0 || b < 0 || a >= count || b >= count || a == b) return;
            edges.Add(a);
            edges.Add(b);
        }

        void DrawParticles() {
            Gizmos.color = color;
            Particle[] particles = body.Particles;
            for (int i = 0; i < particles.Length; i++) {
                Vec3 position = particles[i].Position;
                Gizmos.DrawSphere(new Vector3(position.X, position.Y, position.Z), 0.03f);
            }
        }

        void DrawConstraints() {
            Gizmos.color = new Color(color.r, color.g, color.b, 0.85f);
            List<IConstraint> constraints = body.Constraints;
            Particle[] particles = body.Particles;
            for (int i = 0; i < constraints.Count; i++) {
                if(!(constraints[i] is DistanceConstraint distance)) continue;
                if(distance.IndexA < 0 || distance.IndexB < 0 || distance.IndexA >= particles.Length || distance.IndexB >= particles.Length)
                    continue;
                Vec3 a = particles[distance.IndexA].Position;
                Vec3 b = particles[distance.IndexB].Position;
                Gizmos.DrawLine(new Vector3(a.X, a.Y, a.Z), new Vector3(b.X, b.Y, b.Z));
            }
        }

        void DrawBounds() {
            Particle[] particles = body.Particles;
            if(particles.Length == 0) return;

            Vec3 first = particles[0].Position;
            Vector3 min = new Vector3(first.X, first.Y, first.Z);
            Vector3 max = min;
            for (int i = 1; i < particles.Length; i++) {
                Vec3 position = particles[i].Position;
                var point = new Vector3(position.X, position.Y, position.Z);
                min = Vector3.Min(min, point);
                max = Vector3.Max(max, point);
            }

            Gizmos.color = new Color(1f, 0.85f, 0.2f, 0.9f);
            Gizmos.DrawWireCube((min + max) * 0.5f, max - min);
        }

    }
}
