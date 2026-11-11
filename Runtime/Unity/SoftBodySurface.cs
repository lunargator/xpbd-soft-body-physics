using System.Diagnostics;
using SoftBodyPhysics.Core;
using UnityEngine;

namespace SoftBodyPhysics.Unity {

    [RequireComponent(typeof(MeshFilter))]
    [RequireComponent(typeof(MeshRenderer))]
    public sealed class SoftBodySurface : MonoBehaviour {
        readonly SoftBodyMesh mesh = new SoftBodyMesh();
        DeformableBody body;
        Material paint;
        MeshFilter filter;
        MeshRenderer renderer;

        public static float PublishedMilliseconds { get; private set; }
        public static float VertexMilliseconds { get; private set; }
        public static float NormalMilliseconds { get; private set; }
        public static float UploadMilliseconds { get; private set; }
        public static float TopologyMilliseconds { get; private set; }

        public static void ResetPublished() {
            PublishedMilliseconds = 0f;
            VertexMilliseconds = 0f;
            NormalMilliseconds = 0f;
            UploadMilliseconds = 0f;
            TopologyMilliseconds = 0f;
        }

        public Mesh SharedMesh => mesh.RuntimeMesh;
        public Mesh SourceMesh => mesh.SourceMesh;
        public int SubmeshCount => mesh.SubmeshCount;

        public int ParticleOf(int vertex) { return mesh.ParticleOf(vertex); }

        public void Bind(DeformableBody source, Color color, SoftBodyUvLayout layout, bool smoothSurface = false) {
            body = source;
            EnsurePaint(color);
            mesh.BuildBody(source, layout, transform, smoothSurface);
            Assign(new[] { paint });
            mesh.Publish(body, transform);
        }

        public void BindSource(DeformableBody source, Mesh display, int[] vertexToParticle, Material[] materials, Color fallback, bool smoothSurface = false) {
            body = source;
            EnsurePaint(fallback);
            mesh.BuildSource(source, display, vertexToParticle, transform, smoothSurface);
            Assign(MaterialsFor(mesh.SubmeshCount, materials));
            mesh.Publish(body, transform);
        }

        public void SetColor(Color color) { SolidPaint.SetColor(paint, color); }

        public void Publish() {
            if(body == null) return;
            long start = Stopwatch.GetTimestamp();
            mesh.Publish(body, transform);
            PublishedMilliseconds += (float)((Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency);
            VertexMilliseconds += mesh.VertexMilliseconds;
            NormalMilliseconds += mesh.NormalMilliseconds;
            UploadMilliseconds += mesh.UploadMilliseconds;
            TopologyMilliseconds += mesh.TopologyMilliseconds;
        }

        public bool PublishIfDue(int frame, int interval) {
            if(interval < 1) interval = 1;
            if(interval > 1 && frame % interval != 0) return false;
            Publish();
            return true;
        }

        void Assign(Material[] materials) {
            if(filter == null) filter = GetComponent<MeshFilter>();
            if(filter == null) filter = gameObject.AddComponent<MeshFilter>();
            filter.sharedMesh = mesh.RuntimeMesh;

            if(renderer == null) renderer = GetComponent<MeshRenderer>();
            if(renderer == null) renderer = gameObject.AddComponent<MeshRenderer>();
            renderer.sharedMaterials = materials;
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.On;
            renderer.receiveShadows = true;
        }

        Material[] MaterialsFor(int submeshes, Material[] materials) {
            int count = submeshes < 1 ? 1 : submeshes;
            var used = new Material[count];
            for (int i = 0; i < count; i++) {
                Material chosen = null;
                if(materials != null && i < materials.Length) chosen = materials[i];
                used[i] = chosen != null ? chosen : paint;
            }

            return used;
        }

        void EnsurePaint(Color color) {
            if(paint == null) paint = SolidPaint.Create(color);
            else SolidPaint.SetColor(paint, color);
        }

        public void Release() {
            if(filter != null) filter.sharedMesh = null;
            mesh.Release();
            DestroyOwned(paint);
            paint = null;
            filter = null;
            renderer = null;
        }

        void OnDestroy() { Release(); }

        static void DestroyOwned(UnityEngine.Object owned) {
            if(owned == null) return;
            if(Application.isPlaying) Destroy(owned);
            else DestroyImmediate(owned);
        }
    }
}
