using UnityEngine;
using SoftBodyPhysics.Unity;

namespace SoftBodyPhysics.Samples {

    public static class SamplePaint {
        public static void ShowSolids() {
            MeshRenderer[] renderers = Object.FindObjectsByType<MeshRenderer>(FindObjectsInactive.Exclude);
            for (int i = 0; i < renderers.Length; i++) {
                MeshRenderer renderer = renderers[i];
                if(renderer == null || renderer.GetComponentInParent<SoftBody>() != null) continue;
                if(Draws(renderer.sharedMaterial)) continue;
                Material saved = Saved(renderer.gameObject.name);
                if(saved != null) renderer.sharedMaterial = saved;
            }
        }

        static bool Draws(Material material) {
            return material != null
                && material.shader != null
                && material.shader.name.StartsWith("Universal Render Pipeline");
        }

        static Material Saved(string name) {
            if(name.StartsWith("Wall")) return Resources.Load<Material>("StageWall");
            if(name.StartsWith("Launch") || name == "Buncher") return Resources.Load<Material>("StageBoard");
            if(name.StartsWith("Spin")) return Resources.Load<Material>("StageSpin");
            if(name.StartsWith("Floor") || name.Length == 0) return Resources.Load<Material>("StageFloor");
            return null;
        }
    }
}
