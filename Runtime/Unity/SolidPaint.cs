using UnityEngine;

namespace SoftBodyPhysics.Unity {

    public static class SolidPaint {
        public static Material Create(Color color, float smoothness = 0.2f) {
            Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
            if(shader == null) shader = Shader.Find("Universal Render Pipeline/Lit");
            if(shader == null) shader = Shader.Find("Sprites/Default");
            var material = new Material(shader);
            if(material.HasProperty("_Smoothness")) material.SetFloat("_Smoothness", smoothness);
            if(material.HasProperty("_Metallic")) material.SetFloat("_Metallic", 0f);
            if(material.HasProperty("_ReceiveShadows")) material.SetFloat("_ReceiveShadows", 1f);
            if(material.HasProperty("_Cull")) material.SetFloat("_Cull", 0f);
            SetColor(material, color);
            material.hideFlags = HideFlags.DontSave;
            return material;
        }

        public static void SetColor(Material material, Color color) {
            if(material == null) return;
            if(material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", color);
            material.color = color;
        }
    }
}
