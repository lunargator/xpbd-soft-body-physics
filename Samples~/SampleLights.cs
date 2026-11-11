using UnityEngine;

namespace SoftBodyPhysics.Samples {

    public static class SampleLights {
        public static void EnableShadows() {
            Light sun = RenderSettings.sun;
            if(sun == null) {
                Light[] lights = Object.FindObjectsByType<Light>(FindObjectsInactive.Exclude);
                for (int i = 0; i < lights.Length; i++) {
                    if(lights[i] != null && lights[i].type == LightType.Directional) {
                        sun = lights[i];
                        break;
                    }
                }
            }

            if(sun == null) return;
            sun.shadows = LightShadows.Soft;
            sun.shadowStrength = 1f;
            RenderSettings.sun = sun;
        }
    }
}
