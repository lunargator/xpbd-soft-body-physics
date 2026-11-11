namespace SoftBodyPhysics.Unity {

    public enum SoftBodyPreset {
        Custom,
        Rubber,
        Jelly,
        Slime,
        SoftFoam,
        Gelatin,
        FirmElastic
    }

    public static class SoftBodyMaterials {
        public struct Recipe {
            public float Stiffness;
            public float Damping;
            public float Friction;
            public float Restitution;
        }

        public static bool TryGet(SoftBodyPreset preset, out Recipe recipe) {
            switch (preset) {
                case SoftBodyPreset.Rubber:
                    recipe = new Recipe { Stiffness = 0.8f, Damping = 0.05f, Friction = 0.7f, Restitution = 0.85f };
                    return true;
                case SoftBodyPreset.Jelly:
                    recipe = new Recipe { Stiffness = 0.05f, Damping = 0.12f, Friction = 0.05f, Restitution = 0.08f };
                    return true;
                case SoftBodyPreset.Slime:
                    recipe = new Recipe { Stiffness = 0f, Damping = 0.7f, Friction = 0.85f, Restitution = 0f };
                    return true;
                case SoftBodyPreset.SoftFoam:
                    recipe = new Recipe { Stiffness = 0.18f, Damping = 0.82f, Friction = 0.55f, Restitution = 0f };
                    return true;
                case SoftBodyPreset.Gelatin:
                    recipe = new Recipe { Stiffness = 0.42f, Damping = 0.08f, Friction = 0.15f, Restitution = 0.32f };
                    return true;
                case SoftBodyPreset.FirmElastic:
                    recipe = new Recipe { Stiffness = 0.98f, Damping = 0.2f, Friction = 0.45f, Restitution = 0.25f };
                    return true;
                default:
                    recipe = default;
                    return false;
            }
        }
    }
}
