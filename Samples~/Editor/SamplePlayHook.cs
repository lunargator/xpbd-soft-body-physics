using UnityEditor;

namespace SoftBodyPhysics.Samples {
    [InitializeOnLoad]
    static class SamplePlayHook {
        static SamplePlayHook() {
            EditorApplication.playModeStateChanged += change => {
                if(change == PlayModeStateChange.EnteredPlayMode) SamplePlay.Entered();
            };
        }
    }
}
