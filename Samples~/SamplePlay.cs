using UnityEngine;

namespace SoftBodyPhysics.Samples {

    public static class SamplePlay {
        public static int Epoch { get; private set; }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        static void Restart() { Epoch++; }

        public static void Entered() { Epoch++; }
    }
}
