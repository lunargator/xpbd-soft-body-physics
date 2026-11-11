using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Core;
using UnityEngine;

namespace SoftBodyPhysics.Unity {

    [ExecuteAlways]
    [DefaultExecutionOrder(50)]
    public sealed class SoftBodySimulation : MonoBehaviour {
        [Tooltip("World gravity. Each body multiplies this by its own gravity scale.")]
        public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
        [Tooltip("Solver iterations for every body in this world.")]
        public int solverIterations = 4;
        [Tooltip("Most solver steps one rendered frame may run. This is not the frame rate.")]
        public int substeps = 1;
        [Tooltip("Seconds in one solver step. A rendered frame runs at most Max Steps of these.")]
        public float fixedDeltaTime = 1f / 60f;
        [Tooltip("Colliders under this transform are copied. Empty keeps a cached scene list until SoftBodyColliders.Invalidate.")]
        public Transform colliderRoot;

        PhysicsWorld world;
        static SoftBodySimulation active;

        public static SoftBodySimulation Active => active;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        static void PrepareForPlay() {
            if(active == null) active = null;
            SoftBodyColliders.Invalidate();
        }

        public PhysicsWorld World {
            get {
                EnsureWorld();
                return world;
            }
        }

        public bool Paused { get; private set; }
        public float LastMeshMilliseconds { get; private set; }
        public float LastVertexMilliseconds { get; private set; }
        public float LastNormalMilliseconds { get; private set; }
        public float LastUploadMilliseconds { get; private set; }
        public float LastTopologyMilliseconds { get; private set; }
        public float LastSyncMilliseconds { get; private set; }
        public float DroppedTime => world == null ? 0f : world.DroppedTime;
        public float LastDroppedTime => world == null ? 0f : world.LastDroppedTime;

        public CollisionWorld Collision => World.Collision;

        public static SoftBodySimulation Ensure() {
            if(active != null) {
                if(!active.enabled) active.enabled = true;
                active.EnsureWorld();
                return active;
            }

            SoftBodySimulation[] found = Object.FindObjectsByType<SoftBodySimulation>(FindObjectsInactive.Include);
            SoftBodySimulation disabled = null;
            for (int i = 0; i < found.Length; i++) {
                if(found[i] == null) continue;
                if(!found[i].enabled) {
                    if(disabled == null) disabled = found[i];
                    continue;
                }

                active = found[i];
                active.EnsureWorld();
                return active;
            }

            if(disabled != null) {
                disabled.enabled = true;
                active = disabled;
                active.EnsureWorld();
                return active;
            }

            var host = new GameObject("Soft Body Simulation");
            host.hideFlags = HideFlags.DontSave;
            SoftBodySimulation created = host.AddComponent<SoftBodySimulation>();
            active = created;
            created.EnsureWorld();
            return created;
        }

        public void Pause() { Paused = true; }

        public void Resume() { Paused = false; }

        public void Register(DeformableBody body) { World.AddBody(body); }

        public void Unregister(DeformableBody body) { if(world != null) world.RemoveBody(body); }

        void Awake() {
            if(active != null && active != this && active.isActiveAndEnabled) {
                enabled = false;
                return;
            }

            active = this;
            enabled = true;
            EnsureWorld();
        }

        void OnEnable() { if(active == null || active == this) active = this; }

        void OnDestroy() { if(active == this) active = null; }

        void OnValidate() {
            if(solverIterations < 1) solverIterations = 1;
            if(substeps < 1) substeps = 1;
            if(fixedDeltaTime <= 0f) fixedDeltaTime = 1f / 60f;
            if(world != null) ApplyConfiguration();
        }

        public void Play(Vector3 gravityValue, int iterations, int stepsPerFrame, float stepSeconds) {
            enabled = true;
            gravity = gravityValue;
            solverIterations = iterations;
            substeps = stepsPerFrame;
            fixedDeltaTime = stepSeconds;
            ApplyConfiguration();
        }

        public void ApplyConfiguration() {
            if(world == null) return;
            world.Settings.FixedDeltaTime = fixedDeltaTime > 0f ? fixedDeltaTime : 1f / 60f;
            world.Settings.SolverIterations = solverIterations < 1 ? 1 : solverIterations;
            world.Settings.MaxStepsPerFrame = substeps < 1 ? 1 : substeps;
        }

        void Update() {
            if(!Application.isPlaying || active != this || Paused) return;

            EnsureWorld();
            ApplyConfiguration();
            SoftBodyColliders.Sync(world.Collision, colliderRoot);
            world.Tick(Time.deltaTime);
        }

        void LateUpdate() {
            if(!Application.isPlaying || active != this) return;
            LastMeshMilliseconds = SoftBodySurface.PublishedMilliseconds;
            LastVertexMilliseconds = SoftBodySurface.VertexMilliseconds;
            LastNormalMilliseconds = SoftBodySurface.NormalMilliseconds;
            LastUploadMilliseconds = SoftBodySurface.UploadMilliseconds;
            LastTopologyMilliseconds = SoftBodySurface.TopologyMilliseconds;
            LastSyncMilliseconds = SoftBodyColliders.LastSyncMilliseconds;
            SoftBodySurface.ResetPublished();
        }

        void EnsureWorld() {
            if(world != null) return;

            world = new PhysicsWorld(new PhysicsSettings());
            ApplyConfiguration();
        }
    }
}
