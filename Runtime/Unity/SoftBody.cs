using System;
using System.Collections.Generic;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using UnityEngine;

namespace SoftBodyPhysics.Unity {

    [ExecuteAlways]
    [DefaultExecutionOrder(-100)]
    public sealed partial class SoftBody : MonoBehaviour {
        public enum ShapeKind {
            Sphere,
            Cube,
            Mesh
        }

        [Header("Shape")]
        [Tooltip("Sphere and cube are built from particles. Mesh welds an assigned mesh.")]
        public ShapeKind shape = ShapeKind.Sphere;
        [Tooltip("Sphere radius in meters.")]
        public float radius = 0.5f;
        [Min(3)] public int sphereSegments = 6;
        [Min(2)] public int sphereStacks = 4;
        [Tooltip("Cube edge length in meters.")]
        public float cubeSize = 1f;
        [Min(1)] public int cubeCells = 2;
        [Tooltip("Imported or generated mesh. Vertices become particles. Materials and UVs stay on this mesh.")]
        public Mesh sourceMesh;
        [Min(0f)]
        [Tooltip("Welds vertices that fall in the same cell. Zero welds only duplicates.")]
        public float meshCellSize;

        [HideInInspector] public Vector3[] bakedLocalPositions;
        [HideInInspector] public int[] bakedTriangles;
        [HideInInspector] public int[] bakedVertexMap;
        [HideInInspector] public bool bakedClosed;
        [HideInInspector] public int bakedSourceVertexCount = -1;
        [HideInInspector] public float bakedCellSize = -1f;

        [Header("Material")]
        [Tooltip("Named material. Anything other than Custom writes stiffness, damping, friction, and restitution.")]
        public SoftBodyPreset preset = SoftBodyPreset.Custom;
        [Range(0f, 1f)]
        [Tooltip("0 is soft. 1 is firm.")]
        public float stiffness = 0.35f;
        [Tooltip("Velocity decay. Larger values settle faster.")]
        public float damping = 0.4f;
        [Tooltip("Surface friction from 0 to 1. Used when Surface Override is on, or a preset is selected.")]
        [Range(0f, 1f)]
        public float friction = 0.3f;
        [Tooltip("Surface bounce from 0 to 1. Used when Surface Override is on, or a preset is selected.")]
        [Range(0f, 1f)]
        public float restitution = 0.05f;
        [Tooltip("Writes friction and bounce onto the body. Off keeps the material default until this is on or a preset is chosen.")]
        public bool surfaceOverride;

        [Header("Simulation")]
        [Tooltip("Total mass, shared evenly across the moving particles.")]
        public float mass = 1f;
        [Tooltip("Scales the gravity on Soft Body Simulation for this body only.")]
        public float gravityMultiplier = 1f;
        [Tooltip("Turns the body off without removing the component.")]
        public bool simulate = true;
        [Tooltip("Holds the body in place. Other bodies can still hit it.")]
        public bool kinematic;
        [Tooltip("Builds a fresh body each time play starts.")]
        public bool resetOnEnable = true;
        [Tooltip("Builds the body when the component becomes active during play.")]
        public bool initializeOnEnable = true;

        [Header("Rendering")]
        [Tooltip("Tint of the generated surface. An assigned display mesh keeps its own materials.")]
        public Color color = new Color(0.93f, 0.36f, 0.28f, 1f);
        [Min(1)]
        [Tooltip("How many rendered frames pass between mesh updates. 1 updates every frame.")]
        public int renderInterval = 1;
        [Tooltip("Draws a Catmull-Rom surface through the particles. The solver still uses the coarse mesh.")]
        public bool smoothSurface;

        [Header("Debug")]
        [Tooltip("Draws a marker at each particle.")]
        public bool showParticles;
        [Tooltip("Draws each distance constraint.")]
        public bool showConstraints;
        [Tooltip("Draws the axis-aligned bounds of the particles.")]
        public bool showCollisionBounds;

        DeformableBody body;
        int[] meshVertexMap;
        SoftBodySimulation simulation;
        SoftBodySurface surface;
        Mesh displayMesh;
        int[] displayMap;
        Material[] displayMaterials;
        Vector3[] positionCache;
        float[] authoredInverseMass;
        bool registered;
        bool massReady;
        bool materialReady;
        bool motionReady;
        bool colorReady;
        float appliedMass;
        float appliedDamping;
        float appliedStiffness;
        float appliedFriction;
        float appliedRestitution;
        bool appliedSurface;
        float appliedGravityScale;
        int previewStamp = int.MinValue;
        Mesh previewMesh;
        Vector3 previewPosition;
        Quaternion previewRotation;
        Vector3[] previewPoints;
        int[] previewEdges;
        Vector3 appliedGravity;
        Color appliedColor;
        bool appliedKinematic;
        bool appliedSmooth;
        int readyForSession = -1;
        static int playSession;

        public DeformableBody Body => body;

        public SoftBodySimulation Simulation {
            get {
                if(simulation == null) simulation = SoftBodySimulation.Ensure();
                return simulation;
            }
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        static void NotePlaySession() { playSession++; }

        void OnEnable() {
            if(initializeOnEnable && Application.isPlaying && (body == null || resetOnEnable)) Initialize();
            else if(body != null && simulate) Register();
        }

        void OnDisable() { Unregister(); }

        void OnDestroy() {
            Unregister();
            if(surface != null) {
                surface.Release();
                DestroyOwned(surface.gameObject);
            }

            surface = null;
            body = null;
        }

        void Update() {
            if(Application.isPlaying && initializeOnEnable && (body == null || (resetOnEnable && readyForSession != playSession)))
                Initialize();
            else ApplySettings();
        }

        void LateUpdate() { if(surface != null && body != null) surface.PublishIfDue(Time.frameCount, renderInterval); }

        void OnValidate() {
            if(mass < 0f) mass = 0f;
            if(renderInterval < 1) renderInterval = 1;
            if(preset != SoftBodyPreset.Custom && SoftBodyMaterials.TryGet(preset, out SoftBodyMaterials.Recipe recipe)) {
                stiffness = recipe.Stiffness;
                damping = recipe.Damping;
                friction = recipe.Friction;
                restitution = recipe.Restitution;
                surfaceOverride = true;
            }

            stiffness = Mathf.Clamp01(stiffness);
            if(damping < 0f) damping = 0f;
            friction = Mathf.Clamp01(friction);
            restitution = Mathf.Clamp01(restitution);
            ApplySettings();
        }

        void OnDrawGizmos() {
            if(body != null) {
                if(showParticles) DrawParticles();
                if(showConstraints) DrawConstraints();
                if(showCollisionBounds) DrawBounds();
                return;
            }

            if(!showParticles && !showConstraints) return;
            DrawRestPreview();
        }

        public int PreviewParticleCount() {
            if(shape == ShapeKind.Cube) {
                int cells = cubeCells < 1 ? 1 : cubeCells;
                int outer = cells + 1;
                int inner = cells - 1;
                return outer * outer * outer - inner * inner * inner;
            }

            if(shape == ShapeKind.Mesh) {
                if(HasBakedMesh()) return bakedLocalPositions.Length;
                return sourceMesh != null ? sourceMesh.vertexCount : 0;
            }

            int segments = sphereSegments < 3 ? 3 : sphereSegments;
            int stacks = sphereStacks < 2 ? 2 : sphereStacks;
            return (stacks - 1) * segments + 2;
        }

        public void CollectIssues(List<string> issues) {
            if(issues == null) return;
            if(mass < 0f) issues.Add("Mass is negative.");
            else if(mass == 0f && !kinematic) issues.Add("Mass is zero, so the particles stay fixed. Turn on Kinematic to pin a body on purpose.");
            if(damping < 0f) issues.Add("Damping must be zero or greater.");
            if(shape == ShapeKind.Sphere) {
                if(radius <= 0f) issues.Add("Radius must be greater than zero.");
                if(sphereSegments < 3) issues.Add("Sphere segments must be at least 3.");
                if(sphereStacks < 2) issues.Add("Sphere stacks must be at least 2.");
            } else if(shape == ShapeKind.Cube) {
                if(cubeSize <= 0f) issues.Add("Cube size must be greater than zero.");
                if(cubeCells < 1) issues.Add("Cube cells must be at least 1.");
            } else if(sourceMesh == null) {
                issues.Add("Assign a mesh.");
            } else if(!sourceMesh.isReadable) { issues.Add("Enable Read/Write on the mesh."); }
        }

        public void Initialize() {
            Unregister();
            simulation = SoftBodySimulation.Ensure();
            body = CreateBody();
            massReady = false;
            materialReady = false;
            motionReady = false;
            colorReady = false;
            readyForSession = playSession;
            EnsureView();
            ApplySettings();
        }

        public void ApplySettings() {
            if(body == null) return;

            if(simulate) {
                if(isActiveAndEnabled && (!registered || simulation == null)) Register();
            } else if(registered) { Unregister(); }

            bool massChanged = !massReady || mass != appliedMass;
            if(massChanged) ApplyMass();
            if(!materialReady || damping != appliedDamping || stiffness != appliedStiffness || gravityMultiplier != appliedGravityScale || GravityChanged()
                || surfaceOverride != appliedSurface
                || (surfaceOverride && (friction != appliedFriction || restitution != appliedRestitution)))
                ApplyMaterial();
            if(massChanged || !motionReady || kinematic != appliedKinematic) ApplyMotionState();
            if(surface != null && (!colorReady || appliedColor != color)) {
                surface.SetColor(color);
                appliedColor = color;
                colorReady = true;
            }

            if(surface != null && smoothSurface != appliedSmooth) EnsureView();
        }

        public void AddForce(Vector3 force) {
            if(body == null || kinematic || !simulate) return;

            int count = DynamicCount();
            if(count == 0) return;

            var share = new Vec3(force.x / count, force.y / count, force.z / count);
            Particle[] particles = body.Particles;
            for (int i = 0; i < particles.Length; i++) {
                if(particles[i].InverseMass <= 0f) continue;
                Particle particle = particles[i];
                particle.Force += share;
                particles[i] = particle;
            }
        }

        public void AddImpulse(Vector3 impulse) {
            if(body == null || kinematic || !simulate) return;

            float total = 0f;
            Particle[] particles = body.Particles;
            for (int i = 0; i < particles.Length; i++) {
                float inverse = particles[i].InverseMass;
                if(inverse > 0f) total += 1f / inverse;
            }

            if(total <= 0f) return;

            var whole = new Vec3(impulse.x, impulse.y, impulse.z);
            for (int i = 0; i < particles.Length; i++) {
                float inverse = particles[i].InverseMass;
                if(inverse <= 0f) continue;
                body.ApplyImpulse(i, whole * ((1f / inverse) / total));
            }
        }

        public void ApplyPreset(SoftBodyPreset value) {
            preset = value;
            if(SoftBodyMaterials.TryGet(value, out SoftBodyMaterials.Recipe recipe)) {
                stiffness = recipe.Stiffness;
                damping = recipe.Damping;
                friction = recipe.Friction;
                restitution = recipe.Restitution;
                surfaceOverride = true;
            }

            ApplySettings();
        }

        public void SetStiffness(float value) {
            preset = SoftBodyPreset.Custom;
            stiffness = Mathf.Clamp01(value);
            ApplyStiffness();
        }

        public void SetDamping(float value) {
            preset = SoftBodyPreset.Custom;
            damping = value < 0f ? 0f : value;
            ApplySettings();
        }

        public void SetFriction(float value) {
            preset = SoftBodyPreset.Custom;
            surfaceOverride = true;
            friction = Mathf.Clamp01(value);
            ApplySettings();
        }

        public void SetRestitution(float value) {
            preset = SoftBodyPreset.Custom;
            surfaceOverride = true;
            restitution = Mathf.Clamp01(value);
            ApplySettings();
        }

        public void SetSimulate(bool enabled) {
            simulate = enabled;
            ApplySettings();
        }

        public void ResetSimulation() {
            if(body == null) {
                Initialize();
                return;
            }

            Particle[] particles = body.Particles;
            for (int i = 0; i < particles.Length; i++) {
                Particle particle = particles[i];
                particle.Position = body.RestPositions[i];
                particle.Previous = body.RestPositions[i];
                particle.Velocity = Vec3.Zero;
                particle.Force = Vec3.Zero;
                particles[i] = particle;
            }

            for (int i = 0; i < body.Constraints.Count; i++) body.Constraints[i].ResetLambda();
            ApplyMass();
            ApplyMotionState();
            if(surface != null) surface.Publish();
        }

        public void SetDisplayMesh(Mesh source, int[] vertexToParticle, Material[] materials) {
            if(source == null) throw new ArgumentNullException(nameof(source));
            displayMesh = source;
            displayMap = vertexToParticle;
            displayMaterials = materials;
            if(body != null && surface != null) surface.BindSource(body, displayMesh, displayMap, displayMaterials, color);
        }

        public void SetKinematic(bool enabled) {
            kinematic = enabled;
            ApplyMotionState();
        }

        public Vector3[] GetParticlePositions() {
            int count = body == null ? 0 : body.Particles.Length;
            if(positionCache == null || positionCache.Length != count) positionCache = new Vector3[count];
            Particle[] particles = body == null ? null : body.Particles;
            for (int i = 0; i < count; i++) {
                Vec3 position = particles[i].Position;
                positionCache[i] = new Vector3(position.X, position.Y, position.Z);
            }

            return positionCache;
        }
    }
}
