using System.Collections.Generic;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using UnityEngine;

namespace SoftBodyPhysics.Unity {

    public sealed partial class SoftBody {
        void Register() {
            if(body == null) return;
            SoftBodySimulation current = SoftBodySimulation.Ensure();
            if(registered && simulation == current) return;
            if(registered && simulation != null) simulation.Unregister(body);
            simulation = current;
            simulation.Register(body);
            registered = true;
        }

        void Unregister() {
            if(!registered) return;
            if(simulation != null) simulation.Unregister(body);
            registered = false;
        }

        DeformableBody CreateBody() {
            BodyMaterial material = BodyMaterial.Default;
            material.Compliance = ComplianceFor(stiffness);
            material.Damping = damping;
            material.Gravity = Gravity();
            if(shape == ShapeKind.Cube) return SoftBodyShapes.Cube(cubeSize, cubeCells, material, Center());
            if(shape == ShapeKind.Mesh) return CreateMeshBody(material);
            return SoftBodyShapes.Sphere(radius, sphereSegments, sphereStacks, material, Center());
        }

        void ApplyMass() {
            if(body == null) return;
            if(authoredInverseMass == null || authoredInverseMass.Length != body.Particles.Length)
                authoredInverseMass = new float[body.Particles.Length];

            int count = body.Particles.Length;
            MassDistribution.Equal(authoredInverseMass, Mathf.Max(mass, 0f));
            for (int i = 0; i < count; i++) {
                Particle particle = body.Particles[i];
                particle.InverseMass = authoredInverseMass[i];
                body.Particles[i] = particle;
            }

            appliedMass = mass;
            massReady = true;
        }

        void ApplyMotionState() {
            if(body == null || authoredInverseMass == null) return;

            Particle[] particles = body.Particles;
            for (int i = 0; i < particles.Length; i++) {
                Particle particle = particles[i];
                particle.InverseMass = kinematic ? 0f : authoredInverseMass[i];
                if(kinematic) {
                    particle.Velocity = Vec3.Zero;
                    particle.Force = Vec3.Zero;
                }

                particles[i] = particle;
            }

            appliedKinematic = kinematic;
            motionReady = true;
        }

        void ApplyMaterial() {
            if(body == null) return;

            BodyMaterial material = body.Material;
            material.Damping = damping;
            material.Gravity = Gravity();
            material.Compliance = ComplianceFor(stiffness);
            if(surfaceOverride) {
                material.Friction = friction;
                material.Restitution = restitution;
            }

            body.Material = material;
            appliedDamping = damping;
            appliedFriction = friction;
            appliedRestitution = restitution;
            appliedSurface = surfaceOverride;
            appliedGravityScale = gravityMultiplier;
            appliedGravity = simulation != null ? simulation.gravity : new Vector3(0f, -9.81f, 0f);
            materialReady = true;
            ApplyStiffness();
        }

        void ApplyStiffness() {
            if(body == null) return;

            float compliance = ComplianceFor(stiffness);
            BodyMaterial material = body.Material;
            material.Compliance = compliance;
            body.Material = material;
            appliedStiffness = stiffness;
            List<IConstraint> constraints = body.Constraints;
            for (int i = 0; i < constraints.Count; i++) {
                if(constraints[i] is DistanceConstraint distance) distance.Compliance = compliance;
                else if(constraints[i] is ShapeMatchingConstraint shapeMatch) shapeMatch.Compliance = compliance * 1.5f;
                else if(constraints[i] is VolumeConstraint volume) volume.Compliance = compliance;
            }
        }

        bool GravityChanged() {
            Vector3 gravity = simulation != null ? simulation.gravity : new Vector3(0f, -9.81f, 0f);
            return gravity != appliedGravity;
        }

        Vec3 Gravity() {
            Vector3 value = simulation != null ? simulation.gravity * gravityMultiplier : new Vector3(0f, -9.81f * gravityMultiplier, 0f);
            return new Vec3(value.x, value.y, value.z);
        }

        Vec3 Center() {
            Vector3 position = transform.position;
            return new Vec3(position.x, position.y, position.z);
        }

        int DynamicCount() {
            int count = 0;
            Particle[] particles = body.Particles;
            for (int i = 0; i < particles.Length; i++) { if(particles[i].InverseMass > 0f) count++; }

            return count;
        }

        void EnsureView() {
            if(surface == null) {
                var child = new GameObject("Soft Body Surface");
                child.hideFlags = HideFlags.DontSave;
                child.transform.SetParent(transform, false);
                surface = child.AddComponent<SoftBodySurface>();
            }

            bool smooth = smoothSurface && displayMesh == null;
            if(displayMesh != null) surface.BindSource(body, displayMesh, displayMap, displayMaterials, color, false);
            else if(shape == ShapeKind.Mesh && sourceMesh != null) {
                surface.BindSource(body, sourceMesh, meshVertexMap, MeshMaterials(), color, smooth);
                HideSourceRenderers();
            } else if(shape == ShapeKind.Cube)
                surface.Bind(body, color, SoftBodyUvLayout.Cube, smooth);
            else surface.Bind(body, color, SoftBodyUvLayout.Sphere, smooth);
            appliedColor = color;
            colorReady = true;
            appliedSmooth = smoothSurface;
        }

        void HideSourceRenderers() {
            MeshRenderer hostMesh = GetComponent<MeshRenderer>();
            if(hostMesh != null) hostMesh.enabled = false;
            SkinnedMeshRenderer skinned = GetComponent<SkinnedMeshRenderer>();
            if(skinned != null) skinned.enabled = false;
        }

        Material[] MeshMaterials() {
            MeshRenderer host = GetComponent<MeshRenderer>();
            if(host == null || host.sharedMaterials == null || host.sharedMaterials.Length == 0) return null;
            return host.sharedMaterials;
        }

        public static float ComplianceFor(float stiffnessValue) {
            float t = Mathf.Clamp01(stiffnessValue);
            const float soft = 0.02f;
            const float firm = 0.00004f;
            return Mathf.Exp(Mathf.Lerp(Mathf.Log(soft), Mathf.Log(firm), t));
        }

        static void DestroyOwned(UnityEngine.Object owned) {
            if(owned == null) return;
            if(Application.isPlaying) Destroy(owned);
            else DestroyImmediate(owned);
        }
    }
}
