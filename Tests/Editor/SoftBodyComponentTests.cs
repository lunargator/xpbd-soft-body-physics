using System.Collections.Generic;
using NUnit.Framework;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Unity;
using UnityEditor;
using UnityEngine;

namespace SoftBodyPhysics.Tests {
    public class SoftBodyComponentTests {
        [TearDown]
        public void Cleanup() {
            SoftBody[] bodies = Object.FindObjectsByType<SoftBody>(FindObjectsInactive.Include);
            for (int i = 0; i < bodies.Length; i++) { if(bodies[i] != null) Object.DestroyImmediate(bodies[i].gameObject); }

            SoftBodySimulation[] worlds = Object.FindObjectsByType<SoftBodySimulation>(FindObjectsInactive.Include);
            for (int i = 0; i < worlds.Length; i++) { if(worlds[i] != null) Object.DestroyImmediate(worlds[i].gameObject); }
        }

        [Test]
        public void SphereAndCubeShareOneWorld() {
            Cleanup();
            SoftBody sphere = Create(SoftBody.ShapeKind.Sphere);
            SoftBody cube = Create(SoftBody.ShapeKind.Cube);
            sphere.Initialize();
            cube.Initialize();

            Assert.AreEqual(20, sphere.Body.Particles.Length);
            Assert.AreEqual(26, cube.Body.Particles.Length);
            Assert.AreSame(sphere.Simulation, cube.Simulation);
            Assert.AreEqual(2, sphere.Simulation.World.Bodies.Count);
            Assert.Greater(sphere.Body.Constraints.Count, 20);
            Assert.IsTrue(cube.Body.ClosedSurface);
        }

        [Test]
        public void DestroyingOneBodyLeavesTheOtherRegistered() {
            Cleanup();
            SoftBody sphere = Create(SoftBody.ShapeKind.Sphere);
            SoftBody cube = Create(SoftBody.ShapeKind.Cube);
            sphere.Initialize();
            cube.Initialize();
            PhysicsWorld world = sphere.Simulation.World;

            Object.DestroyImmediate(sphere.gameObject);

            Assert.AreEqual(1, world.Bodies.Count);
            Assert.AreSame(cube.Body, world.Bodies[0]);
        }

        [Test]
        public void KinematicBodyStaysAndDynamicBodyFalls() {
            Cleanup();
            SoftBody stayed = Create(SoftBody.ShapeKind.Cube);
            stayed.kinematic = true;
            stayed.Initialize();
            float pinned = stayed.Body.CenterOfMass().Y;

            SoftBody falling = Create(SoftBody.ShapeKind.Sphere);
            falling.kinematic = false;
            falling.transform.position = new Vector3(3f, 2f, 0f);
            falling.Initialize();
            float start = falling.Body.CenterOfMass().Y;

            for (int i = 0; i < 10; i++) stayed.Simulation.World.Step(0.02f);

            Assert.AreEqual(pinned, stayed.Body.CenterOfMass().Y, 1e-4f);
            Assert.Less(falling.Body.CenterOfMass().Y, start - 0.05f);
        }

        [Test]
        public void ResetAndImpulseUseThePublicApi() {
            Cleanup();
            SoftBody body = Create(SoftBody.ShapeKind.Sphere);
            body.Initialize();
            Vector3[] first = body.GetParticlePositions();
            Vector3 origin = first[0];
            body.Body.Particles[0].Position = new Vec3(origin.x + 0.4f, origin.y, origin.z);
            body.ResetSimulation();
            Vector3[] again = body.GetParticlePositions();

            Assert.AreSame(first, again);
            Assert.AreEqual(origin.x, again[0].x, 1e-5f);
            Assert.AreEqual(0f, body.Body.Particles[0].Velocity.X, 1e-6f);

            body.AddImpulse(new Vector3(1f, 0f, 0f));
            float momentum = 0f;
            for (int i = 0; i < body.Body.Particles.Length; i++) {
                float inverse = body.Body.Particles[i].InverseMass;
                if(inverse > 0f) momentum += body.Body.Particles[i].Velocity.X / inverse;
            }

            Assert.AreEqual(1f, momentum, 1e-3f);

            body.SetKinematic(true);
            float before = body.Body.Particles[0].Velocity.X;
            body.AddImpulse(new Vector3(5f, 0f, 0f));
            Assert.AreEqual(0f, body.Body.Particles[0].InverseMass, 1e-6f);
            Assert.AreEqual(before, body.Body.Particles[0].Velocity.X, 1e-6f);
        }

        [Test]
        public void SettingsStayOnTheBodyAndTheStepSizeStaysOnTheWorld() {
            Cleanup();
            SoftBody sphere = Create(SoftBody.ShapeKind.Sphere);
            sphere.Initialize();

            sphere.ApplySettings();
            Assert.AreEqual(4, sphere.Simulation.World.Settings.SolverIterations);
            Assert.AreEqual(1, sphere.Simulation.World.Settings.MaxStepsPerFrame);

            sphere.Simulation.solverIterations = 6;
            sphere.Simulation.substeps = 2;
            sphere.Simulation.ApplyConfiguration();
            sphere.ApplySettings();
            Assert.AreEqual(6, sphere.Simulation.World.Settings.SolverIterations);
            Assert.AreEqual(2, sphere.Simulation.World.Settings.MaxStepsPerFrame);

            sphere.mass = 2f;
            sphere.ApplySettings();
            Assert.AreEqual(10f, sphere.Body.Particles[0].InverseMass, 1e-3f);
            Assert.AreEqual(20, sphere.Body.Particles.Length);

            PhysicsWorld world = sphere.Simulation.World;
            sphere.simulate = false;
            sphere.ApplySettings();
            Assert.AreEqual(0, world.Bodies.Count);
            sphere.simulate = true;
            sphere.ApplySettings();
            Assert.AreSame(sphere.Body, world.Bodies[0]);
        }

        [Test]
        public void ALaterBodyLeavesTheSimulationStepAlone() {
            Cleanup();
            var defaults = new GameObject("defaults");
            SoftBodySimulation created = defaults.AddComponent<SoftBodySimulation>();
            Assert.AreEqual(1f / 60f, created.fixedDeltaTime, 1e-5f);
            Object.DestroyImmediate(defaults);

            SoftBody first = Create(SoftBody.ShapeKind.Sphere);
            first.Initialize();
            SoftBodySimulation simulation = first.Simulation;
            simulation.solverIterations = 6;
            simulation.substeps = 2;
            simulation.fixedDeltaTime = 1f / 60f;
            simulation.ApplyConfiguration();

            SoftBody second = Create(SoftBody.ShapeKind.Cube);
            second.Initialize();

            Assert.AreSame(simulation, second.Simulation);
            Assert.AreEqual(6, simulation.World.Settings.SolverIterations);
            Assert.AreEqual(2, simulation.World.Settings.MaxStepsPerFrame);
            Assert.AreEqual(1f / 60f, simulation.World.Settings.FixedDeltaTime, 1e-5f);
        }

        [Test]
        public void RuntimeSurfaceIsASeparateMeshCopy() {
            Cleanup();
            SoftBody body = Create(SoftBody.ShapeKind.Sphere);
            body.Initialize();

            Transform surface = body.transform.Find("Soft Body Surface");
            Assert.IsNotNull(surface);
            Assert.AreEqual(HideFlags.DontSave, surface.gameObject.hideFlags);
            SoftBodySurface view = surface.GetComponent<SoftBodySurface>();
            Mesh mesh = view.SharedMesh;
            Assert.IsNotNull(mesh);
            Assert.AreEqual(HideFlags.DontSave, mesh.hideFlags);
            Assert.GreaterOrEqual(mesh.vertexCount, body.Body.Particles.Length);
            Assert.AreEqual(mesh.vertexCount, mesh.uv.Length);
            Assert.AreEqual(mesh.vertexCount, mesh.normals.Length);
        }

        [Test]
        public void ReplacingTheSimulationKeepsTheBodyRegistered() {
            Cleanup();
            SoftBody body = Create(SoftBody.ShapeKind.Cube);
            body.Initialize();
            SoftBodySimulation first = body.Simulation;

            var host = new GameObject("Second Simulation");
            SoftBodySimulation second = host.AddComponent<SoftBodySimulation>();
            Assert.IsFalse(second.enabled);
            Assert.AreSame(first, SoftBodySimulation.Ensure());

            Object.DestroyImmediate(first.gameObject);
            body.ApplySettings();

            SoftBodySimulation recovered = SoftBodySimulation.Ensure();
            Assert.AreSame(second, recovered);
            Assert.IsTrue(recovered.enabled);
            Assert.AreEqual(1, recovered.World.Bodies.Count);
            Assert.AreSame(body.Body, recovered.World.Bodies[0]);
        }

        [Test]
        public void PresetsWriteDifferentSolverValues() {
            Cleanup();
            SoftBody body = Create(SoftBody.ShapeKind.Sphere);
            body.Initialize();
            var seen = new HashSet<string>();
            SoftBodyPreset[] presets = {
                SoftBodyPreset.Rubber,
                SoftBodyPreset.Jelly,
                SoftBodyPreset.Slime,
                SoftBodyPreset.SoftFoam,
                SoftBodyPreset.Gelatin,
                SoftBodyPreset.FirmElastic
            };
            try {
                float rubber = 0f;
                float slime = 0f;
                for (int i = 0; i < presets.Length; i++) {
                    body.ApplyPreset(presets[i]);
                    Assert.IsTrue(SoftBodyMaterials.TryGet(presets[i], out SoftBodyMaterials.Recipe recipe));
                    Assert.AreEqual(recipe.Stiffness, body.stiffness, 1e-5f);
                    Assert.AreEqual(recipe.Damping, body.Body.Material.Damping, 1e-5f);
                    Assert.AreEqual(recipe.Friction, body.Body.Material.Friction, 1e-5f);
                    Assert.AreEqual(recipe.Restitution, body.Body.Material.Restitution, 1e-5f);
                    Assert.AreEqual(SoftBody.ComplianceFor(recipe.Stiffness), body.Body.Material.Compliance, 1e-5f);
                    string key = recipe.Stiffness + ":" + recipe.Damping + ":" + recipe.Friction + ":" + recipe.Restitution;
                    Assert.IsTrue(seen.Add(key));
                    if(presets[i] == SoftBodyPreset.Rubber) rubber = body.Body.Material.Compliance;
                    if(presets[i] == SoftBodyPreset.Slime) slime = body.Body.Material.Compliance;
                }

                Assert.Less(rubber, slime);
                float kept = body.stiffness;
                body.ApplyPreset(SoftBodyPreset.Custom);
                Assert.AreEqual(kept, body.stiffness, 1e-5f);
            } finally { Cleanup(); }
        }

        [Test]
        public void SlimeFlattensMoreThanAFirmBody() {
            Cleanup();
            SoftBody slime = Create(SoftBody.ShapeKind.Sphere);
            slime.transform.position = new Vector3(-2f, 1.4f, 0f);
            slime.radius = 0.3f;
            slime.ApplyPreset(SoftBodyPreset.Slime);
            slime.Initialize();
            SoftBody firm = Create(SoftBody.ShapeKind.Sphere);
            firm.transform.position = new Vector3(2f, 1.4f, 0f);
            firm.radius = 0.3f;
            firm.ApplyPreset(SoftBodyPreset.FirmElastic);
            firm.Initialize();
            slime.Simulation.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            float slimeHeight = 1f;
            float firmHeight = 1f;
            try {
                for (int i = 0; i < 120; i++) {
                    slime.Simulation.World.Step(0.02f);
                    slimeHeight = Mathf.Min(slimeHeight, Height(slime));
                    firmHeight = Mathf.Min(firmHeight, Height(firm));
                }

                Assert.Less(slimeHeight, firmHeight - 0.05f);
            } finally { Cleanup(); }
        }

        static float Height(SoftBody body) {
            Particle[] particles = body.Body.Particles;
            float min = particles[0].Position.Y;
            float max = min;
            for (int i = 1; i < particles.Length; i++) {
                float y = particles[i].Position.Y;
                if(y < min) min = y;
                if(y > max) max = y;
            }

            return max - min;
        }

        [Test]
        public void InvalidShapesAreReportedAndPreviewMatchesTheBuild() {
            Cleanup();
            SoftBody sphere = Create(SoftBody.ShapeKind.Sphere);
            var issues = new List<string>();
            try {
                sphere.CollectIssues(issues);
                Assert.AreEqual(0, issues.Count);
                Assert.AreEqual(20, sphere.PreviewParticleCount());
                sphere.Initialize();
                Assert.AreEqual(sphere.Body.Particles.Length, sphere.PreviewParticleCount());

                SoftBody cube = Create(SoftBody.ShapeKind.Cube);
                Assert.AreEqual(26, cube.PreviewParticleCount());
                cube.Initialize();
                Assert.AreEqual(cube.Body.Particles.Length, cube.PreviewParticleCount());

                sphere.radius = 0f;
                issues.Clear();
                sphere.CollectIssues(issues);
                Assert.IsTrue(HasIssue(issues, "Radius"));

                sphere.shape = SoftBody.ShapeKind.Mesh;
                sphere.sourceMesh = null;
                issues.Clear();
                sphere.CollectIssues(issues);
                Assert.IsTrue(HasIssue(issues, "Assign a mesh"));

                sphere.shape = SoftBody.ShapeKind.Sphere;
                sphere.radius = 0.5f;
                sphere.mass = 0f;
                sphere.kinematic = false;
                issues.Clear();
                sphere.CollectIssues(issues);
                Assert.IsTrue(HasIssue(issues, "Mass is zero"));
                sphere.kinematic = true;
                issues.Clear();
                sphere.CollectIssues(issues);
                Assert.IsFalse(HasIssue(issues, "Mass is zero"));
            } finally { Cleanup(); }
        }

        [Test]
        public void DampingFrictionAndSimulateFollowTheRuntimeApi() {
            Cleanup();
            SoftBody body = Create(SoftBody.ShapeKind.Sphere);
            body.Initialize();
            try {
                body.SetDamping(0.55f);
                Assert.AreEqual(0.55f, body.Body.Material.Damping, 1e-5f);
                body.SetFriction(0.2f);
                body.SetRestitution(0.4f);
                Assert.AreEqual(0.2f, body.Body.Material.Friction, 1e-5f);
                Assert.AreEqual(0.4f, body.Body.Material.Restitution, 1e-5f);
                Assert.AreEqual(SoftBodyPreset.Custom, body.preset);
                Vector3[] positions = body.GetParticlePositions();
                Assert.AreEqual(body.Body.Particles.Length, positions.Length);

                PhysicsWorld world = body.Simulation.World;
                body.SetSimulate(false);
                Assert.AreEqual(0, world.Bodies.Count);
                body.SetSimulate(true);
                Assert.AreSame(body.Body, world.Bodies[0]);
            } finally { Cleanup(); }
        }

        [Test]
        public void CopiedBakeBuildsTheSameBody() {
            Cleanup();
            Mesh mesh = Tetra();
            SoftBody baked = Create(SoftBody.ShapeKind.Mesh);
            baked.sourceMesh = mesh;
            baked.meshCellSize = 0f;
            SoftBody copy = Create(SoftBody.ShapeKind.Mesh);
            try {
                baked.BakeMesh();
                copy.sourceMesh = mesh;
                copy.meshCellSize = baked.meshCellSize;
                copy.bakedLocalPositions = (Vector3[])baked.bakedLocalPositions.Clone();
                copy.bakedTriangles = (int[])baked.bakedTriangles.Clone();
                copy.bakedVertexMap = (int[])baked.bakedVertexMap.Clone();
                copy.bakedClosed = baked.bakedClosed;
                copy.bakedSourceVertexCount = baked.bakedSourceVertexCount;
                copy.bakedCellSize = baked.bakedCellSize;
                Assert.IsTrue(copy.HasBakedMesh());
                copy.Initialize();
                Assert.AreEqual(baked.bakedLocalPositions.Length, copy.Body.Particles.Length);
                Assert.AreEqual(baked.bakedLocalPositions.Length, copy.PreviewParticleCount());
            } finally {
                Object.DestroyImmediate(mesh);
                Cleanup();
            }
        }

        static bool HasIssue(List<string> issues, string text) {
            for (int i = 0; i < issues.Count; i++) { if(issues[i].Contains(text)) return true; }

            return false;
        }

        [Test]
        public void SamplePrefabsBuildASphereCubeAndSimulation() {
            Cleanup();
            ClearHiddenSimulations();
            GameObject simulationObject = null;
            GameObject sphereObject = null;
            GameObject cubeObject = null;
            try {
                simulationObject = InstantiatePrefab("SoftBodySimulation");
                sphereObject = InstantiatePrefab("SoftBodySphere");
                cubeObject = InstantiatePrefab("SoftBodyCube");
                SoftBodySimulation simulation = simulationObject.GetComponent<SoftBodySimulation>();
                Assert.AreEqual(4, simulation.solverIterations);
                Assert.AreEqual(1, simulation.substeps);
                Assert.AreEqual(1f / 60f, simulation.fixedDeltaTime, 1e-6f);
                Assert.AreEqual(4, simulation.World.Settings.SolverIterations);
                Assert.AreEqual(1f / 60f, simulation.World.Settings.FixedDeltaTime, 1e-5f);

                SoftBody sphere = sphereObject.GetComponent<SoftBody>();
                Assert.AreEqual(SoftBody.ShapeKind.Sphere, sphere.shape);
                Assert.AreEqual(6, sphere.sphereSegments);
                Assert.AreEqual(4, sphere.sphereStacks);
                sphere.Initialize();
                Assert.AreEqual(20, sphere.Body.Particles.Length);
                Assert.AreSame(simulation, sphere.Simulation);

                SoftBody cube = cubeObject.GetComponent<SoftBody>();
                Assert.AreEqual(SoftBody.ShapeKind.Cube, cube.shape);
                Assert.AreEqual(SoftBodyPreset.Jelly, cube.preset);
                Assert.IsTrue(cube.surfaceOverride);
                cube.Initialize();
                Assert.AreEqual(26, cube.Body.Particles.Length);
                float start = cube.Body.CenterOfMass().Y;
                simulation.World.Step(simulation.fixedDeltaTime);
                Assert.Less(cube.Body.CenterOfMass().Y, start);
            } finally {
                if(sphereObject != null) Object.DestroyImmediate(sphereObject);
                if(cubeObject != null) Object.DestroyImmediate(cubeObject);
                if(simulationObject != null) Object.DestroyImmediate(simulationObject);
                Cleanup();
            }
        }

        static void ClearHiddenSimulations() {
            SoftBodySimulation[] worlds = Resources.FindObjectsOfTypeAll<SoftBodySimulation>();
            for (int i = 0; i < worlds.Length; i++) {
                if(worlds[i] == null || EditorUtility.IsPersistent(worlds[i])) continue;
                Object.DestroyImmediate(worlds[i].gameObject);
            }
        }

        static GameObject InstantiatePrefab(string name) {
            string[] found = AssetDatabase.FindAssets(name + " t:Prefab");
            Assert.Greater(found.Length, 0, name);
            string path = AssetDatabase.GUIDToAssetPath(found[0]);
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(path);
            Assert.IsNotNull(prefab, path);
            return (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        }

        static Mesh Tetra() {
            var mesh = new Mesh {
                vertices = new[] {
                    new Vector3(0f, 0f, 0f),
                    new Vector3(1f, 0f, 0f),
                    new Vector3(0f, 1f, 0f),
                    new Vector3(0f, 0f, 1f)
                },
                triangles = new[] { 0, 1, 2, 0, 2, 3, 0, 3, 1, 1, 3, 2 }
            };
            return mesh;
        }

        static SoftBody Create(SoftBody.ShapeKind shape) {
            var host = new GameObject(shape.ToString());
            SoftBody body = host.AddComponent<SoftBody>();
            body.shape = shape;
            body.sphereSegments = 6;
            body.sphereStacks = 4;
            body.cubeCells = 2;
            body.initializeOnEnable = false;
            return body;
        }
    }
}
