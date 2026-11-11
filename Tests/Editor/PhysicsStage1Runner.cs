using SoftBodyPhysics.Samples;
using System;
using System.IO;
using System.Text;
using NUnit.Framework;
using SoftBodyPhysics.Unity;
using UnityEditor;
using UnityEditor.Callbacks;

namespace SoftBodyPhysics.Tests {
    public static class PhysicsStage1Runner {
        const string RequestName = "PhysicsStage10.request";
        const string ResultName = "PhysicsStage10.txt";

        [MenuItem("Tools/Soft Body Physics/Run Solver Tests")]
        public static void RunFromMenu() { Run(); }

        [DidReloadScripts]
        static void RunAfterCompile() { if(File.Exists(RequestPath())) Run(); }

        public static void Run() {
            string request = RequestPath();
            if(File.Exists(request)) File.Delete(request);

            var report = new StringBuilder();
            int failures = 0;
            var tests = new PhysicsWorldTests();
            failures += Invoke(report, "remove body", tests.RemovingABodyDropsItFromTheWorld);
            failures += Invoke(report, "free fall", tests.FreeParticleFallsAndFixedParticleStays);
            failures += Invoke(report, "frame split", tests.SameSimulatedTimeIsIndependentOfFrameSplits);
            failures += Invoke(report, "dropped time", tests.AFrameLongerThanTheCapDropsTheExtraTime);
            failures += Invoke(report, "damping", tests.DampingReducesThePredictedSpeed);
            var distance = new DistanceConstraintTests();
            failures += Invoke(report, "rest length", distance.StiffPairReachesRestLengthInOneIteration);
            failures += Invoke(report, "fixed end", distance.FixedParticleStaysWhileTheFreeEndMoves);
            failures += Invoke(report, "lambda reset", distance.ComplianceCorrectsAgainAfterLambdaReset);
            failures += Invoke(report, "chain", distance.SharedChainTightensAsIterationsIncrease);
            failures += Invoke(report, "hang", distance.StiffConstraintHoldsAParticleAgainstGravity);
            failures += Invoke(report, "coincident", distance.CoincidentParticlesDoNotProduceNaNs);
            var sphere = new SphereGeometryTests();
            failures += Invoke(report, "closed sphere", sphere.SphereIsAClosedMeshOfParticles);
            failures += Invoke(report, "sphere surface", sphere.VerticesLieOnTheSphereAndMatchTheirEdges);
            failures += Invoke(report, "sphere rest", sphere.RestSphereDoesNotDrift);
            failures += Invoke(report, "sphere arguments", sphere.RejectsADegenerateSphere);
            var cube = new CubeGeometryTests();
            failures += Invoke(report, "cube closed", cube.CubeIsAClosedOutwardSurface);
            failures += Invoke(report, "cube volume", cube.FinerCubeKeepsItsVolume);
            failures += Invoke(report, "cube arguments", cube.RejectsADegenerateCube);
            var imported = new MeshGeometryTests();
            failures += Invoke(report, "mesh tetra", imported.TetrahedronIsClosedAndKeepsItsVolume);
            failures += Invoke(report, "mesh pyramid", imported.PyramidIsADifferentClosedShape);
            failures += Invoke(report, "mesh open", imported.OpenQuadDoesNotReceiveAVolumeConstraint);
            failures += Invoke(report, "mesh weld", imported.DuplicateVerticesWeldAndACoarseCellMergesNeighbors);
            failures += Invoke(report, "mesh error", imported.BadMeshesExplainThemselves);
            var component = new SoftBodyComponentTests();
            failures += Invoke(report, "soft body world", component.SphereAndCubeShareOneWorld);
            failures += Invoke(report, "soft body remove", component.DestroyingOneBodyLeavesTheOtherRegistered);
            failures += Invoke(report, "soft body kinematic", component.KinematicBodyStaysAndDynamicBodyFalls);
            failures += Invoke(report, "soft body api", component.ResetAndImpulseUseThePublicApi);
            failures += Invoke(report, "soft body settings", component.SettingsStayOnTheBodyAndTheStepSizeStaysOnTheWorld);
            failures += Invoke(report, "step owner", component.ALaterBodyLeavesTheSimulationStepAlone);
            failures += Invoke(report, "soft body surface", component.RuntimeSurfaceIsASeparateMeshCopy);
            failures += Invoke(report, "soft body recover", component.ReplacingTheSimulationKeepsTheBodyRegistered);
            failures += Invoke(report, "presets", component.PresetsWriteDifferentSolverValues);
            failures += Invoke(report, "sample prefabs", component.SamplePrefabsBuildASphereCubeAndSimulation);
            failures += Invoke(report, "preset squash", component.SlimeFlattensMoreThanAFirmBody);
            failures += Invoke(report, "invalid shape", component.InvalidShapesAreReportedAndPreviewMatchesTheBuild);
            failures += Invoke(report, "runtime controls", component.DampingFrictionAndSimulateFollowTheRuntimeApi);
            failures += Invoke(report, "bake keeps", component.CopiedBakeBuildsTheSameBody);
            var performance = new PerformanceTests();
            failures += Invoke(report, "benchmark", performance.BenchmarkRecordsStepCosts);
            failures += Invoke(report, "bowl pace", performance.BowlStaysInsideAFrame);
            failures += Invoke(report, "course pace", performance.CourseStaysInsideAFrame);
            failures += Invoke(report, "fast impact", performance.FastImpactStaysFinite);
            failures += Invoke(report, "large deformation", performance.LargeDeformationDoesNotExplode);
            failures += Invoke(report, "extreme stiffness", performance.ExtremeStiffnessAndTimestepsStayFinite);
            failures += Invoke(report, "odd colliders", performance.LowAndHighCountsAndOddCollidersStayFinite);
            failures += Invoke(report, "particle pace", performance.SolverTimeGrowsWithParticleCount);
            failures += Invoke(report, "constraint pace", performance.SolverTimeGrowsWithConstraintCount);
            failures += Invoke(report, "iteration pace", performance.SolverTimeGrowsWithIterations);
            var meshes = new SoftBodyMeshTests();
            failures += Invoke(report, "mesh uv", meshes.SphereAndCubeKeepUvsNormalsAndBounds);
            failures += Invoke(report, "mesh import", meshes.ImportedMeshKeepsItsUvsAndReusesTheBake);
            failures += Invoke(report, "mesh source", meshes.DisplayMeshKeepsItsMaterialAndTheSourceStaysPut);
            failures += Invoke(report, "mesh map", meshes.ABadVertexMapExplainsItself);
            failures += Invoke(report, "mesh interval", meshes.RenderIntervalSkipsFramesBetweenPublishes);
            failures += Invoke(report, "smooth surface", meshes.SmoothSurfaceKeepsTheParticlesAndAddsVertices);
            var integration = new SoftBodyIntegrationTests();
            failures += Invoke(report, "drop mesh", integration.DroppedBodiesLandAndTheMeshStaysOnTheParticles);
            failures += Invoke(report, "body forces", integration.ForcesStiffnessAndGravityStayOnEachBody);
            failures += Invoke(report, "display rebuild", integration.DisplayMeshCanBeAssignedBeforeInitializeAndSurvivesARebuild);
            failures += Invoke(report, "bad particle", integration.AParticleIndexOutsideTheBodyIsRejected);
            var bowl = new SoftBodyBowlTests();
            failures += Invoke(report, "box holds", bowl.SpheresRestInTheBoxUntilShoved);
            failures += Invoke(report, "box rocks", bowl.RandomRocksStayClosedInsideTheBox);
            failures += Invoke(report, "box settles", bowl.SettledBodiesStayPut);
            var soft = new ShapeAndVolumeTests();
            failures += Invoke(report, "rigid motion", soft.RigidMotionIsLeftAlone);
            failures += Invoke(report, "shape restore", soft.StiffShapeMatchingUndoesASquash);
            failures += Invoke(report, "shape compliance", soft.ComplianceLeavesSomeOfTheSquash);
            failures += Invoke(report, "shape pin", soft.ShapeMatchingDoesNotMoveAFixedParticle);
            failures += Invoke(report, "octahedron volume", soft.OctahedronVolumeMatchesTheClosedSurface);
            failures += Invoke(report, "volume restore", soft.VolumeConstraintRestoresAUniformShrink);
            failures += Invoke(report, "volume pin", soft.VolumeConstraintDoesNotMoveAFixedParticle);
            failures += Invoke(report, "volume gradient", soft.VolumeGradientMatchesAFiniteDifference);
            failures += Invoke(report, "volume invert", soft.AnInvertedBodyIsNotPulledBackByVolume);
            failures += Invoke(report, "volume degenerate", soft.ADegenerateFaceDoesNotMakeTheVolumeGradientNaN);
            failures += Invoke(report, "shape goal", soft.OneRigidFitPerStepStaysTheShippedPath);
            var shell = new ShellLimitTests();
            failures += Invoke(report, "shell spread", shell.ACrushedBodyCanSpreadPastTheOriginalShell);
            var response = new ContactResponseTests();
            failures += Invoke(report, "equal momentum", response.EqualMassesKeepTheirMomentum);
            failures += Invoke(report, "light heavy", response.ALightBodyMovesFartherThanAHeavyBody);
            failures += Invoke(report, "fixed host", response.AFixedBodyDoesNotTakeTheCorrection);
            failures += Invoke(report, "opposite velocity", response.OppositeVelocitiesKeepTheCombinedMomentum);
            failures += Invoke(report, "two points", response.TwoPointsShareOneTriangle);
            failures += Invoke(report, "resting contact", response.RestingContactDoesNotGainHeight);
            var masses = new MassDistributionTests();
            failures += Invoke(report, "area mass", masses.AreaWeightsKeepTheTotalAndAvoidZeroMass);
            failures += Invoke(report, "degenerate mass", masses.ADegenerateMeshFallsBackToAnEvenSplit);
            failures += Invoke(report, "zero mass", masses.ZeroTotalMassFixesEveryParticle);
            var plane = new PlaneCollisionTests();
            failures += Invoke(report, "plane push", plane.ParticleBelowAPlaneIsPushedOutAlongTheNormal);
            failures += Invoke(report, "plane miss", plane.ParticleAboveThePlaneIsLeftAlone);
            failures += Invoke(report, "restitution", plane.RestitutionScalesTheOutgoingNormalSpeed);
            failures += Invoke(report, "friction", plane.FrictionReducesTheSlideAlongThePlane);
            failures += Invoke(report, "plane rest", plane.ParticleUnderGravityRestsOnThePlane);
            failures += Invoke(report, "sphere land", plane.SoftSphereLandsWithoutSinking);
            var box = new BoxCollisionTests();
            failures += Invoke(report, "box face", box.ParticleInsideABoxLeavesThroughTheNearestFace);
            failures += Invoke(report, "box miss", box.ParticleOutsideTheBoxIsLeftAlone);
            failures += Invoke(report, "box hit", box.HittingTheTopUsesRestitutionAndFriction);
            failures += Invoke(report, "box rest", box.ParticleUnderGravityRestsOnTheBox);
            failures += Invoke(report, "sphere on box", box.SoftSphereLandsOnABoxWithoutEnteringIt);
            var solids = new ColliderPrimitiveTests();
            failures += Invoke(report, "solid sphere", solids.SpherePushesAnInteriorPointOutAndLeavesTheOutsideAlone);
            failures += Invoke(report, "sphere tunnel", solids.FastParticleStopsOnTheNearSideOfASphere);
            failures += Invoke(report, "solid capsule", solids.CapsulePushesAnInteriorPointOutRadially);
            failures += Invoke(report, "oriented box", solids.OrientedBoxMatchesAnAxisAlignedExit);
            failures += Invoke(report, "floor hold", solids.SoftSphereStaysAboveAWideStaticFloor);
            failures += Invoke(report, "rotated box", solids.RotatedBoxPushesThroughTheNearestFace);
            failures += Invoke(report, "mesh floor", solids.TriangleFloorStopsAPointFromBelowAndFromAFastFall);
            failures += Invoke(report, "moving box", solids.MovingBoxCarriesAPointUp);
            failures += Invoke(report, "sliding box", solids.SlidingBoxCarriesAPointSideways);
            failures += Invoke(report, "open mesh", solids.OpenTriangleCatchesAFallFromEitherSide);
            failures += Invoke(report, "open slope", solids.OpenSlopeDoesNotPullADistantPointThroughItself);
            failures += Invoke(report, "rise under slope", solids.OpenSlopeBlocksARiseFromUnderneath);
            failures += Invoke(report, "sphere under slope", solids.OpenSlopeBlocksASphereRisingFromUnderneath);
            failures += Invoke(report, "box rise", solids.RisingBoxDoesNotPullAPointUpThroughItself);
            failures += Invoke(report, "box descend", solids.DescendingBoxDoesNotPullARiderThroughTheBottom);
            failures += Invoke(report, "box spin", solids.SpinningBoxDragsAPointAcrossItsTop);
            failures += Invoke(report, "box spin body", solids.SpinningBoxTurnsAParticleThatIsNotTouchingIt);
            failures += Invoke(report, "thin wall", solids.ThinWallPushesBackThroughTheEnteredFace);
            failures += Invoke(report, "closed face", solids.ClosedFaceDoesNotPullADistantPoint);
            var unityColliders = new UnityColliderTests();
            failures += Invoke(report, "unity box", unityColliders.SoftSphereLandsOnAUnityBox);
            failures += Invoke(report, "unity sphere", unityColliders.SoftSphereLandsOnASphereCollider);
            failures += Invoke(report, "unity capsule", unityColliders.SoftSphereLandsOnACapsuleCollider);
            failures += Invoke(report, "unity mesh", unityColliders.SoftSphereLandsOnAStaticMesh);
            failures += Invoke(report, "unity platform", unityColliders.KinematicPlatformLiftsTheSoftSphere);
            failures += Invoke(report, "unity dynamic", unityColliders.DynamicRigidbodiesAreNotCopied);
            var profile = new PublishProfileTests();
            failures += Invoke(report, "mesh publish", profile.PublishSplitsVerticesNormalsAndUpload);
            failures += Invoke(report, "collider cache", profile.AStillMeshColliderIsReadOnce);
            var bodies = new BodyCollisionTests();
            failures += Invoke(report, "surface push", bodies.AVertexInsideATriangleIsPushedToTheSurface);
            failures += Invoke(report, "fixed triangle", bodies.AFixedTriangleDoesNotMove);
            failures += Invoke(report, "no early contact", bodies.OutsideAndTheSameBodyDoNotCollide);
            failures += Invoke(report, "sphere gap", bodies.SeparatedSpheresDoNotCollideUntilTheyTouch);
            failures += Invoke(report, "sphere cross", bodies.OverlappingSpheresSeparateWhenTheSurfacesCross);
            failures += Invoke(report, "no tunnel", bodies.AFastSphereStopsOnTheSurfaceInsteadOfPassingThrough);
            failures += Invoke(report, "wall dent", bodies.ASphereDentsWhenItHitsAWall);
            failures += Invoke(report, "no slide", bodies.ASphereCannotSlideThroughAnother);
            failures += Invoke(report, "air shape", bodies.FallingSpheresKeepTheirShapeUntilTheSurfacesMeet);
            failures += Invoke(report, "pile holds", bodies.ASmallPileSquishesWithoutCollapsingToAPoint);
            failures += Invoke(report, "triangle bounce", bodies.AVertexBouncesOffAFixedTriangle);
            failures += Invoke(report, "missed face", bodies.AVertexThatMissesAFaceDoesNotDragTheFaceWithIt);
            failures += Invoke(report, "sphere drop", bodies.DroppedSpheresStayInContactInsteadOfFlinging);
            var rendering = new RenderingDebugTests();
            failures += Invoke(report, "mesh follow", rendering.MeshVerticesAndNormalsFollowTheParticles);
            failures += Invoke(report, "spline surface", rendering.SplineSurfacePassesThroughTheParticles);
            failures += Invoke(report, "wireframe", rendering.WireframeDrawsEachEdgeOnce);
            failures += Invoke(report, "distance lines", rendering.DistanceLinesFollowTheConstraints);
            failures += Invoke(report, "center mark", rendering.CenterOfMassMarkSitsOnTheBody);
            failures += Invoke(report, "contact normal", rendering.ContactNormalIsDrawnFromTheParticle);
            var driven = new ImpulseAttachmentTests();
            failures += Invoke(report, "impulse", driven.ImpulseChangesOnlyThatParticlesVelocity);
            failures += Invoke(report, "fixed impulse", driven.FixedParticleIgnoresAnImpulse);
            failures += Invoke(report, "impulse travel", driven.LocalizedImpulseTravelsWithTheBodyWhileTheShapeReturns);
            failures += Invoke(report, "stiff pin", driven.StiffAttachmentMovesTheParticleToTheTarget);
            failures += Invoke(report, "soft pin", driven.ComplianceLeavesTheParticlePartwayToTheTarget);
            failures += Invoke(report, "fixed pin", driven.FixedParticleStaysWhenItIsAttached);
            failures += Invoke(report, "pin drag", driven.MovingTheTargetPullsOneParticleAndTheOtherFollows);
            var validation = new ValidationTests();
            failures += Invoke(report, "tiny mass", validation.TinyAndHugeMassesStayFinite);
            failures += Invoke(report, "large force", validation.ALargeForceStaysFinite);
            failures += Invoke(report, "extreme speed", validation.AnExtremeSpeedStaysFinite);
            failures += Invoke(report, "degenerate triangle", validation.ADegenerateTriangleDoesNotProduceNaN);
            failures += Invoke(report, "zero length", validation.ZeroRestLengthPullsWithoutNaN);
            failures += Invoke(report, "flat volume", validation.ANearlyFlatVolumeStaysFinite);
            failures += Invoke(report, "inverted run", validation.AnInvertedSurfaceStaysFiniteForManySteps);
            failures += Invoke(report, "collapsed shape", validation.ACollapsedShapeMatchStaysFinite);
            failures += Invoke(report, "deep penetration", validation.ADeepPenetrationLeavesTheBox);
            failures += Invoke(report, "extreme surface", validation.ExtremeFrictionAndRestitutionStayFinite);
            failures += Invoke(report, "mass ratio", validation.AOneToFourMassRatioKeepsMomentum);
            failures += Invoke(report, "soak", validation.ARestingSphereStaysFiniteForALongRun);
            failures += Invoke(report, "self collision", validation.ABodyDoesNotCollideWithItsOwnSurface);
            failures += Invoke(report, "two worlds", validation.TwoWorldsDoNotShareAStep);
            failures += Invoke(report, "rebuild body", validation.DestroyingAndRebuildingKeepsTheSurvivors);
            failures += Invoke(report, "scene disable", validation.DisablingABodyRemovesItUntilEnabled);
            failures += Invoke(report, "second simulation", validation.ASecondSimulationTurnsItselfOff);
            failures += Invoke(report, "play gate", validation.EnablingOutsidePlayDoesNotBuildTheBody);
            failures += Invoke(report, "copied settings", validation.ASerializedCopyKeepsTheBodySettings);
            failures += Invoke(report, "benchmark names", validation.BenchmarkScenariosStayNamed);
            var acceptance = new AcceptanceTests();
            failures += Invoke(report, "fall rest", acceptance.FallingSphereDeformsThenRests);
            failures += Invoke(report, "attachment drag", acceptance.MovingAnAttachmentDeformsTheBody);
            report.AppendLine(failures == 0 ? "PASS" : "FAIL " + failures);
            WriteResult(report.ToString());
            if(failures == 0) UnityEngine.Debug.Log(report.ToString());
            else UnityEngine.Debug.LogError(report.ToString());
        }

        static int Invoke(StringBuilder report, string name, Action test) {
            try {
                test();
                report.AppendLine("OK   " + name);
                return 0;
            } catch (Exception exception) {
                report.AppendLine("FAIL " + name);
                report.AppendLine(exception.ToString());
                return 1;
            }
        }

        static string RequestPath() { return Path.Combine(Path.GetDirectoryName(UnityEngine.Application.dataPath) ?? ".", "Temp", RequestName); }

        static void WriteResult(string text) {
            string directory = Path.Combine(Path.GetDirectoryName(UnityEngine.Application.dataPath) ?? ".", "Temp");
            Directory.CreateDirectory(directory);
            File.WriteAllText(Path.Combine(directory, ResultName), text);
        }
    }
}
