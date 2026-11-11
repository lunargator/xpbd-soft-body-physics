using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;
using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Collision {
    public sealed partial class CollisionWorld {
        const int PlaneKind = 0;
        const int BoxKind = 1;
        const int TriangleKind = 2;
        const int EdgeKind = 3;
        const int FaceKind = 4;
        const int SphereKind = 5;
        const int CapsuleKind = 6;
        const int OrientedKind = 7;
        const int MeshKind = 8;
        const float PairQuerySlack = 0.004f;

        readonly List<CollisionPlane> planes = new List<CollisionPlane>();
        readonly List<CollisionBox> boxes = new List<CollisionBox>();
        readonly List<CollisionSphere> spheres = new List<CollisionSphere>();
        readonly List<CollisionCapsule> capsules = new List<CollisionCapsule>();
        readonly List<CollisionOrientedBox> orientedBoxes = new List<CollisionOrientedBox>();
        readonly List<CollisionTriangle> triangles = new List<CollisionTriangle>();
        readonly List<Ride> rides = new List<Ride>();
        readonly HashSet<long> rideKeys = new HashSet<long>();
        bool triangleBoundsValid;
        bool movingMeshes;
        float triangleMinX;
        float triangleMinY;
        float triangleMinZ;
        float triangleMaxX;
        float triangleMaxY;
        float triangleMaxZ;
        Contact[] contacts = new Contact[64];
        float[] speedLimit = new float[8];
        int[] impactPick = new int[8];
        bool[] impactShared = System.Array.Empty<bool>();
        int[] bounceKeep = new int[64];
        ContactSolver.FacePlane[] facePlanes = new ContactSolver.FacePlane[256];
        int[] faceOrigin = new int[8];
        int[] faceLength = new int[8];
        BodyBounds[] bodyBounds = new BodyBounds[8];
        Vec3[] bodyCenter = new Vec3[8];
        float[] bodyReach = new float[8];
        bool[] bodyRound = new bool[8];
        bool[] bodyConvex = new bool[8];
        const int BinAxis = 6;
        const int BinCells = 216;
        int[] faceBinCount = System.Array.Empty<int>();
        int[] faceBinStart = System.Array.Empty<int>();
        int[] faceBinList = System.Array.Empty<int>();
        float[] binOriginX = System.Array.Empty<float>();
        float[] binOriginY = System.Array.Empty<float>();
        float[] binOriginZ = System.Array.Empty<float>();
        float[] binCell = System.Array.Empty<float>();
        bool[] faceWide = new bool[8];
        int[] neighborStart = System.Array.Empty<int>();
        int[] neighborCount = System.Array.Empty<int>();
        int[] faceNeighbor = System.Array.Empty<int>();
        int[] faceSeeds = System.Array.Empty<int>();
        int[] vertexCover = System.Array.Empty<int>();
        int[][] coverRows = System.Array.Empty<int[]>();
        int[] builtFaceCount = System.Array.Empty<int>();
        SurfaceBatch[] batches = System.Array.Empty<SurfaceBatch>();
        Vec3[] motionSample = System.Array.Empty<Vec3>();
        float[] bodyMove = System.Array.Empty<float>();
        float[] pairDrift = System.Array.Empty<float>();
        int[] pairAwake = System.Array.Empty<int>();
        int[] lastInside = System.Array.Empty<int>();
        int[] pairAsleep = System.Array.Empty<int>();
        int[] spanSlot = System.Array.Empty<int>();
        int[] spanSeen = System.Array.Empty<int>();
        int motionParticles = -1;
        bool motionReady;
        int pairWidth = -1;
        int spanStamp;
        int spanMask = -1;
        bool freshSurfaces = true;
        bool forceSurfaces;

        public int SurfaceQueries;
        public int SurfaceSkips;
        public float SurfaceMotion;
        public int BinHits;
        public int BinMisses;
        public int BinUnsure;
        public int FullQueries;
        public float PrepareMilliseconds;
        public float SurfaceMilliseconds;
        public float SolveMilliseconds;
        public float BroadMilliseconds;
        public float HoldMilliseconds;
        public int OrientedTests;
        public int SphereTests;
        public int TriangleTests;
        int seedBodyCount = -1;
        int seedParticleCount = -1;
        bool[] supported = new bool[8];
        int[] insideCount = new int[64];
        int insideStride;
        int contactCount;
        int collectId;
        int lookupStamp;
        int particleSpan;
        int trackedBodies;
        int planeStride;
        int boxStride;
        int surfaceStride;
        int sphereStride;
        int capsuleStride;
        int orientedStride;
        int[] particleOffset = new int[8];
        int[] planeSlot = System.Array.Empty<int>();
        int[] planeSeen = System.Array.Empty<int>();
        int[] boxSlot = System.Array.Empty<int>();
        int[] boxSeen = System.Array.Empty<int>();
        int[] surfaceSlot = System.Array.Empty<int>();
        int[] surfaceSeen = System.Array.Empty<int>();
        int[] sphereSlot = System.Array.Empty<int>();
        int[] sphereSeen = System.Array.Empty<int>();
        int[] capsuleSlot = System.Array.Empty<int>();
        int[] capsuleSeen = System.Array.Empty<int>();
        int[] orientedSlot = System.Array.Empty<int>();
        int[] orientedSeen = System.Array.Empty<int>();
        int[] meshSlot = System.Array.Empty<int>();
        int[] meshSeen = System.Array.Empty<int>();

        public float RestitutionSleepSpeed = 0.5f;

        public float ShellScale = 1.05f;
        public int PlaneCount => planes.Count;
        public int BoxCount => boxes.Count;
        public int SphereCount => spheres.Count;
        public int CapsuleCount => capsules.Count;
        public int OrientedBoxCount => orientedBoxes.Count;
        public int TriangleCount => triangles.Count;
        public int ContactCount => contactCount;

        public Contact GetContact(int index) {
            if(index < 0 || index >= contactCount) return default;
            return contacts[index];
        }

        public void AddPlane(CollisionPlane plane) {
            float length = plane.Normal.Length;
            if(length < 1e-8f) return;
            if(Math.Abs(length - 1f) > 1e-4f) plane.Normal = plane.Normal * (1f / length);
            planes.Add(plane);
        }

        public void ClearPlanes() { planes.Clear(); }

        public void AddBox(CollisionBox box) { boxes.Add(CollisionBox.Ordered(box.Min, box.Max)); }

        public void AddSphere(CollisionSphere sphere) {
            if(sphere.Radius <= 1e-8f) return;
            spheres.Add(sphere);
        }

        public void AddCapsule(CollisionCapsule capsule) {
            if(capsule.Radius <= 1e-8f) return;
            capsules.Add(capsule);
        }

        public void AddOrientedBox(CollisionOrientedBox box) { orientedBoxes.Add(box); }

        public void AddTriangle(CollisionTriangle triangle) {
            if(triangle.Motion.SqrLength > 1e-8f) movingMeshes = true;
            triangles.Add(triangle);
            NoteTriangle(triangle.A);
            NoteTriangle(triangle.B);
            NoteTriangle(triangle.C);
        }

        void NoteTriangle(Vec3 point) {
            if(!triangleBoundsValid) {
                triangleMinX = triangleMaxX = point.X;
                triangleMinY = triangleMaxY = point.Y;
                triangleMinZ = triangleMaxZ = point.Z;
                triangleBoundsValid = true;
                return;
            }

            if(point.X < triangleMinX) triangleMinX = point.X;
            if(point.Y < triangleMinY) triangleMinY = point.Y;
            if(point.Z < triangleMinZ) triangleMinZ = point.Z;
            if(point.X > triangleMaxX) triangleMaxX = point.X;
            if(point.Y > triangleMaxY) triangleMaxY = point.Y;
            if(point.Z > triangleMaxZ) triangleMaxZ = point.Z;
        }

        public void ClearSolids() {
            spheres.Clear();
            capsules.Clear();
            orientedBoxes.Clear();
            triangles.Clear();
            movingMeshes = false;
            triangleBoundsValid = false;
        }

        public void ClearRetained() {
            planes.Clear();
            boxes.Clear();
        }

        public void BeginStep() {
            freshSurfaces = true;
            SurfaceQueries = 0;
            SurfaceSkips = 0;
            SurfaceMotion = 0f;
            BinHits = 0;
            BinMisses = 0;
            BinUnsure = 0;
            FullQueries = 0;
            PrepareMilliseconds = 0f;
            SurfaceMilliseconds = 0f;
            SolveMilliseconds = 0f;
            BroadMilliseconds = 0f;
            HoldMilliseconds = 0f;
            OrientedTests = 0;
            SphereTests = 0;
            TriangleTests = 0;
            contactCount = 0;
            rides.Clear();
            rideKeys.Clear();
            lookupStamp++;
            if(lookupStamp == 0) {
                lookupStamp = 1;
                System.Array.Clear(planeSeen, 0, planeSeen.Length);
                System.Array.Clear(boxSeen, 0, boxSeen.Length);
                System.Array.Clear(surfaceSeen, 0, surfaceSeen.Length);
                System.Array.Clear(sphereSeen, 0, sphereSeen.Length);
                System.Array.Clear(capsuleSeen, 0, capsuleSeen.Length);
                System.Array.Clear(orientedSeen, 0, orientedSeen.Length);
                System.Array.Clear(meshSeen, 0, meshSeen.Length);
            }
        }

        public void Collect(IReadOnlyList<DeformableBody> bodies) {
            collectId++;
            if(collectId == 0) collectId = 1;
            long broadStart = Stopwatch.GetTimestamp();
            PrepareLookup(bodies);
            RememberSpeeds(bodies);
            for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++) {
                Particle[] particles = bodies[bodyIndex].Particles;
                BodyMaterial material = bodies[bodyIndex].Material;
                float friction = material.Friction < 0f ? 0f : material.Friction;
                float restitution = material.Restitution < 0f ? 0f : material.Restitution;
                if(restitution > 1f) restitution = 1f;

                for (int particleIndex = 0; particleIndex < particles.Length; particleIndex++) {
                    Particle particle = particles[particleIndex];
                    if(particle.InverseMass <= 0f) continue;
                    for (int planeIndex = 0; planeIndex < planes.Count; planeIndex++) {
                        int existingPlane = Find(bodyIndex, particleIndex, PlaneKind, planeIndex);
                        if(existingPlane >= 0) {
                            contacts[existingPlane].Seen = collectId;
                            continue;
                        }
                        CollisionPlane plane = planes[planeIndex];
                        float separation = Vec3.Dot(plane.Normal, particle.Position) - plane.Offset;
                        if(separation >= 0f) continue;

                        Add(MakeContact(bodyIndex, particleIndex, PlaneKind, planeIndex, plane.Normal, plane.Offset, particle, friction, restitution));
                    }

                    for (int boxIndex = 0; boxIndex < boxes.Count; boxIndex++) {
                        int existingBox = Find(bodyIndex, particleIndex, BoxKind, boxIndex);
                        if(existingBox >= 0) {
                            contacts[existingBox].Seen = collectId;
                            continue;
                        }
                        if(!boxes[boxIndex].TryNearestExit(particle.Position, out Vec3 normal, out float offset, out _)) continue;

                        Add(MakeContact(bodyIndex, particleIndex, BoxKind, boxIndex, normal, offset, particle, friction, restitution));
                    }

                    CollectEnvironment(bodyIndex, particleIndex, particle, friction, restitution);
                }
            }

            ApplyRides(bodies);
            BroadMilliseconds += (float)((Stopwatch.GetTimestamp() - broadStart) * 1000.0 / Stopwatch.Frequency);
            long prepareStart = Stopwatch.GetTimestamp();
            PrepareSurfaces(bodies);
            PrepareMilliseconds += (float)((Stopwatch.GetTimestamp() - prepareStart) * 1000.0 / Stopwatch.Frequency);
            long surfaceStart = Stopwatch.GetTimestamp();
            CollectSurfaces(bodies);
            SurfaceMilliseconds += (float)((Stopwatch.GetTimestamp() - surfaceStart) * 1000.0 / Stopwatch.Frequency);
        }

        void CollectEnvironment(int bodyIndex, int particleIndex, Particle particle, float friction, float restitution) {
            float movedX = particle.Position.X - particle.Previous.X;
            float movedY = particle.Position.Y - particle.Previous.Y;
            float movedZ = particle.Position.Z - particle.Previous.Z;
            bool settled = !freshSurfaces && movedX * movedX + movedY * movedY + movedZ * movedZ < 0.0009f;
            for (int index = 0; index < spheres.Count; index++) {
                int existing = Find(bodyIndex, particleIndex, SphereKind, index);
                if(existing >= 0) {
                    contacts[existing].Seen = collectId;
                    continue;
                }

                CollisionSphere sphere = spheres[index];
                if(settled && sphere.Motion.SqrLength < 1e-6f) continue;
                if(MissesSphere(sphere, particle.Previous, particle.Position)) continue;
                SphereTests++;
                if(!sphere.TryContact(particle.Previous, particle.Position, out Vec3 normal, out float offset, out _)) continue;
                Add(Carry(MakeContact(bodyIndex, particleIndex, SphereKind, index, normal, offset, particle, MixFriction(friction, sphere.UseSurface, sphere.SurfaceFriction), MixRestitution(restitution, sphere.UseSurface, sphere.SurfaceRestitution)), SphereKind, index, sphere.Pose, sphere.Motion, sphere.Pose.At(particle.Position, sphere.Motion)));
            }

            for (int index = 0; index < capsules.Count; index++) {
                int existing = Find(bodyIndex, particleIndex, CapsuleKind, index);
                if(existing >= 0) {
                    contacts[existing].Seen = collectId;
                    continue;
                }

                CollisionCapsule capsule = capsules[index];
                if(settled && capsule.Motion.SqrLength < 1e-6f) continue;
                if(MissesCapsule(capsule, particle.Previous, particle.Position)) continue;
                if(!capsule.TryContact(particle.Previous, particle.Position, out Vec3 normal, out float offset, out _)) continue;
                Add(Carry(MakeContact(bodyIndex, particleIndex, CapsuleKind, index, normal, offset, particle, MixFriction(friction, capsule.UseSurface, capsule.SurfaceFriction), MixRestitution(restitution, capsule.UseSurface, capsule.SurfaceRestitution)), CapsuleKind, index, capsule.Pose, capsule.Motion, capsule.Pose.At(particle.Position, capsule.Motion)));
            }

            for (int index = 0; index < orientedBoxes.Count; index++) {
                int existing = Find(bodyIndex, particleIndex, OrientedKind, index);
                if(existing >= 0) {
                    contacts[existing].Seen = collectId;
                    continue;
                }

                CollisionOrientedBox box = orientedBoxes[index];
                if(settled && box.Motion.SqrLength < 1e-6f) continue;
                if(MissesOriented(box, particle.Previous, particle.Position)) continue;
                OrientedTests++;
                if(!box.TryContact(particle.Previous, particle.Position, out Vec3 normal, out float offset, out _)) continue;
                Add(Carry(MakeContact(bodyIndex, particleIndex, OrientedKind, index, normal, offset, particle, MixFriction(friction, box.UseSurface, box.SurfaceFriction), MixRestitution(restitution, box.UseSurface, box.SurfaceRestitution)), OrientedKind, index, box.Pose, box.Motion, box.Pose.At(particle.Position, box.Motion)));
            }

            if(triangles.Count == 0) return;

            if(RefreshMesh(bodyIndex, particleIndex)) return;

            if(settled && !movingMeshes) return;

            if(MissesTriangles(particle.Previous, particle.Position)) return;

            int best = -1;
            float bestDepth = float.MaxValue;
            Vec3 bestNormal = Vec3.Zero;
            float bestOffset = 0f;
            int second = -1;
            float secondDepth = float.MaxValue;
            Vec3 secondNormal = Vec3.Zero;
            float secondOffset = 0f;
            for (int index = 0; index < triangles.Count; index++) {
                CollisionTriangle candidate = triangles[index];
                if(candidate.Misses(particle.Previous, particle.Position)) continue;
                TriangleTests++;
                if(!candidate.TryContact(particle.Previous, particle.Position, out Vec3 normal, out float offset, out float depth))
                    continue;
                if(depth < bestDepth) {
                    second = best;
                    secondDepth = bestDepth;
                    secondNormal = bestNormal;
                    secondOffset = bestOffset;
                    best = index;
                    bestDepth = depth;
                    bestNormal = normal;
                    bestOffset = offset;
                    continue;
                }

                if(depth < secondDepth) {
                    second = index;
                    secondDepth = depth;
                    secondNormal = normal;
                    secondOffset = offset;
                }
            }

            if(best < 0) return;
            CollisionTriangle triangle = triangles[best];
            Add(Carry(MakeContact(bodyIndex, particleIndex, MeshKind, best, bestNormal, bestOffset, particle, MixFriction(friction, triangle.UseSurface, triangle.SurfaceFriction), MixRestitution(restitution, triangle.UseSurface, triangle.SurfaceRestitution)), MeshKind, best, triangle.Pose, triangle.Motion, triangle.Pose.At(particle.Position, triangle.Motion)));
            if(second < 0 || secondDepth > 0.1f || Vec3.Dot(secondNormal, bestNormal) > 0.75f) return;
            CollisionTriangle other = triangles[second];
            Add(Carry(MakeContact(bodyIndex, particleIndex, MeshKind, second, secondNormal, secondOffset, particle, MixFriction(friction, other.UseSurface, other.SurfaceFriction), MixRestitution(restitution, other.UseSurface, other.SurfaceRestitution)), MeshKind, second, other.Pose, other.Motion, other.Pose.At(particle.Position, other.Motion)));
        }

        Contact Carry(Contact contact, int shapeKind, int shapeIndex, ColliderPose pose, Vec3 translation, Vec3 motion) {
            contact.Anchor += motion;
            NoteRide(contact.BodyIndex, shapeKind, shapeIndex, pose, translation);
            return contact;
        }

        void NoteRide(int bodyIndex, int shapeKind, int shapeIndex, ColliderPose pose, Vec3 translation) {
            if(pose.HasBasis == 0) return;
            long key = ((long)bodyIndex << 42) | ((long)shapeKind << 21) | (uint)shapeIndex;
            if(!rideKeys.Add(key)) return;
            rides.Add(new Ride {
                BodyIndex = bodyIndex,
                Pose = pose,
                Translation = translation
            });
        }

        void ApplyRides(IReadOnlyList<DeformableBody> bodies) {
            for (int i = 0; i < rides.Count; i++) {
                Ride ride = rides[i];
                if(ride.BodyIndex < 0 || ride.BodyIndex >= bodies.Count) continue;
                Particle[] particles = bodies[ride.BodyIndex].Particles;

                Vec3 slide = new Vec3(ride.Translation.X, 0f, ride.Translation.Z);
                for (int p = 0; p < particles.Length; p++) {
                    Particle particle = particles[p];
                    if(particle.InverseMass <= 0f) continue;
                    Vec3 carried = ride.Pose.At(particle.Position, ride.Translation);
                    Vec3 carry = carried - ride.Translation + slide;
                    if(carry.SqrLength <= 1e-8f) continue;
                    particle.Position += carry;
                    particle.Previous += carry;
                    particles[p] = particle;
                }
            }

            rides.Clear();
        }

        struct Ride {
            public int BodyIndex;
            public ColliderPose Pose;
            public Vec3 Translation;
        }

        bool MissesTriangles(Vec3 from, Vec3 to) {
            if(!triangleBoundsValid) return true;
            const float pad = 1f;
            if((from.X < triangleMinX - pad && to.X < triangleMinX - pad) || (from.X > triangleMaxX + pad && to.X > triangleMaxX + pad))
                return true;
            if((from.Y < triangleMinY - pad && to.Y < triangleMinY - pad) || (from.Y > triangleMaxY + pad && to.Y > triangleMaxY + pad))
                return true;
            if((from.Z < triangleMinZ - pad && to.Z < triangleMinZ - pad) || (from.Z > triangleMaxZ + pad && to.Z > triangleMaxZ + pad))
                return true;
            return false;
        }

        static float MixFriction(float bodyFriction, int useSurface, float surfaceFriction) {
            if(useSurface == 0) return bodyFriction;
            float extra = surfaceFriction < 0f ? 0f : surfaceFriction;
            return 0.5f * (bodyFriction + extra);
        }

        static float MixRestitution(float bodyRestitution, int useSurface, float surfaceRestitution) {
            if(useSurface == 0) return bodyRestitution;
            float extra = surfaceRestitution < 0f ? 0f : surfaceRestitution;
            if(extra > 1f) extra = 1f;
            return extra > bodyRestitution ? extra : bodyRestitution;
        }

        void PrepareSurfaces(IReadOnlyList<DeformableBody> bodies) {
            int count = bodies.Count;
            if(faceOrigin.Length < count) {
                faceOrigin = new int[count];
                faceLength = new int[count];
                bodyBounds = new BodyBounds[count];
                bodyCenter = new Vec3[count];
                bodyReach = new float[count];
                bodyRound = new bool[count];
                bodyConvex = new bool[count];
                supported = new bool[count];
            }

            int needed = 0;
            for (int bodyIndex = 0; bodyIndex < count; bodyIndex++) {
                int[] triangles = bodies[bodyIndex].Triangles;
                if(triangles != null) needed += triangles.Length / 3;
            }

            if(facePlanes.Length < needed) Array.Resize(ref facePlanes, needed < 16 ? 16 : needed);

            bool heavy = freshSurfaces || SurfaceMoved(count);
            int cursor = 0;
            for (int bodyIndex = 0; bodyIndex < count; bodyIndex++) {
                Particle[] particles = bodies[bodyIndex].Particles;
                bodyBounds[bodyIndex] = FitBounds(particles);
                if(!heavy) continue;
                MeasureReach(particles, out bodyCenter[bodyIndex], out bodyReach[bodyIndex]);
                bodyRound[bodyIndex] = ContactSolver.IsRound(particles, bodies[bodyIndex].RestPositions);
                bodyConvex[bodyIndex] = bodies[bodyIndex].Convex;
                faceOrigin[bodyIndex] = cursor;
                int[] triangles = bodies[bodyIndex].Triangles;
                if(bodies[bodyIndex].ClosedSurface && triangles != null && particles != null)
                    cursor = ContactSolver.WriteFacePlanes(particles, triangles, facePlanes, cursor);
                faceLength[bodyIndex] = cursor - faceOrigin[bodyIndex];
            }

            if(!heavy) return;

            RememberSeedLayout();
            bool anyRound = false;
            for (int bodyIndex = 0; bodyIndex < count; bodyIndex++) { if(bodyRound[bodyIndex]) anyRound = true; }

            if(anyRound) BuildFaceNeighbors(bodies, cursor);
            BuildFaceBins(bodies);
        }

        bool SurfaceMoved(int count) {
            int limit = count < bodyMove.Length ? count : bodyMove.Length;
            for (int i = 0; i < limit; i++) { if(bodyMove[i] > 0.1f) return true; }

            return false;
        }

        static BodyBounds FitBounds(Particle[] particles) {
            if(particles == null || particles.Length == 0) return default;

            const float margin = 1e-3f;
            Vec3 first = particles[0].Position;
            float minX = first.X;
            float minY = first.Y;
            float minZ = first.Z;
            float maxX = first.X;
            float maxY = first.Y;
            float maxZ = first.Z;
            for (int i = 0; i < particles.Length; i++) {
                Include(ref minX, ref minY, ref minZ, ref maxX, ref maxY, ref maxZ, particles[i].Position);
                Include(ref minX, ref minY, ref minZ, ref maxX, ref maxY, ref maxZ, particles[i].Previous);
            }

            return new BodyBounds {
                MinX = minX - margin,
                MinY = minY - margin,
                MinZ = minZ - margin,
                MaxX = maxX + margin,
                MaxY = maxY + margin,
                MaxZ = maxZ + margin,
                Valid = true
            };
        }

        static void Include(ref float minX, ref float minY, ref float minZ, ref float maxX, ref float maxY, ref float maxZ, Vec3 point) {
            if(point.X < minX) minX = point.X;
            if(point.Y < minY) minY = point.Y;
            if(point.Z < minZ) minZ = point.Z;
            if(point.X > maxX) maxX = point.X;
            if(point.Y > maxY) maxY = point.Y;
            if(point.Z > maxZ) maxZ = point.Z;
        }

        static void MeasureReach(Particle[] particles, out Vec3 center, out float reach) {
            center = Vec3.Zero;
            reach = 0f;
            if(particles == null || particles.Length == 0) return;

            for (int i = 0; i < particles.Length; i++) center += particles[i].Position;
            center *= 1f / particles.Length;
            for (int i = 0; i < particles.Length; i++) {
                float current = (particles[i].Position - center).Length;
                float before = (particles[i].Previous - center).Length;
                if(current > reach) reach = current;
                if(before > reach) reach = before;
            }
        }

        static bool BoundsOverlap(BodyBounds a, BodyBounds b) {
            if(!a.Valid || !b.Valid) return true;
            return a.MaxX >= b.MinX && b.MaxX >= a.MinX
                && a.MaxY >= b.MinY && b.MaxY >= a.MinY
                && a.MaxZ >= b.MinZ && b.MaxZ >= a.MinZ;
        }

        void CollectSurfaces(IReadOnlyList<DeformableBody> bodies) {
            ClearInside(bodies.Count);
            MeasureMoves(bodies);
            EnsurePairs(bodies.Count);
            IndexSpans();
            forceSurfaces = freshSurfaces;
            freshSurfaces = false;
            if(forceSurfaces) {
                int pairCells = bodies.Count * bodies.Count;
                if(pairCells > pairAwake.Length) pairCells = pairAwake.Length;
                for (int i = 0; i < pairCells; i++) pairAwake[i] = 0;
            }
            int hot = 0;
            for (int vertexBody = 0; vertexBody < bodies.Count; vertexBody++) {
                for (int triangleBody = 0; triangleBody < bodies.Count; triangleBody++) {
                    if(Pairs(bodies, vertexBody, triangleBody)) hot++;
                }
            }

            if(hot < 6) {
                for (int vertexBody = 0; vertexBody < bodies.Count; vertexBody++) CollectGuest(bodies, vertexBody, null);
                RefreshAsleep();
                return;
            }

            if(batches.Length < bodies.Count) batches = new SurfaceBatch[bodies.Count];
            for (int i = 0; i < bodies.Count; i++) {
                if(batches[i] == null) batches[i] = new SurfaceBatch();
                batches[i].Clear();
                Particle[] particles = bodies[i].Particles;
                CoverRow(i, particles == null ? 0 : particles.Length);
            }

            Parallel.For(0, bodies.Count, vertexBody => CollectGuest(bodies, vertexBody, batches[vertexBody]));
            for (int vertexBody = 0; vertexBody < bodies.Count; vertexBody++) batches[vertexBody].Commit(this);
            RefreshAsleep();
        }

        bool Pairs(IReadOnlyList<DeformableBody> bodies, int vertexBody, int triangleBody) {
            if(vertexBody == triangleBody) return false;
            if(!bodies[triangleBody].ClosedSurface) return true;
            if(BoundsOverlap(bodyBounds[vertexBody], bodyBounds[triangleBody]) && ReachesOverlap(vertexBody, triangleBody)) return true;
            float moveA = Moved(vertexBody);
            float moveB = Moved(triangleBody);
            if(moveA + moveB <= 0f) return false;
            return BoundsOverlap(Pad(bodyBounds[vertexBody], moveA), Pad(bodyBounds[triangleBody], moveB))
                && ReachesWithin(vertexBody, triangleBody, moveA + moveB);
        }

        float Moved(int body) { return body >= 0 && body < bodyMove.Length ? bodyMove[body] : 0f; }

        bool ReachesWithin(int a, int b, float pad) {
            float limit = bodyReach[a] + bodyReach[b] + pad;
            return (bodyCenter[a] - bodyCenter[b]).SqrLength <= limit * limit;
        }

        static BodyBounds Pad(BodyBounds box, float pad) {
            if(!box.Valid || pad <= 0f) return box;
            box.MinX -= pad;
            box.MinY -= pad;
            box.MinZ -= pad;
            box.MaxX += pad;
            box.MaxY += pad;
            box.MaxZ += pad;
            return box;
        }

        void CollectGuest(IReadOnlyList<DeformableBody> bodies, int vertexBody, SurfaceBatch batch) {
            for (int triangleBody = 0; triangleBody < bodies.Count; triangleBody++) {
                if(!Pairs(bodies, vertexBody, triangleBody)) continue;
                int key = vertexBody * insideStride + triangleBody;
                float step = bodyMove[vertexBody] + bodyMove[triangleBody];
                if(!forceSurfaces
                    && bodies[vertexBody].ClosedSurface
                    && bodies[triangleBody].ClosedSurface
                    && pairAwake[key] == 1) {
                    pairDrift[key] += step;
                    if(pairDrift[key] < PairQuerySlack) {
                        SurfaceSkips++;
                        pairAsleep[key] = 1;
                        insideCount[key] = lastInside[key];
                        continue;
                    }
                }

                SurfaceQueries++;
                pairAsleep[key] = 0;
                pairDrift[key] = 0f;
                pairAwake[key] = 1;
                CollectVerticesAgainst(bodies, vertexBody, triangleBody, batch);
                lastInside[key] = insideCount[key];
            }
        }

        void ClearInside(int count) {
            insideStride = count;
            int needed = count * count;
            if(insideCount.Length < needed) insideCount = new int[needed];
            for (int i = 0; i < needed; i++) insideCount[i] = 0;
        }

        bool ReachesOverlap(int a, int b) {
            float limit = bodyReach[a] + bodyReach[b];
            return (bodyCenter[a] - bodyCenter[b]).SqrLength <= limit * limit;
        }

        void NoteInside(int vertexBody, int triangleBody) {
            if(insideStride <= 0 || vertexBody < 0 || triangleBody < 0 || vertexBody >= insideStride || triangleBody >= insideStride)
                return;
            insideCount[vertexBody * insideStride + triangleBody]++;
        }

        int InsideOf(int vertexBody, int triangleBody) {
            if(insideStride <= 0 || vertexBody < 0 || triangleBody < 0 || vertexBody >= insideStride || triangleBody >= insideStride)
                return 0;
            return insideCount[vertexBody * insideStride + triangleBody];
        }

        void CollectVerticesAgainst(IReadOnlyList<DeformableBody> bodies, int vertexBody, int triangleBody, SurfaceBatch batch) {
            int[] triangles = bodies[triangleBody].Triangles;
            if(triangles == null || triangles.Length < 3) return;

            Particle[] points = bodies[vertexBody].Particles;
            Particle[] surface = bodies[triangleBody].Particles;
            float friction = CombinedFriction(bodies[vertexBody].Material.Friction, bodies[triangleBody].Material.Friction);
            float restitution = CombinedRestitution(bodies[vertexBody].Material.Restitution, bodies[triangleBody].Material.Restitution);
            int token = triangleBody + 1;
            int[] cover = CoverRow(vertexBody, points.Length);

            for (int particleIndex = 0; particleIndex < points.Length; particleIndex++) {
                Vec3 sample = points[particleIndex].Position;
                Vec3 previous = points[particleIndex].Previous;
                if(bodies[triangleBody].ClosedSurface && Misses(triangleBody, previous, sample)) continue;
                bool hit = bodies[triangleBody].ClosedSurface
                    ? QueryClosed(triangleBody, surface, triangles, previous, sample, vertexBody, particleIndex, out int triangle, out Vec3 normal, out float distance, out float weightA, out float weightB, out float weightC)
                    : ContactSolver.TryNearestSurface(surface, triangles, sample, out triangle, out normal, out distance, out weightA, out weightB, out weightC);
                if(!hit) continue;
                cover[particleIndex] = token;
                int contained = Contained(normal, previous, surface, triangles, triangle);
                if(contained != 0) NoteInside(vertexBody, triangleBody);
                if(points[particleIndex].InverseMass <= 0f && TriangleFixed(surface, triangles, triangle)) continue;

                int existing = FindSurface(vertexBody, particleIndex, triangleBody);
                if(existing >= 0) {
                    Contact updated = contacts[existing];
                    updated.Seen = collectId;
                    updated.Anchor = new Vec3(weightA, weightB, weightC);
                    updated.Normal = normal;
                    updated.Contained = contained;
                    if(updated.ShapeIndex != triangle) {
                        updated.ShapeIndex = triangle;
                        updated.Lambda = 0f;
                    }

                    SaveExisting(batch, existing, updated);
                    continue;
                }

                int slot = triangle * 3;
                Vec3 surfaceVelocity =
                    surface[triangles[slot]].Velocity * weightA
                    + surface[triangles[slot + 1]].Velocity * weightB
                    + surface[triangles[slot + 2]].Velocity * weightC;
                SaveNew(batch, new Contact {
                    BodyIndex = vertexBody,
                    ParticleIndex = particleIndex,
                    OtherBodyIndex = triangleBody,
                    OtherParticleIndex = -1,
                    ThirdParticleIndex = -1,
                    ShapeKind = TriangleKind,
                    ShapeIndex = triangle,
                    Normal = normal,
                    Offset = distance,
                    Anchor = new Vec3(weightA, weightB, weightC),
                    IncomingNormalSpeed = Vec3.Dot(points[particleIndex].Velocity - surfaceVelocity, normal),
                    Friction = friction,
                    Restitution = restitution,
                    Contained = contained
                });
            }

            CollectSpans(bodies, vertexBody, triangleBody, batch, cover, token);
        }

        void CollectSpans(IReadOnlyList<DeformableBody> bodies, int vertexBody, int triangleBody, SurfaceBatch batch, int[] cover, int token) {
            if(!bodies[triangleBody].ClosedSurface) return;
            if(!forceSurfaces
                && triangleBody < bodyConvex.Length
                && bodyConvex[triangleBody]
                && vertexBody < bodyMove.Length
                && triangleBody < bodyMove.Length
                && bodyMove[vertexBody] + bodyMove[triangleBody] < PairQuerySlack)
                return;

            int[] own = bodies[vertexBody].Triangles;
            Particle[] points = bodies[vertexBody].Particles;
            Particle[] surface = bodies[triangleBody].Particles;
            int[] triangles = bodies[triangleBody].Triangles;
            if(own == null || points == null || surface == null || triangles == null) return;

            for (int slot = 0; slot + 2 < own.Length; slot += 3) {
                int i0 = own[slot];
                int i1 = own[slot + 1];
                int i2 = own[slot + 2];
                if(!Inside(points, i0) || !Inside(points, i1) || !Inside(points, i2)) continue;

                if(i0 < i1 && !MissesEdge(triangleBody, points[i0], points[i1]))
                    ConsiderEdge(points, surface, triangles, vertexBody, triangleBody, i0, i1, batch, cover, token);
                if(i1 < i2 && !MissesEdge(triangleBody, points[i1], points[i2]))
                    ConsiderEdge(points, surface, triangles, vertexBody, triangleBody, i1, i2, batch, cover, token);
                if(i2 < i0 && !MissesEdge(triangleBody, points[i2], points[i0]))
                    ConsiderEdge(points, surface, triangles, vertexBody, triangleBody, i2, i0, batch, cover, token);
                if(!MissesFace(triangleBody, points[i0], points[i1], points[i2]))
                    ConsiderFace(points, surface, triangles, vertexBody, triangleBody, i0, i1, i2, batch, cover, token);
            }
        }

        static bool Inside(Particle[] points, int index) { return index >= 0 && index < points.Length; }

        void ConsiderEdge(Particle[] points, Particle[] surface, int[] triangles, int vertexBody, int triangleBody, int i0, int i1, SurfaceBatch batch, int[] cover, int token) {
            bool cover0 = Covered(cover, token, i0);
            bool cover1 = Covered(cover, token, i1);
            if(cover0 && cover1) return;
            if(EdgeMissesReach(triangleBody, points[i0], points[i1])) return;

            bool found = false;
            float bestDistance = 0f;
            float bestT = 0.5f;
            int triangle = 0;
            Vec3 normal = Vec3.Zero;
            Vec3 previousSample = Vec3.Zero;
            float weightA = 0f;
            float weightB = 0f;
            float weightC = 0f;
            int sampleCount = 1;
            for (int sample = 0; sample < sampleCount; sample++) {
                float t = sampleCount == 1 ? 0.5f : (sample == 0 ? 0.35f : 0.65f);
                float keep = 1f - t;
                Vec3 position = points[i0].Position * keep + points[i1].Position * t;
                Vec3 previous = points[i0].Previous * keep + points[i1].Previous * t;
                if(!QueryClosed(triangleBody, surface, triangles, previous, position, vertexBody, i0, out int hit, out Vec3 hitNormal, out float distance, out float hitWeightA, out float hitWeightB, out float hitWeightC))
                    continue;
                if(found && distance >= bestDistance) continue;
                found = true;
                bestDistance = distance;
                bestT = t;
                triangle = hit;
                normal = hitNormal;
                previousSample = previous;
                weightA = hitWeightA;
                weightB = hitWeightB;
                weightC = hitWeightC;
            }

            if(!found) return;

            int contained = Contained(normal, previousSample, surface, triangles, triangle);
            int existing = FindSpan(vertexBody, triangleBody, EdgeKind, i0, i1, -1);
            if(existing >= 0) {
                Contact updated = contacts[existing];
                updated.Seen = collectId;
                updated.ShapeIndex = triangle;
                updated.Normal = normal;
                updated.Offset = bestT;
                updated.Anchor = new Vec3(weightA, weightB, weightC);
                updated.Contained = contained;
                SaveExisting(batch, existing, updated);
                return;
            }

            SaveNew(batch, new Contact {
                BodyIndex = vertexBody,
                ParticleIndex = i0,
                OtherParticleIndex = i1,
                ThirdParticleIndex = -1,
                OtherBodyIndex = triangleBody,
                ShapeKind = EdgeKind,
                ShapeIndex = triangle,
                Normal = normal,
                Offset = bestT,
                Anchor = new Vec3(weightA, weightB, weightC),
                Contained = contained
            });
        }

        void ConsiderFace(Particle[] points, Particle[] surface, int[] triangles, int vertexBody, int triangleBody, int i0, int i1, int i2, SurfaceBatch batch, int[] cover, int token) {
            if(Covered(cover, token, i0) && Covered(cover, token, i1) && Covered(cover, token, i2)) return;

            Vec3 position = (points[i0].Position + points[i1].Position + points[i2].Position) * (1f / 3f);
            Vec3 previous = (points[i0].Previous + points[i1].Previous + points[i2].Previous) * (1f / 3f);
            if(!QueryClosed(triangleBody, surface, triangles, previous, position, vertexBody, i0, out int triangle, out Vec3 normal, out _, out float weightA, out float weightB, out float weightC))
                return;

            int contained = Contained(normal, previous, surface, triangles, triangle);
            int existing = FindSpan(vertexBody, triangleBody, FaceKind, i0, i1, i2);
            if(existing >= 0) {
                Contact updated = contacts[existing];
                updated.Seen = collectId;
                updated.ShapeIndex = triangle;
                updated.Normal = normal;
                updated.Anchor = new Vec3(weightA, weightB, weightC);
                updated.Contained = contained;
                SaveExisting(batch, existing, updated);
                return;
            }

            SaveNew(batch, new Contact {
                BodyIndex = vertexBody,
                ParticleIndex = i0,
                OtherParticleIndex = i1,
                ThirdParticleIndex = i2,
                OtherBodyIndex = triangleBody,
                ShapeKind = FaceKind,
                ShapeIndex = triangle,
                Normal = normal,
                Anchor = new Vec3(weightA, weightB, weightC),
                Contained = contained
            });
        }

        static int Contained(Vec3 normal, Vec3 previous, Particle[] surface, int[] triangles, int triangle) {
            int slot = triangle * 3;
            if(surface == null || triangles == null || slot < 0 || slot + 2 >= triangles.Length) return 0;
            int index = triangles[slot];
            if(index < 0 || index >= surface.Length) return 0;
            return Vec3.Dot(normal, previous - surface[index].Position) <= 1e-5f ? 1 : 0;
        }

        static bool TriangleFixed(Particle[] surface, int[] triangles, int triangle) {
            int slot = triangle * 3;
            return surface[triangles[slot]].InverseMass <= 0f
                && surface[triangles[slot + 1]].InverseMass <= 0f
                && surface[triangles[slot + 2]].InverseMass <= 0f;
        }

        static float CombinedFriction(float a, float b) {
            if(a < 0f) a = 0f;
            if(b < 0f) b = 0f;
            return a < b ? a : b;
        }

        static float CombinedRestitution(float a, float b) {
            if(a < 0f) a = 0f;
            if(b < 0f) b = 0f;
            float value = a > b ? a : b;
            return value > 1f ? 1f : value;
        }

        static Contact MakeContact(int bodyIndex, int particleIndex, int shapeKind, int shapeIndex, Vec3 normal, float offset, Particle particle, float friction, float restitution) {
            return new Contact {
                BodyIndex = bodyIndex,
                ParticleIndex = particleIndex,
                OtherBodyIndex = -1,
                OtherParticleIndex = -1,
                ThirdParticleIndex = -1,
                ShapeKind = shapeKind,
                ShapeIndex = shapeIndex,
                Normal = normal,
                Offset = offset,
                Anchor = particle.Previous + normal * (offset - Vec3.Dot(normal, particle.Previous)),
                IncomingNormalSpeed = Vec3.Dot(particle.Velocity, normal),
                Friction = friction,
                Restitution = restitution
            };
        }

    }
}
