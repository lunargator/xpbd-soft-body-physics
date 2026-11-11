using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;
using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Collision {
    public sealed partial class CollisionWorld {
        public void Solve(IReadOnlyList<DeformableBody> bodies, float stepSeconds) {
            long solveStart = Stopwatch.GetTimestamp();
            RefreshSupported(bodies);
            for (int i = 0; i < contactCount; i++) SolveOne(bodies, stepSeconds, i, false);
            SolveMilliseconds += (float)((Stopwatch.GetTimestamp() - solveStart) * 1000.0 / Stopwatch.Frequency);
        }

        public void HoldShells(IReadOnlyList<DeformableBody> bodies) {
            long holdStart = Stopwatch.GetTimestamp();
            if(bodies == null) {
                HoldMilliseconds += (float)((Stopwatch.GetTimestamp() - holdStart) * 1000.0 / Stopwatch.Frequency);
                return;
            }
            for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++) {
                bool touching = false;
                for (int i = 0; i < contactCount; i++) {
                    if(contacts[i].Seen != collectId || contacts[i].BodyIndex != bodyIndex) continue;
                    touching = true;
                    break;
                }

                if(touching) ContactSolver.HoldShell(bodies[bodyIndex].Particles, bodies[bodyIndex].RestPositions, ShellScale);
            }

            HoldMilliseconds += (float)((Stopwatch.GetTimestamp() - holdStart) * 1000.0 / Stopwatch.Frequency);
        }

        public void PinOpenSheets(IReadOnlyList<DeformableBody> bodies) {
            SeatSolids(bodies);
            PushOutOfSolids(bodies);
            if(bodies == null || triangles.Count == 0) return;

            for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++) {
                Particle[] particles = bodies[bodyIndex].Particles;
                if(particles == null) continue;
                for (int particleIndex = 0; particleIndex < particles.Length; particleIndex++) {
                    Particle particle = particles[particleIndex];
                    if(particle.InverseMass <= 0f) continue;
                    if(MissesTriangles(particle.Previous, particle.Position)) continue;

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
                        if(candidate.Closed != 0) continue;
                        if(candidate.Misses(particle.Previous, particle.Position)) continue;
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

                    if(best < 0) continue;
                    Seat(ref particle, bestNormal, bestOffset);
                    if(second >= 0 && secondDepth <= 0.1f && Vec3.Dot(secondNormal, bestNormal) <= 0.75f)
                        Seat(ref particle, secondNormal, secondOffset);
                    particles[particleIndex] = particle;
                }
            }
        }

        void PushOutOfSolids(IReadOnlyList<DeformableBody> bodies) {
            if(bodies == null) return;
            for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++) {
                Particle[] particles = bodies[bodyIndex].Particles;
                if(particles == null) continue;
                for (int particleIndex = 0; particleIndex < particles.Length; particleIndex++) {
                    Particle particle = particles[particleIndex];
                    if(particle.InverseMass <= 0f) continue;
                    bool moved = false;
                    for (int i = 0; i < spheres.Count; i++) {
                        CollisionSphere sphere = spheres[i];
                        if(MissesSphere(sphere, particle.Position, particle.Position)) continue;
                        if(!sphere.TryNearestExit(particle.Position, out Vec3 normal, out float offset, out _)) continue;
                        Seat(ref particle, normal, offset);
                        moved = true;
                    }

                    for (int i = 0; i < capsules.Count; i++) {
                        CollisionCapsule capsule = capsules[i];
                        if(MissesCapsule(capsule, particle.Position, particle.Position)) continue;
                        if(!capsule.TryNearestExit(particle.Position, out Vec3 normal, out float offset, out _)) continue;
                        Seat(ref particle, normal, offset);
                        moved = true;
                    }

                    for (int i = 0; i < orientedBoxes.Count; i++) {
                        CollisionOrientedBox box = orientedBoxes[i];
                        if(MissesOriented(box, particle.Position, particle.Position)) continue;
                        if(!box.TryNearestExit(particle.Position, out Vec3 normal, out float offset, out _)) continue;
                        Seat(ref particle, normal, offset);
                        moved = true;
                    }

                    if(moved) particles[particleIndex] = particle;
                }
            }
        }

        void SeatSolids(IReadOnlyList<DeformableBody> bodies) {
            if(bodies == null) return;
            for (int i = 0; i < contactCount; i++) {
                Contact contact = contacts[i];
                if(contact.Seen != collectId || contact.OtherBodyIndex >= 0) continue;
                if(contact.ShapeKind == EdgeKind || contact.ShapeKind == FaceKind) continue;
                int bodyIndex = contact.BodyIndex;
                int particleIndex = contact.ParticleIndex;
                if(bodyIndex < 0 || bodyIndex >= bodies.Count) continue;
                Particle[] particles = bodies[bodyIndex].Particles;
                if(particles == null || particleIndex < 0 || particleIndex >= particles.Length) continue;
                Particle particle = particles[particleIndex];
                if(particle.InverseMass <= 0f) continue;
                Seat(ref particle, contact.Normal, contact.Offset);
                particles[particleIndex] = particle;
            }
        }

        static void Seat(ref Particle particle, Vec3 normal, float offset) {
            float separation = Vec3.Dot(normal, particle.Position) - offset;
            if(separation < 0f) particle.Position += normal * (-separation);
            Vec3 delta = particle.Position - particle.Previous;
            float along = Vec3.Dot(delta, normal);
            if(along < 0f) particle.Previous += normal * along;
        }

        public void SolveMesh(IReadOnlyList<DeformableBody> bodies, float stepSeconds) {
            RefreshSupported(bodies);
            for (int i = 0; i < contactCount; i++) SolveOne(bodies, stepSeconds, i, true);
        }

        void SolveOne(IReadOnlyList<DeformableBody> bodies, float stepSeconds, int index, bool meshOnly) {
            Contact contact = contacts[index];
            if(contact.Seen != collectId) return;
            if(contact.BodyIndex < 0 || contact.BodyIndex >= bodies.Count) return;
            if(SkipsStuck(contact, bodies)) return;
            if(contact.ShapeKind == EdgeKind || contact.ShapeKind == FaceKind) {
                if(contact.OtherBodyIndex < 0 || contact.OtherBodyIndex >= bodies.Count) return;
                ContactSolver.SolveSpan(
                    ref contact,
                    bodies[contact.BodyIndex].Particles,
                    bodies[contact.BodyIndex].RestPositions,
                    bodies[contact.OtherBodyIndex].Particles,
                    bodies[contact.OtherBodyIndex].Triangles,
                    stepSeconds);
            } else if(contact.OtherBodyIndex >= 0) {
                if(contact.OtherBodyIndex >= bodies.Count) return;
                ContactSolver.SolveTriangle(
                    ref contact,
                    bodies[contact.BodyIndex].Particles,
                    bodies[contact.BodyIndex].RestPositions,
                    bodies[contact.OtherBodyIndex].Particles,
                    bodies[contact.OtherBodyIndex].RestPositions,
                    bodies[contact.OtherBodyIndex].Triangles,
                    stepSeconds,
                    !bodies[contact.OtherBodyIndex].ClosedSurface);
            } else if(!meshOnly) { ContactSolver.Solve(ref contact, bodies[contact.BodyIndex].Particles, stepSeconds); }

            contacts[index] = contact;
        }

        public void ApplyRestitution(IReadOnlyList<DeformableBody> bodies) {
            if(bounceKeep.Length < contactCount) bounceKeep = new int[contactCount];

            for (int i = 0; i < contactCount; i++) {
                Contact contact = contacts[i];
                bounceKeep[i] = 0;
                if(contact.BodyIndex < 0 || contact.BodyIndex >= bodies.Count) continue;
                if(contact.ParticleIndex < 0) continue;
                if(contact.IncomingNormalSpeed >= 0f) continue;
                bounceKeep[i] = 1;
            }

            for (int i = 0; i < contactCount; i++) {
                if(bounceKeep[i] == 0) continue;
                Contact contact = contacts[i];
                for (int j = i + 1; j < contactCount; j++) {
                    if(bounceKeep[j] == 0) continue;
                    Contact other = contacts[j];
                    if(other.BodyIndex != contact.BodyIndex || other.ParticleIndex != contact.ParticleIndex) continue;
                    if(other.IncomingNormalSpeed < contact.IncomingNormalSpeed) {
                        bounceKeep[i] = 0;
                        break;
                    }

                    bounceKeep[j] = 0;
                }
            }

            for (int i = 0; i < contactCount; i++) {
                if(bounceKeep[i] == 0) continue;
                Contact contact = contacts[i];
                if(contact.OtherBodyIndex >= 0) {
                    if(contact.OtherBodyIndex >= bodies.Count) continue;
                    ContactSolver.ApplyTriangleRestitution(
                        ref contact,
                        bodies[contact.BodyIndex].Particles,
                        bodies[contact.OtherBodyIndex].Particles,
                        bodies[contact.OtherBodyIndex].Triangles,
                        RestitutionSleepSpeed,
                        !bodies[contact.OtherBodyIndex].ClosedSurface);
                } else { ContactSolver.ApplyRestitution(ref contact, bodies[contact.BodyIndex].Particles, RestitutionSleepSpeed); }

                contacts[i] = contact;
            }

            ShareImpact(bodies);
            CapSpeeds(bodies);
        }

        void ShareImpact(IReadOnlyList<DeformableBody> bodies) {
            if(bodies == null || bodies.Count == 0) return;
            if(impactPick.Length < bodies.Count) impactPick = new int[bodies.Count];
            for (int i = 0; i < bodies.Count; i++) impactPick[i] = -1;

            for (int i = 0; i < contactCount; i++) {
                Contact contact = contacts[i];
                if(contact.Contained != 0 || contact.IncomingNormalSpeed >= 0f || contact.OtherBodyIndex < 0) continue;
                int struck = contact.OtherBodyIndex;
                if(struck >= bodies.Count || contact.BodyIndex < 0 || contact.BodyIndex >= bodies.Count) continue;
                if(!bodies[struck].ClosedSurface) continue;
                int chosen = impactPick[struck];
                if(chosen < 0 || contact.IncomingNormalSpeed < contacts[chosen].IncomingNormalSpeed) impactPick[struck] = i;
            }

            int pairCount = bodies.Count * bodies.Count;
            if(impactShared.Length < pairCount) impactShared = new bool[pairCount];
            for (int i = 0; i < pairCount; i++) impactShared[i] = false;
            for (int struck = 0; struck < bodies.Count; struck++) {
                int index = impactPick[struck];
                if(index < 0) continue;
                Contact contact = contacts[index];
                int hitter = contact.BodyIndex;
                int low = hitter < struck ? hitter : struck;
                int high = hitter < struck ? struck : hitter;
                int pair = low * bodies.Count + high;
                if(impactShared[pair]) continue;
                impactShared[pair] = true;

                float struckSpeed = struck < speedLimit.Length ? speedLimit[struck] : 0f;
                float hitterSpeed = hitter < speedLimit.Length ? speedLimit[hitter] : 0f;
                if(struckSpeed > hitterSpeed + 0.5f) continue;
                float approach = -contact.IncomingNormalSpeed;
                if(approach < 1.2f) continue;

                float hitterMass = DynamicMass(bodies[hitter]);
                float struckMass = DynamicMass(bodies[struck]);
                float total = hitterMass + struckMass;
                if(hitterMass <= 1e-8f || struckMass <= 1e-8f) continue;

                float relative = approach * 0.9f;
                Vec3 normal = ImpactNormal(bodies[hitter], bodies[struck], contact.Normal);
                AddVelocity(bodies[struck], normal * (-relative * (hitterMass / total)));
                AddVelocity(bodies[hitter], normal * (relative * (struckMass / total)));
            }
        }

        static Vec3 ImpactNormal(DeformableBody hitter, DeformableBody struck, Vec3 contactNormal) {
            Vec3 towardHitter = BodyCenter(hitter) - BodyCenter(struck);
            float length = towardHitter.Length;
            if(length < 1e-4f || Vec3.Dot(towardHitter, contactNormal) <= 0f) return contactNormal;
            return towardHitter * (1f / length);
        }

        static Vec3 BodyCenter(DeformableBody body) {
            Particle[] particles = body.Particles;
            if(particles == null || particles.Length == 0) return Vec3.Zero;
            Vec3 sum = Vec3.Zero;
            float mass = 0f;
            for (int i = 0; i < particles.Length; i++) {
                float inverse = particles[i].InverseMass;
                if(inverse <= 0f) continue;
                float particleMass = 1f / inverse;
                sum += particles[i].Position * particleMass;
                mass += particleMass;
            }

            return mass > 0f ? sum * (1f / mass) : particles[0].Position;
        }

        static float DynamicMass(DeformableBody body) {
            float mass = 0f;
            Particle[] particles = body.Particles;
            if(particles == null) return 0f;
            for (int i = 0; i < particles.Length; i++) {
                float inverse = particles[i].InverseMass;
                if(inverse > 0f) mass += 1f / inverse;
            }

            return mass;
        }

        static void AddVelocity(DeformableBody body, Vec3 change) {
            Particle[] particles = body.Particles;
            if(particles == null || change.SqrLength <= 0f) return;
            for (int i = 0; i < particles.Length; i++) {
                Particle particle = particles[i];
                if(particle.InverseMass <= 0f) continue;
                particle.Velocity += change;
                particles[i] = particle;
            }
        }

        void RememberSpeeds(IReadOnlyList<DeformableBody> bodies) {
            if(speedLimit.Length < bodies.Count) speedLimit = new float[bodies.Count];
            for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++) {
                Particle[] particles = bodies[bodyIndex].Particles;
                float fastest = 0f;
                for (int i = 0; i < particles.Length; i++) {
                    float speed = particles[i].Velocity.Length;
                    if(speed > fastest) fastest = speed;
                }

                speedLimit[bodyIndex] = fastest;
            }
        }

        void CapSpeeds(IReadOnlyList<DeformableBody> bodies) {
            int count = bodies.Count < speedLimit.Length ? bodies.Count : speedLimit.Length;
            for (int bodyIndex = 0; bodyIndex < count; bodyIndex++) {
                float cap = speedLimit[bodyIndex];
                if(cap < 1e-4f) continue;
                float capSqr = cap * cap;
                Particle[] particles = bodies[bodyIndex].Particles;
                for (int i = 0; i < particles.Length; i++) {
                    Particle particle = particles[i];
                    float speedSqr = particle.Velocity.SqrLength;
                    if(speedSqr <= capSqr || speedSqr < 1e-8f) continue;
                    particle.Velocity *= cap / (float)Math.Sqrt(speedSqr);
                    particles[i] = particle;
                }
            }
        }

        void MeasureMoves(IReadOnlyList<DeformableBody> bodies) {
            int count = bodies.Count;
            if(bodyMove.Length < count) bodyMove = new float[count];
            if(motionSample.Length < particleSpan || motionParticles != particleSpan) {
                motionSample = new Vec3[particleSpan < 1 ? 1 : particleSpan];
                motionParticles = particleSpan;
                motionReady = false;
            }

            for (int bodyIndex = 0; bodyIndex < count; bodyIndex++) {
                Particle[] particles = bodies[bodyIndex].Particles;
                float maxSqr = 0f;
                int origin = bodyIndex < particleOffset.Length ? particleOffset[bodyIndex] : 0;
                if(particles != null) {
                    for (int i = 0; i < particles.Length; i++) {
                        int slot = origin + i;
                        if(slot < 0 || slot >= motionSample.Length) continue;
                        Vec3 position = particles[i].Position;
                        if(motionReady) {
                            float dx = position.X - motionSample[slot].X;
                            float dy = position.Y - motionSample[slot].Y;
                            float dz = position.Z - motionSample[slot].Z;
                            float sqr = dx * dx + dy * dy + dz * dz;
                            if(sqr > maxSqr) maxSqr = sqr;
                        }

                        motionSample[slot] = position;
                    }
                }

                bodyMove[bodyIndex] = motionReady ? (float)Math.Sqrt(maxSqr) : 1f;
                if(bodyMove[bodyIndex] > SurfaceMotion) SurfaceMotion = bodyMove[bodyIndex];
            }

            motionReady = true;
        }

        void EnsurePairs(int count) {
            int need = count * count;
            if(need < 1) need = 1;
            if(pairWidth != count || pairDrift.Length < need) {
                pairWidth = count;
                pairDrift = new float[need];
                pairAwake = new int[need];
                lastInside = new int[need];
                pairAsleep = new int[need];
                motionReady = false;
                return;
            }

            for (int i = 0; i < need; i++) pairAsleep[i] = 0;
        }

        void RefreshAsleep() {
            for (int i = 0; i < contactCount; i++) {
                int other = contacts[i].OtherBodyIndex;
                int body = contacts[i].BodyIndex;
                if(other < 0 || body < 0 || insideStride <= 0) continue;
                int key = body * insideStride + other;
                if(key < 0 || key >= pairAsleep.Length || pairAsleep[key] == 0) continue;
                Contact contact = contacts[i];
                contact.Seen = collectId;
                contacts[i] = contact;
            }
        }

        void IndexSpans() {
            int cap = 16;
            int need = contactCount * 2;
            if(need < 16) need = 16;
            while (cap < need) cap <<= 1;
            if(spanSlot.Length != cap) {
                spanSlot = new int[cap];
                spanSeen = new int[cap];
                spanStamp = 0;
            }

            spanStamp++;
            if(spanStamp == 0) {
                Array.Clear(spanSeen, 0, spanSeen.Length);
                spanStamp = 1;
            }

            spanMask = cap - 1;
            for (int i = 0; i < contactCount; i++) {
                int kind = contacts[i].ShapeKind;
                if(kind != EdgeKind && kind != FaceKind) continue;
                int slot = SpanHash(
                    contacts[i].BodyIndex,
                    contacts[i].OtherBodyIndex,
                    kind,
                    contacts[i].ParticleIndex,
                    contacts[i].OtherParticleIndex,
                    contacts[i].ThirdParticleIndex) & spanMask;
                for (int n = 0; n < spanSlot.Length; n++) {
                    if(spanSeen[slot] != spanStamp) {
                        spanSeen[slot] = spanStamp;
                        spanSlot[slot] = i;
                        break;
                    }

                    slot = (slot + 1) & spanMask;
                }
            }
        }

        static int SpanHash(int body, int host, int kind, int i0, int i1, int i2) {
            unchecked {
                int hash = 17;
                hash = hash * 31 + body;
                hash = hash * 31 + host;
                hash = hash * 31 + kind;
                hash = hash * 31 + i0;
                hash = hash * 31 + i1;
                hash = hash * 31 + i2;
                return hash;
            }
        }

        int Find(int bodyIndex, int particleIndex, int shapeKind, int shapeIndex) {
            if(shapeKind == PlaneKind && planeStride > 0)
                return Stamped(planeSlot, planeSeen, planeStride, bodyIndex, particleIndex, shapeIndex);
            if(shapeKind == BoxKind && boxStride > 0) return Stamped(boxSlot, boxSeen, boxStride, bodyIndex, particleIndex, shapeIndex);
            if(shapeKind == SphereKind && sphereStride > 0)
                return Stamped(sphereSlot, sphereSeen, sphereStride, bodyIndex, particleIndex, shapeIndex);
            if(shapeKind == CapsuleKind && capsuleStride > 0)
                return Stamped(capsuleSlot, capsuleSeen, capsuleStride, bodyIndex, particleIndex, shapeIndex);
            if(shapeKind == OrientedKind && orientedStride > 0)
                return Stamped(orientedSlot, orientedSeen, orientedStride, bodyIndex, particleIndex, shapeIndex);
            if(shapeKind == MeshKind) return FindMesh(bodyIndex, particleIndex, shapeIndex);

            for (int i = 0; i < contactCount; i++) {
                Contact contact = contacts[i];
                if(contact.BodyIndex == bodyIndex
                    && contact.ParticleIndex == particleIndex
                    && contact.ShapeKind == shapeKind
                    && contact.ShapeIndex == shapeIndex)
                    return i;
            }

            return -1;
        }

        int FindSpan(int vertexBody, int triangleBody, int shapeKind, int i0, int i1, int i2) {
            if(spanMask < 0 || spanSlot.Length == 0) return -1;

            int slot = SpanHash(vertexBody, triangleBody, shapeKind, i0, i1, i2) & spanMask;
            for (int n = 0; n < spanSlot.Length; n++) {
                if(spanSeen[slot] != spanStamp) return -1;
                int index = spanSlot[slot];
                if(index >= 0 && index < contactCount) {
                    Contact contact = contacts[index];
                    if(contact.ShapeKind == shapeKind
                        && contact.BodyIndex == vertexBody
                        && contact.OtherBodyIndex == triangleBody
                        && contact.ParticleIndex == i0
                        && contact.OtherParticleIndex == i1
                        && contact.ThirdParticleIndex == i2)
                        return index;
                }

                slot = (slot + 1) & spanMask;
            }

            return -1;
        }

        int FindSurface(int vertexBody, int particleIndex, int triangleBody) {
            if(surfaceStride > 0) return Stamped(surfaceSlot, surfaceSeen, surfaceStride, vertexBody, particleIndex, triangleBody);

            for (int i = 0; i < contactCount; i++) {
                Contact contact = contacts[i];
                if(contact.ShapeKind != TriangleKind) continue;
                if(contact.BodyIndex == vertexBody
                    && contact.ParticleIndex == particleIndex
                    && contact.OtherBodyIndex == triangleBody)
                    return i;
            }

            return -1;
        }
    }
}
