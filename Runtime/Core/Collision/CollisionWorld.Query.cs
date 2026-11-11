using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;
using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Collision {
    public sealed partial class CollisionWorld {
        void PrepareLookup(IReadOnlyList<DeformableBody> bodies) {
            int count = bodies == null ? 0 : bodies.Count;
            trackedBodies = count;
            if(particleOffset.Length < count) particleOffset = new int[count];
            int cursor = 0;
            for (int i = 0; i < count; i++) {
                particleOffset[i] = cursor;
                Particle[] particles = bodies[i].Particles;
                cursor += particles == null ? 0 : particles.Length;
            }

            particleSpan = cursor;
            planeStride = planes.Count;
            boxStride = boxes.Count;
            surfaceStride = count;
            sphereStride = spheres.Count;
            capsuleStride = capsules.Count;
            orientedStride = orientedBoxes.Count;
            Grow(ref planeSlot, ref planeSeen, cursor * (planeStride > 0 ? planeStride : 1));
            Grow(ref boxSlot, ref boxSeen, cursor * (boxStride > 0 ? boxStride : 1));
            Grow(ref surfaceSlot, ref surfaceSeen, cursor * (surfaceStride > 0 ? surfaceStride : 1));
            Grow(ref sphereSlot, ref sphereSeen, cursor * (sphereStride > 0 ? sphereStride : 1));
            Grow(ref capsuleSlot, ref capsuleSeen, cursor * (capsuleStride > 0 ? capsuleStride : 1));
            Grow(ref orientedSlot, ref orientedSeen, cursor * (orientedStride > 0 ? orientedStride : 1));
            Grow(ref meshSlot, ref meshSeen, cursor * 2);
        }

        bool RefreshMesh(int bodyIndex, int particleIndex) {
            bool held = false;
            for (int slot = 0; slot < 2; slot++) {
                if(!MeshKey(bodyIndex, particleIndex, slot, out int key) || meshSeen[key] != lookupStamp) continue;
                int index = meshSlot[key];
                if(index < 0 || index >= contactCount) continue;
                Contact contact = contacts[index];
                if(contact.ShapeKind != MeshKind || contact.BodyIndex != bodyIndex || contact.ParticleIndex != particleIndex) continue;
                contacts[index].Seen = collectId;
                held = true;
            }

            return held;
        }

        int FindMesh(int bodyIndex, int particleIndex, int shapeIndex) {
            for (int slot = 0; slot < 2; slot++) {
                if(!MeshKey(bodyIndex, particleIndex, slot, out int key) || meshSeen[key] != lookupStamp) continue;
                int index = meshSlot[key];
                if(index < 0 || index >= contactCount) continue;
                if(contacts[index].ShapeIndex == shapeIndex) return index;
            }

            return -1;
        }

        bool MeshKey(int body, int particle, int slot, out int key) {
            key = 0;
            if(slot < 0 || slot > 1 || particleOffset == null || body < 0 || body >= trackedBodies || particle < 0) return false;
            int start = particleOffset[body];
            int count = body + 1 < trackedBodies ? particleOffset[body + 1] - start : particleSpan - start;
            if(particle >= count) return false;
            key = (start + particle) * 2 + slot;
            return key >= 0 && key < meshSlot.Length;
        }

        void RememberMesh(Contact contact, int index) {
            for (int slot = 0; slot < 2; slot++) {
                if(!MeshKey(contact.BodyIndex, contact.ParticleIndex, slot, out int key) || meshSeen[key] != lookupStamp) continue;
                int stored = meshSlot[key];
                if(stored < 0 || stored >= contactCount || contacts[stored].ShapeIndex != contact.ShapeIndex) continue;
                meshSlot[key] = index;
                return;
            }

            for (int slot = 0; slot < 2; slot++) {
                if(!MeshKey(contact.BodyIndex, contact.ParticleIndex, slot, out int key) || meshSeen[key] == lookupStamp) continue;
                meshSlot[key] = index;
                meshSeen[key] = lookupStamp;
                return;
            }
        }

        static bool MissesSphere(CollisionSphere sphere, Vec3 previous, Vec3 current) {
            float reach = sphere.Radius + 0.05f + Math.Abs(sphere.Motion.X) + Math.Abs(sphere.Motion.Y) + Math.Abs(sphere.Motion.Z);
            if(reach < 0.05f) reach = 0.05f;
            return OutsideBall(sphere.Center, reach, previous) && OutsideBall(sphere.Center, reach, current) && OutsideBall(sphere.Center, reach, ClosestPoint(previous, current, sphere.Center));
        }

        static bool MissesCapsule(CollisionCapsule capsule, Vec3 previous, Vec3 current) {
            float reach = capsule.Radius + 0.05f;
            float minX = Math.Min(capsule.Start.X, capsule.End.X) - reach;
            float minY = Math.Min(capsule.Start.Y, capsule.End.Y) - reach;
            float minZ = Math.Min(capsule.Start.Z, capsule.End.Z) - reach;
            float maxX = Math.Max(capsule.Start.X, capsule.End.X) + reach;
            float maxY = Math.Max(capsule.Start.Y, capsule.End.Y) + reach;
            float maxZ = Math.Max(capsule.Start.Z, capsule.End.Z) + reach;
            minX -= Math.Abs(capsule.Motion.X);
            minY -= Math.Abs(capsule.Motion.Y);
            minZ -= Math.Abs(capsule.Motion.Z);
            maxX += Math.Abs(capsule.Motion.X);
            maxY += Math.Abs(capsule.Motion.Y);
            maxZ += Math.Abs(capsule.Motion.Z);
            return Separated(previous, current, minX, minY, minZ, maxX, maxY, maxZ);
        }

        static bool MissesOriented(CollisionOrientedBox box, Vec3 previous, Vec3 current) {
            float reachX = Math.Abs(box.AxisX.X) + Math.Abs(box.AxisY.X) + Math.Abs(box.AxisZ.X) + Math.Abs(box.Motion.X) + 0.02f;
            float reachY = Math.Abs(box.AxisX.Y) + Math.Abs(box.AxisY.Y) + Math.Abs(box.AxisZ.Y) + Math.Abs(box.Motion.Y) + 0.02f;
            float reachZ = Math.Abs(box.AxisX.Z) + Math.Abs(box.AxisY.Z) + Math.Abs(box.AxisZ.Z) + Math.Abs(box.Motion.Z) + 0.02f;
            return Separated(previous, current, box.Center.X - reachX, box.Center.Y - reachY, box.Center.Z - reachZ, box.Center.X + reachX, box.Center.Y + reachY, box.Center.Z + reachZ);
        }

        static bool Separated(Vec3 previous, Vec3 current, float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
            if((previous.X < minX && current.X < minX) || (previous.X > maxX && current.X > maxX)) return true;
            if((previous.Y < minY && current.Y < minY) || (previous.Y > maxY && current.Y > maxY)) return true;
            if((previous.Z < minZ && current.Z < minZ) || (previous.Z > maxZ && current.Z > maxZ)) return true;
            return false;
        }

        static bool OutsideBall(Vec3 center, float reach, Vec3 point) {
            float dx = point.X - center.X;
            float dy = point.Y - center.Y;
            float dz = point.Z - center.Z;
            return dx * dx + dy * dy + dz * dz > reach * reach;
        }

        static void Grow(ref int[] slots, ref int[] seen, int size) {
            if(size <= 0 || (slots != null && slots.Length >= size)) return;
            slots = new int[size];
            seen = new int[size];
        }

        int Stamped(int[] slots, int[] seen, int stride, int body, int particle, int shape) {
            if(!TryKey(body, particle, shape, stride, out int key) || seen[key] != lookupStamp) return -1;
            int index = slots[key];
            return index >= 0 && index < contactCount ? index : -1;
        }

        void Remember(Contact contact, int index) {
            int[] slots;
            int[] seen;
            int stride;
            int shape;
            if(contact.ShapeKind == PlaneKind) {
                slots = planeSlot;
                seen = planeSeen;
                stride = planeStride;
                shape = contact.ShapeIndex;
            } else if(contact.ShapeKind == BoxKind) {
                slots = boxSlot;
                seen = boxSeen;
                stride = boxStride;
                shape = contact.ShapeIndex;
            } else if(contact.ShapeKind == SphereKind) {
                slots = sphereSlot;
                seen = sphereSeen;
                stride = sphereStride;
                shape = contact.ShapeIndex;
            } else if(contact.ShapeKind == CapsuleKind) {
                slots = capsuleSlot;
                seen = capsuleSeen;
                stride = capsuleStride;
                shape = contact.ShapeIndex;
            } else if(contact.ShapeKind == OrientedKind) {
                slots = orientedSlot;
                seen = orientedSeen;
                stride = orientedStride;
                shape = contact.ShapeIndex;
            } else if(contact.ShapeKind == MeshKind) {
                RememberMesh(contact, index);
                return;
            } else if(contact.ShapeKind == TriangleKind) {
                slots = surfaceSlot;
                seen = surfaceSeen;
                stride = surfaceStride;
                shape = contact.OtherBodyIndex;
            } else
                return;

            if(slots == null || !TryKey(contact.BodyIndex, contact.ParticleIndex, shape, stride, out int key)) return;
            slots[key] = index;
            seen[key] = lookupStamp;
        }

        bool TryKey(int body, int particle, int shape, int stride, out int key) {
            key = 0;
            if(stride <= 0 || particleOffset == null || body < 0 || body >= trackedBodies || particle < 0 || shape < 0 || shape >= stride)
                return false;
            int start = particleOffset[body];
            int count = body + 1 < trackedBodies ? particleOffset[body + 1] - start : particleSpan - start;
            if(particle >= count) return false;
            key = (start + particle) * stride + shape;
            return true;
        }

        bool Misses(int triangleBody, Vec3 previous, Vec3 point) {
            if(triangleBody < 0 || triangleBody >= bodyBounds.Length) return false;
            BodyBounds box = bodyBounds[triangleBody];
            return box.Valid && Separated(previous, point, box.MinX, box.MinY, box.MinZ, box.MaxX, box.MaxY, box.MaxZ);
        }

        bool MissesEdge(int triangleBody, Particle a, Particle b) {
            if(triangleBody < 0 || triangleBody >= bodyBounds.Length) return false;
            BodyBounds box = bodyBounds[triangleBody];
            return box.Valid
                && Separated(a.Previous, a.Position, box.MinX, box.MinY, box.MinZ, box.MaxX, box.MaxY, box.MaxZ)
                && Separated(b.Previous, b.Position, box.MinX, box.MinY, box.MinZ, box.MaxX, box.MaxY, box.MaxZ)
                && Separated(a.Position, b.Position, box.MinX, box.MinY, box.MinZ, box.MaxX, box.MaxY, box.MaxZ)
                && Separated(a.Previous, b.Previous, box.MinX, box.MinY, box.MinZ, box.MaxX, box.MaxY, box.MaxZ);
        }

        bool MissesFace(int triangleBody, Particle a, Particle b, Particle c) {
            if(triangleBody < 0 || triangleBody >= bodyBounds.Length) return false;
            BodyBounds box = bodyBounds[triangleBody];
            return box.Valid
                && Separated(a.Previous, a.Position, box.MinX, box.MinY, box.MinZ, box.MaxX, box.MaxY, box.MaxZ)
                && Separated(b.Previous, b.Position, box.MinX, box.MinY, box.MinZ, box.MaxX, box.MaxY, box.MaxZ)
                && Separated(c.Previous, c.Position, box.MinX, box.MinY, box.MinZ, box.MaxX, box.MaxY, box.MaxZ)
                && Separated(a.Position, b.Position, box.MinX, box.MinY, box.MinZ, box.MaxX, box.MaxY, box.MaxZ)
                && Separated(b.Position, c.Position, box.MinX, box.MinY, box.MinZ, box.MaxX, box.MaxY, box.MaxZ)
                && Separated(c.Position, a.Position, box.MinX, box.MinY, box.MinZ, box.MaxX, box.MaxY, box.MaxZ)
                && Separated(a.Previous, b.Previous, box.MinX, box.MinY, box.MinZ, box.MaxX, box.MaxY, box.MaxZ)
                && Separated(b.Previous, c.Previous, box.MinX, box.MinY, box.MinZ, box.MaxX, box.MaxY, box.MaxZ)
                && Separated(c.Previous, a.Previous, box.MinX, box.MinY, box.MinZ, box.MaxX, box.MaxY, box.MaxZ);
        }

        void RefreshSupported(IReadOnlyList<DeformableBody> bodies) {
            if(bodies == null) return;
            int count = bodies.Count;
            if(supported.Length < count) supported = new bool[count];
            for (int i = 0; i < count; i++) supported[i] = false;
            for (int i = 0; i < contactCount; i++) {
                Contact contact = contacts[i];
                if(contact.Seen != collectId) continue;
                if(contact.ShapeKind != PlaneKind && contact.ShapeKind != BoxKind
                    && contact.ShapeKind != SphereKind && contact.ShapeKind != CapsuleKind
                    && contact.ShapeKind != OrientedKind && contact.ShapeKind != MeshKind)
                    continue;
                if(contact.BodyIndex >= 0 && contact.BodyIndex < count) supported[contact.BodyIndex] = true;
            }
        }

        bool SkipsStuck(Contact contact, IReadOnlyList<DeformableBody> bodies) {
            if(contact.Contained == 0 || contact.OtherBodyIndex < 0) return false;
            int mine = contact.BodyIndex;
            int other = contact.OtherBodyIndex;
            if(mine < 0 || other < 0 || mine >= bodies.Count || other >= bodies.Count) return false;
            if(InsideOf(mine, other) <= 0 || InsideOf(other, mine) <= 0) return false;
            if(IsHost(mine, other)) return true;
            if(contact.ParticleIndex < 0 || contact.ParticleIndex >= bodies[mine].Particles.Length) return true;
            Vec3 position = bodies[mine].Particles[contact.ParticleIndex].Position;
            return Vec3.Dot(contact.Normal, position - bodyCenter[other]) <= 0f;
        }

        bool IsHost(int body, int other) {
            bool bodyGrounded = body >= 0 && body < supported.Length && supported[body];
            bool otherGrounded = other >= 0 && other < supported.Length && supported[other];
            if(bodyGrounded && !otherGrounded) return true;
            if(otherGrounded && !bodyGrounded) return false;
            int mineInside = InsideOf(body, other);
            int otherInside = InsideOf(other, body);
            if(mineInside < otherInside) return true;
            if(mineInside > otherInside) return false;
            return body < other;
        }

        static bool Covered(int[] cover, int token, int index) {
            return cover != null && index >= 0 && index < cover.Length && cover[index] == token;
        }

        int[] CoverRow(int body, int particles) {
            if(coverRows.Length < trackedBodies) {
                var grown = new int[trackedBodies < 1 ? 1 : trackedBodies][];
                for (int i = 0; i < coverRows.Length && i < grown.Length; i++) grown[i] = coverRows[i];
                coverRows = grown;
            }

            if(body < 0 || body >= coverRows.Length) return vertexCover;

            int[] row = coverRows[body];
            if(row == null || row.Length < particles) {
                row = new int[particles < 1 ? 1 : particles];
                coverRows[body] = row;
            }

            return row;
        }

        void SaveExisting(SurfaceBatch batch, int existing, Contact contact) {
            if(batch == null) contacts[existing] = contact;
            else {
                batch.EditIndex.Add(existing);
                batch.Edited.Add(contact);
            }
        }

        void SaveNew(SurfaceBatch batch, Contact contact) {
            if(batch == null) Add(contact);
            else {
                contact.Seen = collectId;
                batch.Created.Add(contact);
            }
        }

        void RememberSeedLayout() {
            int bodies = trackedBodies < 1 ? 1 : trackedBodies;
            int particles = particleSpan < 0 ? 0 : particleSpan;
            int size = particles * bodies;
            if(size < 1) size = 1;
            if(faceSeeds.Length < size) {
                int old = faceSeeds.Length;
                Array.Resize(ref faceSeeds, size);
                for (int i = old; i < faceSeeds.Length; i++) faceSeeds[i] = -1;
            }

            if(seedBodyCount == bodies && seedParticleCount == particles) return;
            seedBodyCount = bodies;
            seedParticleCount = particles;
            for (int i = 0; i < faceSeeds.Length; i++) faceSeeds[i] = -1;
        }

        void BuildFaceNeighbors(IReadOnlyList<DeformableBody> bodies, int planeCount) {
            if(builtFaceCount.Length != bodies.Count) builtFaceCount = new int[bodies.Count];
            bool unchanged = neighborStart.Length >= planeCount && planeCount > 0;
            if(unchanged) {
                for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++) {
                    if(builtFaceCount[bodyIndex] != faceLength[bodyIndex]) unchanged = false;
                }
            }

            if(unchanged) return;

            for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++) builtFaceCount[bodyIndex] = faceLength[bodyIndex];

            if(neighborStart.Length < planeCount) {
                neighborStart = new int[planeCount < 16 ? 16 : planeCount];
                neighborCount = new int[neighborStart.Length];
            }

            int listRoom = planeCount * 16;
            if(listRoom < 16) listRoom = 16;
            if(faceNeighbor.Length < listRoom) faceNeighbor = new int[listRoom];

            if(faceWide.Length < bodies.Count) faceWide = new bool[bodies.Count];

            const int cap = 16;
            int cursor = 0;
            for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++) {
                faceWide[bodyIndex] = false;
                int origin = faceOrigin[bodyIndex];
                int length = faceLength[bodyIndex];
                int[] mesh = bodies[bodyIndex].Triangles;
                for (int face = 0; face < length; face++) {
                    int plane = origin + face;
                    neighborStart[plane] = cursor;
                    int written = 0;
                    if(mesh != null) {
                        int slot = facePlanes[plane].Slot;
                        for (int other = 0; other < length; other++) {
                            if(other == face) continue;
                            int otherSlot = facePlanes[origin + other].Slot;
                            if(!SharesVertex(mesh, slot, otherSlot)) continue;
                            if(written >= cap || cursor >= faceNeighbor.Length) {
                                faceWide[bodyIndex] = true;
                                continue;
                            }

                            faceNeighbor[cursor++] = other;
                            written++;
                        }
                    }

                    neighborCount[plane] = written;
                }
            }
        }

        static bool SharesVertex(int[] mesh, int slot, int otherSlot) {
            if(slot < 0 || otherSlot < 0 || slot + 2 >= mesh.Length || otherSlot + 2 >= mesh.Length) return false;
            int a0 = mesh[slot];
            int a1 = mesh[slot + 1];
            int a2 = mesh[slot + 2];
            int b0 = mesh[otherSlot];
            int b1 = mesh[otherSlot + 1];
            int b2 = mesh[otherSlot + 2];
            return a0 == b0 || a0 == b1 || a0 == b2
                || a1 == b0 || a1 == b1 || a1 == b2
                || a2 == b0 || a2 == b1 || a2 == b2;
        }

        bool QueryClosed(int host, Particle[] surface, int[] triangles, Vec3 previous, Vec3 point, int guest, int guestParticle, out int triangle, out Vec3 normal, out float distance, out float weightA, out float weightB, out float weightC) {
            triangle = -1;
            normal = Vec3.Zero;
            distance = 0f;
            weightA = weightB = weightC = 0f;
            if(surface == null || triangles == null || host < 0 || host >= faceLength.Length || faceLength[host] <= 0) return false;

            int seedKey = SeedKey(guest, guestParticle, host);
            if(MissesReach(host, previous, point)) return false;

            if(!bodyRound[host] || host >= faceWide.Length || faceWide[host] || neighborStart.Length <= faceOrigin[host]) {
                if((host >= bodyConvex.Length || !bodyConvex[host])
                    && QueryBin(host, surface, triangles, previous, point, out triangle, out normal, out distance, out weightA, out weightB, out weightC))
                    return distance < 0f;

                return FullQuery(host, surface, triangles, previous, point, seedKey, out triangle, out normal, out distance, out weightA, out weightB, out weightC);
            }

            float allowance = bodyReach[host] * 0.35f;
            if(allowance < 0.02f) allowance = 0.02f;
            if(allowance > 0.12f) allowance = 0.12f;
            float movedX = point.X - previous.X;
            float movedY = point.Y - previous.Y;
            float movedZ = point.Z - previous.Z;
            bool leapt = movedX * movedX + movedY * movedY + movedZ * movedZ > allowance * allowance;
            if(leapt || seedKey < 0 || faceSeeds[seedKey] < 0 || faceSeeds[seedKey] >= faceLength[host])
                return FullQuery(host, surface, triangles, previous, point, seedKey, out triangle, out normal, out distance, out weightA, out weightB, out weightC);

            int origin = faceOrigin[host];
            int best = faceSeeds[seedKey];
            if(neighborCount[origin + best] <= 0 && faceLength[host] > 1)
                return FullQuery(host, surface, triangles, previous, point, seedKey, out triangle, out normal, out distance, out weightA, out weightB, out weightC);

            float bestDistance = PlaneDistance(origin + best, point);
            int guard = 0;
            bool climbed = true;
            while (climbed && guard < 6) {
                climbed = false;
                guard++;
                int start = neighborStart[origin + best];
                int count = neighborCount[origin + best];
                for (int i = 0; i < count; i++) {
                    int face = faceNeighbor[start + i];
                    float next = PlaneDistance(origin + face, point);
                    if(next <= bestDistance) continue;
                    bestDistance = next;
                    best = face;
                    climbed = true;
                }
            }

            if(guard >= 6 && climbed)
                return FullQuery(host, surface, triangles, previous, point, seedKey, out triangle, out normal, out distance, out weightA, out weightB, out weightC);
            if(NeighborhoodStraddles(origin, best, previous, point))
                return FullQuery(host, surface, triangles, previous, point, seedKey, out triangle, out normal, out distance, out weightA, out weightB, out weightC);

            if(bestDistance > 1e-5f) {
                if(OutsideReach(host, point) && OutsideReach(host, previous)) return false;
                if(OutsideHull(surface, bodyCenter[host], point) && OutsideHull(surface, bodyCenter[host], previous)) return false;
                return FullQuery(host, surface, triangles, previous, point, seedKey, out triangle, out normal, out distance, out weightA, out weightB, out weightC);
            }

            if(!ContactSolver.TryProjectedFace(facePlanes[origin + best], bodyRound[host], surface, triangles, point, bestDistance, out triangle, out normal, out distance, out weightA, out weightB, out weightC))
                return FullQuery(host, surface, triangles, previous, point, seedKey, out triangle, out normal, out distance, out weightA, out weightB, out weightC);

            faceSeeds[seedKey] = best;
            return true;
        }

        void BuildFaceBins(IReadOnlyList<DeformableBody> bodies) {
            int count = bodies.Count;
            int cells = count * BinCells;
            if(cells < BinCells) cells = BinCells;
            if(faceBinCount.Length < cells) {
                faceBinCount = new int[cells];
                faceBinStart = new int[cells];
            }

            if(binCell.Length < count) {
                binCell = new float[count];
                binOriginX = new float[count];
                binOriginY = new float[count];
                binOriginZ = new float[count];
            }

            Array.Clear(faceBinCount, 0, count * BinCells);
            int listSize = 0;
            for (int bodyIndex = 0; bodyIndex < count; bodyIndex++) {
                binCell[bodyIndex] = 0f;
                if(bodyIndex < bodyConvex.Length && bodyConvex[bodyIndex]) continue;
                if(!bodies[bodyIndex].ClosedSurface || faceLength[bodyIndex] <= 0) continue;
                FitBin(bodyIndex, bodies[bodyIndex].Particles);
                if(binCell[bodyIndex] <= 1e-5f) continue;
                listSize += CountBinFaces(bodyIndex, bodies[bodyIndex].Particles, bodies[bodyIndex].Triangles);
            }

            if(listSize < 1) listSize = 1;
            if(faceBinList.Length < listSize) faceBinList = new int[listSize];

            int cursor = 0;
            for (int bodyIndex = 0; bodyIndex < count; bodyIndex++) {
                int origin = bodyIndex * BinCells;
                for (int cell = 0; cell < BinCells; cell++) {
                    faceBinStart[origin + cell] = cursor;
                    cursor += faceBinCount[origin + cell];
                    faceBinCount[origin + cell] = 0;
                }
            }

            for (int bodyIndex = 0; bodyIndex < count; bodyIndex++) {
                if(binCell[bodyIndex] <= 1e-5f) continue;
                FillBinFaces(bodyIndex, bodies[bodyIndex].Particles, bodies[bodyIndex].Triangles);
            }
        }

        void FitBin(int bodyIndex, Particle[] particles) {
            if(particles == null || particles.Length == 0 || !bodyBounds[bodyIndex].Valid) return;
            BodyBounds bounds = bodyBounds[bodyIndex];
            float spanX = bounds.MaxX - bounds.MinX;
            float spanY = bounds.MaxY - bounds.MinY;
            float spanZ = bounds.MaxZ - bounds.MinZ;
            float span = spanX > spanY ? spanX : spanY;
            if(spanZ > span) span = spanZ;
            float cell = span / BinAxis;
            if(cell < 0.07f) cell = 0.07f;
            binCell[bodyIndex] = cell;
            binOriginX[bodyIndex] = bounds.MinX;
            binOriginY[bodyIndex] = bounds.MinY;
            binOriginZ[bodyIndex] = bounds.MinZ;
        }

        int CountBinFaces(int bodyIndex, Particle[] surface, int[] triangles) {
            return VisitBinFaces(bodyIndex, surface, triangles, false);
        }

        void FillBinFaces(int bodyIndex, Particle[] surface, int[] triangles) { VisitBinFaces(bodyIndex, surface, triangles, true); }

        int VisitBinFaces(int bodyIndex, Particle[] surface, int[] triangles, bool fill) {
            if(surface == null || triangles == null) return 0;
            int added = 0;
            int origin = faceOrigin[bodyIndex];
            int count = faceLength[bodyIndex];
            for (int plane = 0; plane < count; plane++) {
                int slot = facePlanes[origin + plane].Slot;
                if(slot < 0 || slot + 2 >= triangles.Length) continue;
                int i0 = triangles[slot];
                int i1 = triangles[slot + 1];
                int i2 = triangles[slot + 2];
                if(i0 < 0 || i1 < 0 || i2 < 0 || i0 >= surface.Length || i1 >= surface.Length || i2 >= surface.Length) continue;
                Vec3 a = surface[i0].Position;
                Vec3 b = surface[i1].Position;
                Vec3 c = surface[i2].Position;
                const float pad = 0.05f;
                float minX = a.X < b.X ? a.X : b.X;
                if(c.X < minX) minX = c.X;
                float minY = a.Y < b.Y ? a.Y : b.Y;
                if(c.Y < minY) minY = c.Y;
                float minZ = a.Z < b.Z ? a.Z : b.Z;
                if(c.Z < minZ) minZ = c.Z;
                float maxX = a.X > b.X ? a.X : b.X;
                if(c.X > maxX) maxX = c.X;
                float maxY = a.Y > b.Y ? a.Y : b.Y;
                if(c.Y > maxY) maxY = c.Y;
                float maxZ = a.Z > b.Z ? a.Z : b.Z;
                if(c.Z > maxZ) maxZ = c.Z;
                int x0 = BinIndex(minX - pad, binOriginX[bodyIndex], binCell[bodyIndex]);
                int x1 = BinIndex(maxX + pad, binOriginX[bodyIndex], binCell[bodyIndex]);
                int y0 = BinIndex(minY - pad, binOriginY[bodyIndex], binCell[bodyIndex]);
                int y1 = BinIndex(maxY + pad, binOriginY[bodyIndex], binCell[bodyIndex]);
                int z0 = BinIndex(minZ - pad, binOriginZ[bodyIndex], binCell[bodyIndex]);
                int z1 = BinIndex(maxZ + pad, binOriginZ[bodyIndex], binCell[bodyIndex]);
                int baseCell = bodyIndex * BinCells;
                for (int z = z0; z <= z1; z++) {
                    for (int y = y0; y <= y1; y++) {
                        for (int x = x0; x <= x1; x++) {
                            int cell = baseCell + (z * BinAxis + y) * BinAxis + x;
                            if(!fill) {
                                faceBinCount[cell]++;
                                added++;
                                continue;
                            }

                            int slotInList = faceBinStart[cell] + faceBinCount[cell];
                            faceBinList[slotInList] = origin + plane;
                            faceBinCount[cell]++;
                        }
                    }
                }
            }

            return added;
        }

        static int BinIndex(float value, float origin, float cell) {
            int index = (int)((value - origin) / cell);
            if(index < 0) return 0;
            if(index >= BinAxis) return BinAxis - 1;
            return index;
        }

        bool QueryBin(int host, Particle[] surface, int[] triangles, Vec3 previous, Vec3 point, out int triangle, out Vec3 normal, out float distance, out float weightA, out float weightB, out float weightC) {
            triangle = -1;
            normal = Vec3.Zero;
            distance = 1f;
            weightA = weightB = weightC = 0f;
            if(host < 0 || host >= binCell.Length || binCell[host] <= 1e-5f) {
                BinUnsure++;
                return false;
            }
            float travelX = point.X - previous.X;
            float travelY = point.Y - previous.Y;
            float travelZ = point.Z - previous.Z;
            if(travelX * travelX + travelY * travelY + travelZ * travelZ > 0.04f) {
                BinUnsure++;
                return false;
            }
            float cell = binCell[host];

            int x = BinIndex(point.X, binOriginX[host], cell);
            int y = BinIndex(point.Y, binOriginY[host], cell);
            int z = BinIndex(point.Z, binOriginZ[host], cell);
            int bucket = host * BinCells + (z * BinAxis + y) * BinAxis + x;
            if(bucket < 0 || bucket >= faceBinCount.Length || faceBinCount[bucket] <= 0) {
                BinUnsure++;
                return false;
            }

            bool hit = false;
            bool sawBehind = false;
            float hitDistance = float.NegativeInfinity;
            float nearestFront = float.MaxValue;
            int start = faceBinStart[bucket];
            int count = faceBinCount[bucket];
            for (int i = 0; i < count; i++) {
                int planeIndex = faceBinList[start + i];
                ContactSolver.FacePlane plane = facePlanes[planeIndex];
                float planeDistance = plane.Normal.X * (point.X - plane.Point.X)
                    + plane.Normal.Y * (point.Y - plane.Point.Y)
                    + plane.Normal.Z * (point.Z - plane.Point.Z);
                if(planeDistance > 1e-5f) {
                    if(planeDistance < nearestFront) nearestFront = planeDistance;
                    continue;
                }

                if(planeDistance >= -1e-8f) continue;
                sawBehind = true;
                if(!ContactSolver.TryProjectedFace(plane, false, surface, triangles, point, planeDistance, out int face, out Vec3 faceNormal, out float faceDistance, out float faceWeightA, out float faceWeightB, out float faceWeightC))
                    continue;
                if(hit && planeDistance <= hitDistance) continue;
                hit = true;
                hitDistance = planeDistance;
                triangle = face;
                normal = faceNormal;
                distance = faceDistance;
                weightA = faceWeightA;
                weightB = faceWeightB;
                weightC = faceWeightC;
            }

            if(hit) {
                BinHits++;
                return true;
            }

            if(!sawBehind && nearestFront < 0.08f) {
                BinMisses++;
                return true;
            }

            BinUnsure++;
            return false;
        }

        bool FullQuery(int host, Particle[] surface, int[] triangles, Vec3 previous, Vec3 point, int seedKey, out int triangle, out Vec3 normal, out float distance, out float weightA, out float weightB, out float weightC) {
            FullQueries++;
            bool convex = bodyRound[host] || (host < bodyConvex.Length && bodyConvex[host]);
            bool hit = ContactSolver.TryConvexInterior(facePlanes, faceOrigin[host], faceLength[host], convex, surface, triangles, previous, point, out triangle, out normal, out distance, out weightA, out weightB, out weightC);
            if(seedKey < 0) return hit;
            if(hit) RememberSeed(seedKey, host, triangle);
            else faceSeeds[seedKey] = -2;
            return hit;
        }

        void RememberSeed(int seedKey, int host, int triangle) {
            if(seedKey < 0) return;
            int slot = triangle * 3;
            int origin = faceOrigin[host];
            int count = faceLength[host];
            for (int i = 0; i < count; i++) {
                if(facePlanes[origin + i].Slot != slot) continue;
                faceSeeds[seedKey] = i;
                return;
            }
        }

        int SeedKey(int guest, int guestParticle, int host) {
            if(guestParticle < 0 || guest < 0 || host < 0 || trackedBodies <= 0 || guest >= trackedBodies) return -1;
            if(guest >= particleOffset.Length) return -1;
            int key = (particleOffset[guest] + guestParticle) * trackedBodies + host;
            if(key < 0 || key >= faceSeeds.Length) return -1;
            return key;
        }

        bool EdgeMissesReach(int host, Particle a, Particle b) {
            if(!OutsideReach(host, a.Position) || !OutsideReach(host, b.Position)) return false;
            if(!OutsideReach(host, a.Previous) || !OutsideReach(host, b.Previous)) return false;
            Vec3 center = bodyCenter[host];
            return OutsideReach(host, ClosestPoint(a.Position, b.Position, center))
                && OutsideReach(host, ClosestPoint(a.Previous, b.Previous, center));
        }

        bool MissesReach(int host, Vec3 previous, Vec3 point) {
            if(!OutsideReach(host, previous) || !OutsideReach(host, point)) return false;
            return OutsideReach(host, ClosestPoint(previous, point, bodyCenter[host]));
        }

        bool OutsideReach(int host, Vec3 point) {
            float reach = bodyReach[host];
            if(reach <= 0f) return false;
            float dx = point.X - bodyCenter[host].X;
            float dy = point.Y - bodyCenter[host].Y;
            float dz = point.Z - bodyCenter[host].Z;
            return dx * dx + dy * dy + dz * dz > reach * reach;
        }

        static Vec3 ClosestPoint(Vec3 from, Vec3 to, Vec3 center) {
            float ax = to.X - from.X;
            float ay = to.Y - from.Y;
            float az = to.Z - from.Z;
            float length = ax * ax + ay * ay + az * az;
            if(length < 1e-12f) return from;
            float t = ((center.X - from.X) * ax + (center.Y - from.Y) * ay + (center.Z - from.Z) * az) / length;
            if(t < 0f) t = 0f;
            if(t > 1f) t = 1f;
            return new Vec3(from.X + ax * t, from.Y + ay * t, from.Z + az * t);
        }

        static bool OutsideHull(Particle[] surface, Vec3 center, Vec3 point) {
            if(surface == null || surface.Length == 0) return false;
            float dx = point.X - center.X;
            float dy = point.Y - center.Y;
            float dz = point.Z - center.Z;
            float limit = dx * dx + dy * dy + dz * dz;
            if(limit < 1e-10f) return false;
            float best = float.NegativeInfinity;
            for (int i = 0; i < surface.Length; i++) {
                Vec3 position = surface[i].Position;
                float score = (position.X - center.X) * dx + (position.Y - center.Y) * dy + (position.Z - center.Z) * dz;
                if(score > best) best = score;
            }

            return best < limit - 1e-5f;
        }

        float PlaneDistance(int planeIndex, Vec3 point) {
            ContactSolver.FacePlane plane = facePlanes[planeIndex];
            return plane.Normal.X * (point.X - plane.Point.X)
                + plane.Normal.Y * (point.Y - plane.Point.Y)
                + plane.Normal.Z * (point.Z - plane.Point.Z);
        }

        bool NeighborhoodStraddles(int origin, int best, Vec3 previous, Vec3 point) {
            if(Straddles(origin + best, previous, point)) return true;
            int start = neighborStart[origin + best];
            int count = neighborCount[origin + best];
            for (int i = 0; i < count; i++) { if(Straddles(origin + faceNeighbor[start + i], previous, point)) return true; }

            return false;
        }

        bool Straddles(int planeIndex, Vec3 previous, Vec3 point) {
            ContactSolver.FacePlane plane = facePlanes[planeIndex];
            float normalX = plane.Normal.X;
            float normalY = plane.Normal.Y;
            float normalZ = plane.Normal.Z;
            float behind = normalX * (point.X - plane.Point.X) + normalY * (point.Y - plane.Point.Y) + normalZ * (point.Z - plane.Point.Z);
            if(behind >= -1e-8f) return false;
            float started = normalX * (previous.X - plane.Point.X) + normalY * (previous.Y - plane.Point.Y) + normalZ * (previous.Z - plane.Point.Z);
            return started > 1e-5f;
        }

        void Add(Contact contact) {
            if(contactCount == contacts.Length) Array.Resize(ref contacts, contacts.Length * 2);
            contact.Seen = collectId;
            int index = contactCount;
            contacts[contactCount++] = contact;
            Remember(contact, index);
        }

        sealed class SurfaceBatch {
            public readonly List<Contact> Created = new List<Contact>(64);
            public readonly List<int> EditIndex = new List<int>(32);
            public readonly List<Contact> Edited = new List<Contact>(32);

            public void Clear() {
                Created.Clear();
                EditIndex.Clear();
                Edited.Clear();
            }

            public void Commit(CollisionWorld world) {
                for (int i = 0; i < EditIndex.Count; i++) world.contacts[EditIndex[i]] = Edited[i];
                for (int i = 0; i < Created.Count; i++) world.Add(Created[i]);
            }
        }

        struct BodyBounds {
            public float MinX;
            public float MinY;
            public float MinZ;
            public float MaxX;
            public float MaxY;
            public float MaxZ;
            public bool Valid;
        }
    }
}
