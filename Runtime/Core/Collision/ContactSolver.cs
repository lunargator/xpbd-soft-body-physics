using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Collision {
    public static class ContactSolver {
        public static void Solve(ref Contact contact, Particle[] particles, float stepSeconds) {
            if(stepSeconds <= 0f || contact.ParticleIndex < 0 || contact.ParticleIndex >= particles.Length) return;

            Particle particle = particles[contact.ParticleIndex];
            if(particle.InverseMass <= 0f) return;

            float separation = Vec3.Dot(contact.Normal, particle.Position) - contact.Offset;
            if(separation < 0f) {
                float deltaLambda = -separation / particle.InverseMass;
                float updated = contact.Lambda + deltaLambda;
                if(updated < 0f) deltaLambda = -contact.Lambda;
                contact.Lambda += deltaLambda;
                particle.Position += contact.Normal * (particle.InverseMass * deltaLambda);
            }

            if(contact.Lambda > 0f && contact.Friction > 0f) {
                Vec3 tangential = particle.Position - contact.Anchor;
                float alongNormal = Vec3.Dot(tangential, contact.Normal);
                tangential -= contact.Normal * alongNormal;
                float length = tangential.Length;
                if(length > 1e-8f) {
                    float limit = contact.Friction * particle.InverseMass * contact.Lambda;
                    float correction = length < limit ? length : limit;
                    particle.Position -= tangential * (correction / length);
                }
            }

            particles[contact.ParticleIndex] = particle;
        }

        public static bool TryNearestSurface(Particle[] surface, int[] triangles, Vec3 point, out int triangle, out Vec3 normal, out float distance, out float weightA, out float weightB, out float weightC) {
            triangle = -1;
            normal = Vec3.Zero;
            distance = 0f;
            weightA = weightB = weightC = 0f;
            if(surface == null || triangles == null) return false;

            float bestAbsolute = float.MaxValue;
            bool found = false;
            for (int slot = 0; slot + 2 < triangles.Length; slot += 3) {
                int i0 = triangles[slot];
                int i1 = triangles[slot + 1];
                int i2 = triangles[slot + 2];
                if(i0 < 0 || i1 < 0 || i2 < 0 || i0 >= surface.Length || i1 >= surface.Length || i2 >= surface.Length) continue;
                if(!TryProject(surface[i0].Position, surface[i1].Position, surface[i2].Position, point, out Vec3 faceNormal, out float faceDistance, out float faceWeightA, out float faceWeightB, out float faceWeightC))
                    continue;

                float absolute = faceDistance < 0f ? -faceDistance : faceDistance;
                if(absolute >= bestAbsolute) continue;
                bestAbsolute = absolute;
                triangle = slot / 3;
                normal = faceNormal;
                distance = faceDistance;
                weightA = faceWeightA;
                weightB = faceWeightB;
                weightC = faceWeightC;
                found = true;
            }

            return found && distance < -1e-8f;
        }

        public struct FacePlane {
            public Vec3 Normal;
            public Vec3 Point;
            public int Slot;
        }

        public static int WriteFacePlanes(Particle[] surface, int[] triangles, FacePlane[] destination, int destinationIndex) {
            if(surface == null || triangles == null || destination == null) return destinationIndex;

            Vec3 center = Vec3.Zero;
            for (int i = 0; i < surface.Length; i++) center += surface[i].Position;
            if(surface.Length > 0) center *= 1f / surface.Length;

            for (int index = 0; index + 2 < triangles.Length; index += 3) {
                int i0 = triangles[index];
                int i1 = triangles[index + 1];
                int i2 = triangles[index + 2];
                if(i0 < 0 || i1 < 0 || i2 < 0 || i0 >= surface.Length || i1 >= surface.Length || i2 >= surface.Length) continue;
                Vec3 a = surface[i0].Position;
                Vec3 b = surface[i1].Position;
                Vec3 c = surface[i2].Position;
                if(!TryFacePlane(a, b, c, a, out Vec3 normal, out _)) continue;
                if(Vec3.Dot(normal, a - center) <= 1e-5f) continue;
                if(destinationIndex >= destination.Length) return destinationIndex;
                destination[destinationIndex++] = new FacePlane {
                    Normal = normal,
                    Point = a,
                    Slot = index
                };
            }

            return destinationIndex;
        }

        public static bool TryConvexInterior(Particle[] surface, int[] triangles, Vec3 previous, Vec3 point, out int triangle, out Vec3 normal, out float distance, out float weightA, out float weightB, out float weightC) {
            triangle = -1;
            normal = Vec3.Zero;
            distance = 0f;
            weightA = weightB = weightC = 0f;
            if(surface == null || triangles == null) return false;

            var planes = new FacePlane[triangles.Length / 3];
            int count = WriteFacePlanes(surface, triangles, planes, 0);
            return TryConvexInterior(planes, 0, count, true, surface, triangles, previous, point, out triangle, out normal, out distance, out weightA, out weightB, out weightC);
        }

        public static bool TryConvexInterior(FacePlane[] planes, int planeStart, int planeCount, bool convex, Particle[] surface, int[] triangles, Vec3 previous, Vec3 point, out int triangle, out Vec3 normal, out float distance, out float weightA, out float weightB, out float weightC) {
            triangle = -1;
            normal = Vec3.Zero;
            distance = 0f;
            weightA = weightB = weightC = 0f;
            if(planes == null || surface == null || triangles == null || planeCount <= 0) return false;

            const float crack = 0.002f;
            float entryT = float.MaxValue;
            int entrySlot = -1;
            Vec3 entryNormal = Vec3.Zero;
            float entryDistance = 0f;
            float entryWeightA = 0f;
            float entryWeightB = 0f;
            float entryWeightC = 0f;
            float nearest = float.NegativeInfinity;
            int nearestSlot = -1;
            Vec3 nearestNormal = Vec3.Zero;
            bool outsideHull = false;
            int planeEnd = planeStart + planeCount;
            if(planeEnd > planes.Length) planeEnd = planes.Length;

            for (int planeIndex = planeStart; planeIndex < planeEnd; planeIndex++) {
                FacePlane plane = planes[planeIndex];
                int index = plane.Slot;
                if(index < 0 || index + 2 >= triangles.Length) continue;
                float normalX = plane.Normal.X;
                float normalY = plane.Normal.Y;
                float normalZ = plane.Normal.Z;
                float planeDistance = normalX * (point.X - plane.Point.X) + normalY * (point.Y - plane.Point.Y) + normalZ * (point.Z - plane.Point.Z);
                if(planeDistance > nearest) {
                    nearest = planeDistance;
                    nearestSlot = index;
                    nearestNormal = plane.Normal;
                }

                if(planeDistance > 1e-5f) outsideHull = true;

                float started = normalX * (previous.X - plane.Point.X) + normalY * (previous.Y - plane.Point.Y) + normalZ * (previous.Z - plane.Point.Z);
                if(started > 1e-5f && planeDistance > 1e-5f) {
                    if(convex && started > 0.06f && planeDistance > 0.06f) return false;
                    if(!convex) {
                        float stepX = point.X - previous.X;
                        float stepY = point.Y - previous.Y;
                        float stepZ = point.Z - previous.Z;
                        if(stepX * stepX + stepY * stepY + stepZ * stepZ < 1e-8f) return false;
                    }

                    continue;
                }

                if(started <= 1e-5f || planeDistance >= -1e-8f) continue;
                float span = started - planeDistance;
                if(span <= 1e-8f) continue;
                float t = started / span;
                if(t < 0f || t > 1f || t >= entryT) continue;
                int i0 = triangles[index];
                int i1 = triangles[index + 1];
                int i2 = triangles[index + 2];
                if(i0 < 0 || i1 < 0 || i2 < 0 || i0 >= surface.Length || i1 >= surface.Length || i2 >= surface.Length) continue;
                Vec3 hit = previous + (point - previous) * t;
                if(!TryProject(surface[i0].Position, surface[i1].Position, surface[i2].Position, hit, out _, out _, out float hitWeightA, out float hitWeightB, out float hitWeightC, crack))
                    continue;
                entryT = t;
                entrySlot = index;
                entryNormal = plane.Normal;
                entryDistance = planeDistance;
                entryWeightA = hitWeightA;
                entryWeightB = hitWeightB;
                entryWeightC = hitWeightC;
            }

            if(entrySlot >= 0) {
                triangle = entrySlot / 3;
                normal = entryNormal;
                distance = entryDistance;
                weightA = entryWeightA;
                weightB = entryWeightB;
                weightC = entryWeightC;
                return true;
            }

            if(!outsideHull && nearestSlot >= 0 && nearest < -1e-8f) {
                var nearestPlane = new FacePlane { Normal = nearestNormal, Point = Vec3.Zero, Slot = nearestSlot };
                if(TryProjectedFace(nearestPlane, convex, surface, triangles, point, nearest, out triangle, out normal, out distance, out weightA, out weightB, out weightC))
                    return true;
                if(!convex) return false;
            }

            return false;
        }

        public static bool TryProjectedFace(FacePlane plane, bool convex, Particle[] surface, int[] triangles, Vec3 point, float planeDistance, out int triangle, out Vec3 normal, out float distance, out float weightA, out float weightB, out float weightC) {
            triangle = -1;
            normal = Vec3.Zero;
            distance = 0f;
            weightA = weightB = weightC = 0f;
            int nearestSlot = plane.Slot;
            if(surface == null || triangles == null || nearestSlot < 0 || nearestSlot + 2 >= triangles.Length || planeDistance >= -1e-8f)
                return false;

            int c0 = triangles[nearestSlot];
            int c1 = triangles[nearestSlot + 1];
            int c2 = triangles[nearestSlot + 2];
            if(c0 < 0 || c1 < 0 || c2 < 0 || c0 >= surface.Length || c1 >= surface.Length || c2 >= surface.Length) return false;

            const float crack = 0.002f;
            if(TryProject(surface[c0].Position, surface[c1].Position, surface[c2].Position, point, out _, out _, out weightA, out weightB, out weightC, crack)) {
                triangle = nearestSlot / 3;
                normal = plane.Normal;
                distance = planeDistance;
                return true;
            }

            if(!convex) return false;

            Vec3 onPlane = point - plane.Normal * planeDistance;
            if(!TryClampedWeights(surface[c0].Position, surface[c1].Position, surface[c2].Position, onPlane, out weightA, out weightB, out weightC))
                return false;

            triangle = nearestSlot / 3;
            normal = plane.Normal;
            distance = planeDistance;
            return true;
        }

        public static bool TryProject(Vec3 a, Vec3 b, Vec3 c, Vec3 point, out Vec3 normal, out float distance, out float weightA, out float weightB, out float weightC, float border = 1e-4f) {
            normal = Vec3.Zero;
            distance = 0f;
            weightA = weightB = weightC = 0f;
            Vec3 raw = Vec3.Cross(b - a, c - a);
            float length = raw.Length;
            if(length < 1e-8f) return false;

            normal = raw * (1f / length);
            distance = Vec3.Dot(normal, point - a);
            Vec3 projected = point - normal * distance;
            Vec3 edge0 = b - a;
            Vec3 edge1 = c - a;
            Vec3 local = projected - a;
            float d00 = Vec3.Dot(edge0, edge0);
            float d01 = Vec3.Dot(edge0, edge1);
            float d11 = Vec3.Dot(edge1, edge1);
            float d02 = Vec3.Dot(edge0, local);
            float d12 = Vec3.Dot(edge1, local);
            float denom = d00 * d11 - d01 * d01;
            if(System.Math.Abs(denom) < 1e-12f) return false;

            float inverse = 1f / denom;
            weightB = (d11 * d02 - d01 * d12) * inverse;
            weightC = (d00 * d12 - d01 * d02) * inverse;
            weightA = 1f - weightB - weightC;
            if(border < 0f) border = 0f;
            return weightA >= -border && weightB >= -border && weightC >= -border;
        }

        static bool TryFacePlane(Vec3 a, Vec3 b, Vec3 c, Vec3 point, out Vec3 normal, out float distance) {
            normal = Vec3.Zero;
            distance = 0f;
            Vec3 raw = Vec3.Cross(b - a, c - a);
            float length = raw.Length;
            if(length < 1e-8f) return false;
            normal = raw * (1f / length);
            distance = Vec3.Dot(normal, point - a);
            return true;
        }

        static bool TryClampedWeights(Vec3 a, Vec3 b, Vec3 c, Vec3 point, out float weightA, out float weightB, out float weightC) {
            weightA = weightB = weightC = 0f;
            if(!TryProject(a, b, c, point, out _, out _, out weightA, out weightB, out weightC)) {
                Vec3 raw = Vec3.Cross(b - a, c - a);
                float length = raw.Length;
                if(length < 1e-8f) return false;
                Vec3 normal = raw * (1f / length);
                float distance = Vec3.Dot(normal, point - a);
                Vec3 projected = point - normal * distance;
                Vec3 edge0 = b - a;
                Vec3 edge1 = c - a;
                Vec3 local = projected - a;
                float d00 = Vec3.Dot(edge0, edge0);
                float d01 = Vec3.Dot(edge0, edge1);
                float d11 = Vec3.Dot(edge1, edge1);
                float d02 = Vec3.Dot(edge0, local);
                float d12 = Vec3.Dot(edge1, local);
                float denom = d00 * d11 - d01 * d01;
                if(System.Math.Abs(denom) < 1e-12f) return false;
                float inverse = 1f / denom;
                weightB = (d11 * d02 - d01 * d12) * inverse;
                weightC = (d00 * d12 - d01 * d02) * inverse;
                weightA = 1f - weightB - weightC;
            }

            ClampWeights(ref weightA, ref weightB, ref weightC);
            return weightA + weightB + weightC > 0.5f;
        }

        public static bool TryDeepestStab(Particle[] points, int vertexIndex, int[] ownTriangles, Particle[] surface, int[] surfaceTriangles, out int triangle, out Vec3 normal, out float distance, out float weightA, out float weightB, out float weightC) {
            triangle = -1;
            normal = Vec3.Zero;
            distance = 0f;
            weightA = weightB = weightC = 0f;
            if(points == null || ownTriangles == null || surface == null || surfaceTriangles == null) return false;
            if(vertexIndex < 0 || vertexIndex >= points.Length) return false;

            float best = 0f;
            bool found = false;
            Vec3 origin = points[vertexIndex].Position;
            Vec3 previous = points[vertexIndex].Previous;
            for (int slot = 0; slot + 2 < ownTriangles.Length; slot += 3) {
                for (int corner = 0; corner < 3; corner++) {
                    int other = ownTriangles[slot + corner];
                    if(other == vertexIndex || other < 0 || other >= points.Length) continue;
                    if(!TryStab(origin, previous, points[other].Position, surface, surfaceTriangles, out int face, out Vec3 faceNormal, out float faceDistance, out float faceWeightA, out float faceWeightB, out float faceWeightC))
                        continue;
                    if(found && faceDistance >= best) continue;
                    found = true;
                    best = faceDistance;
                    triangle = face;
                    normal = faceNormal;
                    distance = faceDistance;
                    weightA = faceWeightA;
                    weightB = faceWeightB;
                    weightC = faceWeightC;
                }
            }

            return found;
        }

        static bool TryStab(Vec3 inside, Vec3 previousInside, Vec3 outside, Particle[] surface, int[] triangles, out int triangle, out Vec3 normal, out float distance, out float weightA, out float weightB, out float weightC) {
            triangle = -1;
            normal = Vec3.Zero;
            distance = 0f;
            weightA = weightB = weightC = 0f;
            float best = 0f;
            bool found = false;
            for (int slot = 0; slot + 2 < triangles.Length; slot += 3) {
                int i0 = triangles[slot];
                int i1 = triangles[slot + 1];
                int i2 = triangles[slot + 2];
                if(i0 < 0 || i1 < 0 || i2 < 0 || i0 >= surface.Length || i1 >= surface.Length || i2 >= surface.Length) continue;
                Vec3 a = surface[i0].Position;
                Vec3 b = surface[i1].Position;
                Vec3 c = surface[i2].Position;
                Vec3 raw = Vec3.Cross(b - a, c - a);
                float length = raw.Length;
                if(length < 1e-8f) continue;
                Vec3 faceNormal = raw * (1f / length);
                float near = Vec3.Dot(faceNormal, inside - a);
                float far = Vec3.Dot(faceNormal, outside - a);
                if(near >= -1e-5f || far <= 1e-5f) continue;
                float previousNear = Vec3.Dot(faceNormal, previousInside - surface[i0].Previous);
                if(previousNear < -1e-4f) continue;
                float span = near - far;
                if(System.Math.Abs(span) < 1e-8f) continue;
                float t = near / span;
                if(t <= 0f || t >= 1f) continue;
                Vec3 hit = inside + (outside - inside) * t;
                if(!TryProject(a, b, c, hit, out _, out _, out float hitWeightA, out float hitWeightB, out float hitWeightC)) continue;
                if(found && near >= best) continue;
                found = true;
                best = near;
                triangle = slot / 3;
                normal = faceNormal;
                distance = near;
                weightA = hitWeightA;
                weightB = hitWeightB;
                weightC = hitWeightC;
            }

            return found;
        }

        public static void SolveTriangle(ref Contact contact, Particle[] points, Vec3[] rest, Particle[] surface, Vec3[] surfaceRest, int[] triangles, float stepSeconds, bool sharePosition) {
            if(stepSeconds <= 0f || !TryReadTriangle(contact, points, surface, triangles, out int pointIndex, out int i0, out int i1, out int i2))
                return;

            Particle point = points[pointIndex];
            Particle a = surface[i0];
            Particle b = surface[i1];
            Particle c = surface[i2];
            if(!TryFacePlane(a.Position, b.Position, c.Position, point.Position, out Vec3 normal, out float distance)) return;
            if(Vec3.Dot(normal, contact.Normal) <= 0f) return;

            contact.Normal = normal;
            if(distance >= 0f) {
                points[pointIndex] = point;
                return;
            }

            float weightA;
            float weightB;
            float weightC;
            if(!TryProject(a.Position, b.Position, c.Position, point.Position, out _, out _, out weightA, out weightB, out weightC)) {
                weightA = contact.Anchor.X;
                weightB = contact.Anchor.Y;
                weightC = contact.Anchor.Z;
            }

            ClampWeights(ref weightA, ref weightB, ref weightC);
            float weight = weightA + weightB + weightC < 0.5f
                ? 0f
                : point.InverseMass + weightA * weightA * a.InverseMass + weightB * weightB * b.InverseMass + weightC * weightC * c.InverseMass;
            bool sameBody = ReferenceEquals(points, surface);
            float started = point.InverseMass > 0f ? Vec3.Dot(normal, point.Previous - a.Position) : 0f;
            if(!sharePosition || sameBody) {
                if(point.InverseMass <= 0f) {
                    points[pointIndex] = point;
                    return;
                }

                if(started > 1e-5f) {
                    float span = started - distance;
                    float rewind = span > 1e-8f ? started / span : 0f;
                    if(rewind < 0f) rewind = 0f;
                    if(rewind > 1f) rewind = 1f;
                    point.Position = point.Previous + (point.Position - point.Previous) * rewind;
                    contact.Lambda += -distance;
                } else {
                    float guestLambda = -distance / point.InverseMass;
                    float guestUpdated = contact.Lambda + guestLambda;
                    if(guestUpdated < 0f) guestLambda = -contact.Lambda;
                    contact.Lambda += guestLambda;
                    Vec3 pushed = point.Position + normal * (point.InverseMass * guestLambda);
                    point.Position = ClampGrowth(points, rest, pointIndex, pushed);
                }

                points[pointIndex] = point;
                return;
            }

            if(started > 1e-5f) {
                float span = started - distance;
                float t = span > 1e-8f ? started / span : 0f;
                if(t < 0f) t = 0f;
                if(t > 1f) t = 1f;
                point.Position = point.Previous + (point.Position - point.Previous) * t;
                contact.Lambda += -distance;
                points[pointIndex] = point;
                if(!sameBody && weight > 1e-12f)
                    PushHost(surface, surfaceRest, ref a, ref b, ref c, i0, i1, i2, normal, weightA, weightB, weightC, -distance / weight);
                return;
            }

            if(weight <= 1e-12f) return;

            float deltaLambda = -distance / weight;
            float updated = contact.Lambda + deltaLambda;
            if(updated < 0f) deltaLambda = -contact.Lambda;
            contact.Lambda += deltaLambda;

            if(point.InverseMass > 0f) {
                Vec3 pushed = point.Position + normal * (point.InverseMass * deltaLambda);
                point.Position = ClampGrowth(points, rest, pointIndex, pushed);
            }

            points[pointIndex] = point;
            if(!sameBody) PushHost(surface, surfaceRest, ref a, ref b, ref c, i0, i1, i2, normal, weightA, weightB, weightC, deltaLambda);
        }

        static void PushHost(
            Particle[] surface,
            Vec3[] rest,
            ref Particle a,
            ref Particle b,
            ref Particle c,
            int i0,
            int i1,
            int i2,
            Vec3 normal,
            float weightA,
            float weightB,
            float weightC,
            float deltaLambda) {
            MoveHost(surface, rest, i0, ref a, normal, weightA, deltaLambda);
            MoveHost(surface, rest, i1, ref b, normal, weightB, deltaLambda);
            MoveHost(surface, rest, i2, ref c, normal, weightC, deltaLambda);
        }

        static void MoveHost(Particle[] surface, Vec3[] rest, int index, ref Particle particle, Vec3 normal, float bary, float deltaLambda) {
            if(particle.InverseMass > 0f && bary > 0f) {
                Vec3 requested = particle.Position - normal * (bary * particle.InverseMass * deltaLambda);
                particle.Position = ClampGrowth(surface, rest, index, requested);
            }

            surface[index] = particle;
        }

        public static void SolveSpan(ref Contact contact, Particle[] points, Vec3[] rest, Particle[] surface, int[] triangles, float stepSeconds) {
            if(stepSeconds <= 0f || points == null) return;

            int i0 = contact.ParticleIndex;
            int i1 = contact.OtherParticleIndex;
            int i2 = contact.ThirdParticleIndex;
            if(i0 < 0 || i1 < 0 || i0 >= points.Length || i1 >= points.Length) return;

            bool face = i2 >= 0 && i2 < points.Length;
            if(!TryReadTriangle(contact, points, surface, triangles, out _, out int a, out int b, out int c)) return;

            float t = contact.Offset;
            Vec3 sample;
            Vec3 previous;
            if(face) {
                sample = (points[i0].Position + points[i1].Position + points[i2].Position) * (1f / 3f);
                previous = (points[i0].Previous + points[i1].Previous + points[i2].Previous) * (1f / 3f);
            } else {
                if(t < 0.05f || t > 0.95f) t = 0.5f;
                float keep = 1f - t;
                sample = points[i0].Position * keep + points[i1].Position * t;
                previous = points[i0].Previous * keep + points[i1].Previous * t;
            }

            if(!TryFacePlane(surface[a].Position, surface[b].Position, surface[c].Position, sample, out Vec3 normal, out float distance))
                return;
            if(Vec3.Dot(normal, contact.Normal) <= 0f || distance >= 0f) return;

            Vec3 target;
            float started = Vec3.Dot(normal, previous - surface[a].Position);
            if(started > 1e-5f) {
                float span = started - distance;
                float hit = span > 1e-8f ? started / span : 0f;
                if(hit < 0f) hit = 0f;
                if(hit > 1f) hit = 1f;
                target = previous + (sample - previous) * hit;
            } else { target = sample - normal * distance; }

            Vec3 delta = target - sample;
            Vec3 com = Center(points, null);
            Vec3 restCom = Center(points, rest);
            Vec3 facePoint = surface[a].Position;
            MoveInside(points, rest, com, restCom, i0, delta, normal, facePoint);
            MoveInside(points, rest, com, restCom, i1, delta, normal, facePoint);
            if(face) MoveInside(points, rest, com, restCom, i2, delta, normal, facePoint);
            contact.Normal = normal;
            contact.Lambda += -distance;
        }

        static void MoveInside(Particle[] points, Vec3[] rest, Vec3 com, Vec3 restCom, int index, Vec3 delta, Vec3 normal, Vec3 facePoint) {
            if(index < 0 || index >= points.Length) return;
            float side = Vec3.Dot(normal, points[index].Position - facePoint);
            if(side >= 1e-4f) return;
            MoveOut(points, rest, com, restCom, index, delta);
        }

        static void MoveOut(Particle[] points, Vec3[] rest, Vec3 com, Vec3 restCom, int index, Vec3 delta) {
            Particle particle = points[index];
            if(particle.InverseMass <= 0f) return;
            particle.Position = ClampGrowth(points, rest, com, restCom, index, particle.Position + delta);
            points[index] = particle;
        }

        public static bool IsRound(Particle[] points, Vec3[] rest) {
            if(points == null || rest == null || rest.Length != points.Length || points.Length < 4) return true;

            Vec3 com = Center(points, null);
            Vec3 restCom = Center(points, rest);
            for (int i = 0; i < points.Length; i++) {
                float restRadius = (rest[i] - restCom).Length;
                if(restRadius < 1e-3f) continue;
                if((points[i].Position - com).Length < restRadius * 0.92f) return false;
            }

            return true;
        }

        public static void HoldShell(Particle[] points, Vec3[] rest) { HoldShell(points, rest, 1.05f); }

        public static void HoldShell(Particle[] points, Vec3[] rest, float outerScale) {
            if(points == null || rest == null || rest.Length != points.Length || points.Length < 4) return;

            float scale = outerScale < 1f ? 1f : outerScale;
            Vec3 com = Center(points, null);
            Vec3 restCom = Center(points, rest);
            for (int i = 0; i < points.Length; i++) {
                Particle particle = points[i];
                if(particle.InverseMass <= 0f) continue;
                float restRadius = (rest[i] - restCom).Length;
                if(restRadius < 1e-3f) continue;
                Vec3 offset = particle.Position - com;
                float radius = offset.Length;
                float outer = restRadius * scale;
                if(radius <= outer || radius < 1e-6f) continue;
                particle.Position = com + offset * (outer / radius);
                points[i] = particle;
            }
        }

        static Vec3 ClampGrowth(Particle[] points, Vec3[] rest, int index, Vec3 position) {
            return ClampGrowth(points, rest, Center(points, null), Center(points, rest), index, position);
        }

        static Vec3 ClampGrowth(Particle[] points, Vec3[] rest, Vec3 com, Vec3 restCom, int index, Vec3 position) {
            if(points == null || rest == null || rest.Length != points.Length || points.Length < 4) return position;
            if(index < 0 || index >= points.Length) return position;

            float restRadius = (rest[index] - restCom).Length;
            if(restRadius < 1e-3f) return position;

            float outer = restRadius * 1.05f;
            float inner = restRadius * 0.85f;
            Vec3 current = points[index].Position;
            Vec3 step = position - current;
            float stepLength = step.Length;
            float maxStep = restRadius * 0.2f;
            if(stepLength > maxStep) position = current + step * (maxStep / stepLength);
            Vec3 offset = position - com;
            float radius = offset.Length;
            float oldRadius = (points[index].Position - com).Length;
            if(radius < 1e-6f) {
                Vec3 oldOffset = points[index].Position - com;
                float oldLength = oldOffset.Length;
                if(oldLength < 1e-6f) return position;
                return com + oldOffset * (inner / oldLength);
            }

            if(radius < inner && radius < oldRadius) return com + offset * (inner / radius);
            if(radius <= outer || radius <= oldRadius) return position;
            float cap = oldRadius > outer ? oldRadius : outer;
            return com + offset * (cap / radius);
        }

        static Vec3 Center(Particle[] points, Vec3[] rest) {
            if(rest != null && (points == null || rest.Length != points.Length)) return Vec3.Zero;
            Vec3 center = Vec3.Zero;
            float mass = 0f;
            int count = points == null ? 0 : points.Length;
            for (int i = 0; i < count; i++) {
                float inverse = rest == null ? points[i].InverseMass : 1f;
                if(rest == null && inverse <= 0f) continue;
                float particleMass = rest == null ? 1f / inverse : 1f;
                center += (rest == null ? points[i].Position : rest[i]) * particleMass;
                mass += particleMass;
            }

            return mass > 0f ? center * (1f / mass) : Vec3.Zero;
        }

        public static void ApplyRestitution(ref Contact contact, Particle[] particles, float sleepSpeed) {
            if(contact.ParticleIndex < 0 || contact.ParticleIndex >= particles.Length) return;
            if(contact.IncomingNormalSpeed >= 0f) return;

            Particle particle = particles[contact.ParticleIndex];
            if(particle.InverseMass <= 0f) return;

            float separation = Vec3.Dot(contact.Normal, particle.Position) - contact.Offset;
            if(separation > 1e-3f) return;

            float normalSpeed = Vec3.Dot(particle.Velocity, contact.Normal);
            bool sleep = -contact.IncomingNormalSpeed < sleepSpeed;
            float target = sleep || contact.Restitution <= 0f
                ? 0f
                : -contact.Restitution * contact.IncomingNormalSpeed;
            particle.Velocity += contact.Normal * (target - normalSpeed);
            particles[contact.ParticleIndex] = particle;
        }

        public static void ApplyTriangleRestitution(ref Contact contact, Particle[] points, Particle[] surface, int[] triangles, float sleepSpeed, bool shareVelocity) {
            if(contact.Lambda <= 0f) return;
            if(!TryReadTriangle(contact, points, surface, triangles, out int pointIndex, out int i0, out int i1, out int i2)) return;

            Particle point = points[pointIndex];
            Particle a = surface[i0];
            Particle b = surface[i1];
            Particle c = surface[i2];
            Vec3 normal;
            float weightA;
            float weightB;
            float weightC;
            if(!TryContactFrame(contact, point.Position, a.Position, b.Position, c.Position, out normal, out _, out weightA, out weightB, out weightC)) {
                normal = contact.Normal;
                weightA = contact.Anchor.X;
                weightB = contact.Anchor.Y;
                weightC = contact.Anchor.Z;
                ClampWeights(ref weightA, ref weightB, ref weightC);
                if(normal.SqrLength < 0.5f || weightA + weightB + weightC < 0.5f) return;
            }

            float weight = point.InverseMass
                + weightA * weightA * a.InverseMass
                + weightB * weightB * b.InverseMass
                + weightC * weightC * c.InverseMass;
            if(weight <= 1e-12f) return;

            float incoming = contact.IncomingNormalSpeed;
            bool sleep = incoming >= -sleepSpeed || contact.Restitution <= 0f;
            float target = sleep ? 0f : -contact.Restitution * incoming;
            Vec3 surfaceVelocity = a.Velocity * weightA + b.Velocity * weightB + c.Velocity * weightC;
            float normalSpeed = Vec3.Dot(point.Velocity - surfaceVelocity, normal);
            bool sameBody = ReferenceEquals(points, surface);
            if(!shareVelocity || sameBody) {
                point.Velocity += normal * (target - normalSpeed);
                points[pointIndex] = point;
                contact.Normal = normal;
                float keptApproach = incoming < 0f ? -incoming : 0f;
                float keptFriction = contact.Friction > 0f ? contact.Friction * keptApproach : 0f;
                if(keptFriction > 0f) {
                    Vec3 keptRelative = point.Velocity - surfaceVelocity;
                    Vec3 keptTangent = keptRelative - normal * Vec3.Dot(keptRelative, normal);
                    float keptLength = keptTangent.Length;
                    if(keptLength > 1e-6f) {
                        float remove = keptLength < keptFriction ? keptLength : keptFriction;
                        point.Velocity -= keptTangent * (remove / keptLength);
                        points[pointIndex] = point;
                    }
                }

                return;
            }

            float impulse = (target - normalSpeed) / weight;
            point.Velocity += normal * (impulse * point.InverseMass);
            if(!sameBody) {
                a.Velocity -= normal * (impulse * weightA * a.InverseMass);
                b.Velocity -= normal * (impulse * weightB * b.InverseMass);
                c.Velocity -= normal * (impulse * weightC * c.InverseMass);
            }

            float approach = incoming < 0f ? -incoming : 0f;
            float frictionLimit = contact.Friction > 0f ? contact.Friction * approach : 0f;
            if(frictionLimit > 0f) {
                surfaceVelocity = a.Velocity * weightA + b.Velocity * weightB + c.Velocity * weightC;
                Vec3 relative = point.Velocity - surfaceVelocity;
                Vec3 tangential = relative - normal * Vec3.Dot(relative, normal);
                float length = tangential.Length;
                if(length > 1e-6f) {
                    float remove = length < frictionLimit ? length : frictionLimit;
                    Vec3 tangent = tangential * (1f / length);
                    float tangentImpulse = remove / weight;
                    point.Velocity -= tangent * (tangentImpulse * point.InverseMass);
                    if(!sameBody) {
                        a.Velocity += tangent * (tangentImpulse * weightA * a.InverseMass);
                        b.Velocity += tangent * (tangentImpulse * weightB * b.InverseMass);
                        c.Velocity += tangent * (tangentImpulse * weightC * c.InverseMass);
                    }
                }
            }

            points[pointIndex] = point;
            if(!sameBody) {
                surface[i0] = a;
                surface[i1] = b;
                surface[i2] = c;
            }

            contact.Normal = normal;
        }

        static bool TryContactFrame(Contact contact, Vec3 point, Vec3 a, Vec3 b, Vec3 c, out Vec3 normal, out float distance, out float weightA, out float weightB, out float weightC) {
            if(!TryProject(a, b, c, point, out normal, out distance, out weightA, out weightB, out weightC)) return false;
            ClampWeights(ref weightA, ref weightB, ref weightC);
            return weightA + weightB + weightC > 0.5f;
        }

        static void ClampWeights(ref float weightA, ref float weightB, ref float weightC) {
            if(weightA < 0f) weightA = 0f;
            if(weightB < 0f) weightB = 0f;
            if(weightC < 0f) weightC = 0f;
            float sum = weightA + weightB + weightC;
            if(sum <= 1e-8f) {
                weightA = weightB = weightC = 0f;
                return;
            }

            weightA /= sum;
            weightB /= sum;
            weightC /= sum;
        }

        static bool TryReadTriangle(Contact contact, Particle[] points, Particle[] surface, int[] triangles, out int pointIndex, out int i0, out int i1, out int i2) {
            pointIndex = contact.ParticleIndex;
            i0 = i1 = i2 = -1;
            if(points == null || surface == null || triangles == null) return false;
            int slot = contact.ShapeIndex * 3;
            if(pointIndex < 0 || pointIndex >= points.Length || slot < 0 || slot + 2 >= triangles.Length) return false;
            i0 = triangles[slot];
            i1 = triangles[slot + 1];
            i2 = triangles[slot + 2];
            return i0 >= 0 && i1 >= 0 && i2 >= 0 && i0 < surface.Length && i1 < surface.Length && i2 < surface.Length;
        }
    }
}
