using System;
using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Collision {

    public struct CollisionPlane {
        public Vec3 Normal;
        public float Offset;

        public static CollisionPlane Horizontal(float height) {
            return new CollisionPlane {
                Normal = new Vec3(0f, 1f, 0f),
                Offset = height
            };
        }

        public static CollisionPlane FromNormalAndPoint(Vec3 normal, Vec3 point) {
            float length = normal.Length;
            if(length < 1e-8f) normal = new Vec3(0f, 1f, 0f);
            else normal = normal * (1f / length);
            return new CollisionPlane {
                Normal = normal,
                Offset = Vec3.Dot(normal, point)
            };
        }
    }

    public struct CollisionBox {
        public Vec3 Min;
        public Vec3 Max;

        public static CollisionBox FromCenterSize(Vec3 center, Vec3 size) {
            var half = new Vec3(Math.Abs(size.X) * 0.5f, Math.Abs(size.Y) * 0.5f, Math.Abs(size.Z) * 0.5f);
            return new CollisionBox {
                Min = center - half,
                Max = center + half
            };
        }

        public static CollisionBox Ordered(Vec3 a, Vec3 b) {
            return new CollisionBox {
                Min = new Vec3(Math.Min(a.X, b.X), Math.Min(a.Y, b.Y), Math.Min(a.Z, b.Z)),
                Max = new Vec3(Math.Max(a.X, b.X), Math.Max(a.Y, b.Y), Math.Max(a.Z, b.Z))
            };
        }

        public bool TryNearestExit(Vec3 point, out Vec3 normal, out float offset, out float depth) {
            normal = Vec3.Zero;
            offset = 0f;
            depth = 0f;
            if(point.X < Min.X || point.X > Max.X
                || point.Y < Min.Y || point.Y > Max.Y
                || point.Z < Min.Z || point.Z > Max.Z)
                return false;

            depth = float.MaxValue;
            Pick(Max.Y - point.Y, new Vec3(0f, 1f, 0f), Max.Y, ref normal, ref offset, ref depth);
            Pick(point.Y - Min.Y, new Vec3(0f, -1f, 0f), -Min.Y, ref normal, ref offset, ref depth);
            Pick(Max.X - point.X, new Vec3(1f, 0f, 0f), Max.X, ref normal, ref offset, ref depth);
            Pick(point.X - Min.X, new Vec3(-1f, 0f, 0f), -Min.X, ref normal, ref offset, ref depth);
            Pick(Max.Z - point.Z, new Vec3(0f, 0f, 1f), Max.Z, ref normal, ref offset, ref depth);
            Pick(point.Z - Min.Z, new Vec3(0f, 0f, -1f), -Min.Z, ref normal, ref offset, ref depth);
            return depth > 1e-8f;
        }

        static void Pick(float distance, Vec3 candidateNormal, float candidateOffset, ref Vec3 normal, ref float offset, ref float depth) {
            if(distance < depth) {
                depth = distance;
                normal = candidateNormal;
                offset = candidateOffset;
            }
        }
    }

    public struct CollisionSphere {
        public Vec3 Center;
        public float Radius;
        public Vec3 Motion;
        public ColliderPose Pose;
        public int UseSurface;
        public float SurfaceFriction;
        public float SurfaceRestitution;

        public bool TryContact(Vec3 previous, Vec3 current, out Vec3 normal, out float offset, out float depth) {
            if(TryNearestExit(current, out normal, out offset, out depth))
                return KeepEnteredFace(previous, current, ref normal, ref offset, ref depth);
            if(Radius > 1e-8f && FirstHit(previous, current, out float time)) {
                Vec3 inside = previous + (current - previous) * Math.Min(time + 1e-4f, 1f);
                if(TryNearestExit(inside, out normal, out offset, out _) && Behind(current, normal, offset, out depth)) return true;
            }

            return TrySweep(current, out normal, out offset, out depth);
        }

        bool KeepEnteredFace(Vec3 previous, Vec3 current, ref Vec3 normal, ref float offset, ref float depth) {
            if(TryNearestExit(previous, out Vec3 exitNormal, out float exitOffset, out _) && Behind(current, exitNormal, exitOffset, out depth)) {
                normal = exitNormal;
                offset = exitOffset;
                return true;
            }

            if(Radius > 1e-8f && FirstHit(previous, current, out float time)) {
                Vec3 inside = previous + (current - previous) * Math.Min(time + 1e-4f, 1f);
                if(TryNearestExit(inside, out Vec3 entryNormal, out float entryOffset, out _) && Behind(current, entryNormal, entryOffset, out depth)) {
                    normal = entryNormal;
                    offset = entryOffset;
                    return true;
                }
            }

            return true;
        }

        static bool Behind(Vec3 point, Vec3 normal, float planeOffset, out float depth) {
            float separation = Vec3.Dot(normal, point) - planeOffset;
            depth = separation < 0f ? -separation : 0f;
            return separation < 0f;
        }

        bool TrySweep(Vec3 current, out Vec3 normal, out float offset, out float depth) {
            normal = Vec3.Zero;
            offset = 0f;
            depth = 0f;
            if(Motion.SqrLength < 1e-10f || Radius <= 1e-8f) return false;
            Vec3 ahead = current + Motion;
            if(!FirstHit(current, ahead, out float time)) return false;
            Vec3 hit = current + (ahead - current) * time;
            Vec3 fromCenter = hit - Center;
            float distance = fromCenter.Length;
            if(distance < 1e-8f) return false;
            normal = fromCenter * (1f / distance);
            offset = Vec3.Dot(normal, Center + normal * Radius);
            return Behind(current, normal, offset, out depth);
        }

        public bool TryNearestExit(Vec3 point, out Vec3 normal, out float offset, out float depth) {
            normal = Vec3.Zero;
            offset = 0f;
            depth = 0f;
            if(Radius <= 1e-8f) return false;
            Vec3 fromCenter = point - Center;
            float distance = fromCenter.Length;
            if(distance > Radius) return false;
            normal = distance > 1e-8f ? fromCenter * (1f / distance) : new Vec3(0f, 1f, 0f);
            depth = Radius - distance;
            offset = Vec3.Dot(normal, Center + normal * Radius);
            return true;
        }

        internal bool FirstHit(Vec3 previous, Vec3 current, out float time) {
            time = 0f;
            Vec3 delta = current - previous;
            Vec3 fromCenter = previous - Center;
            float a = delta.SqrLength;
            if(a < 1e-12f) return false;
            float b = 2f * Vec3.Dot(fromCenter, delta);
            float c = fromCenter.SqrLength - Radius * Radius;
            float discriminant = b * b - 4f * a * c;
            if(discriminant < 0f) return false;
            float root = (float)Math.Sqrt(discriminant);
            float enter = (-b - root) / (2f * a);
            if(enter < 0f || enter > 1f) enter = (-b + root) / (2f * a);
            if(enter < 0f || enter > 1f) return false;
            time = enter;
            return true;
        }
    }

    public struct CollisionCapsule {
        public Vec3 Start;
        public Vec3 End;
        public float Radius;
        public Vec3 Motion;
        public ColliderPose Pose;
        public int UseSurface;
        public float SurfaceFriction;
        public float SurfaceRestitution;

        public bool TryContact(Vec3 previous, Vec3 current, out Vec3 normal, out float offset, out float depth) {
            if(TryNearestExit(current, out normal, out offset, out depth))
                return KeepEnteredFace(previous, current, ref normal, ref offset, ref depth);
            if(Radius > 1e-8f && FirstHit(previous, current, out float time)) {
                Vec3 inside = previous + (current - previous) * Math.Min(time + 1e-4f, 1f);
                if(TryNearestExit(inside, out normal, out offset, out _) && Behind(current, normal, offset, out depth)) return true;
            }

            return TrySweep(current, out normal, out offset, out depth);
        }

        bool KeepEnteredFace(Vec3 previous, Vec3 current, ref Vec3 normal, ref float offset, ref float depth) {
            if(TryNearestExit(previous, out Vec3 exitNormal, out float exitOffset, out _) && Behind(current, exitNormal, exitOffset, out depth)) {
                normal = exitNormal;
                offset = exitOffset;
                return true;
            }

            if(Radius > 1e-8f && FirstHit(previous, current, out float time)) {
                Vec3 inside = previous + (current - previous) * Math.Min(time + 1e-4f, 1f);
                if(TryNearestExit(inside, out Vec3 entryNormal, out float entryOffset, out _) && Behind(current, entryNormal, entryOffset, out depth)) {
                    normal = entryNormal;
                    offset = entryOffset;
                    return true;
                }
            }

            return true;
        }

        static bool Behind(Vec3 point, Vec3 normal, float planeOffset, out float depth) {
            float separation = Vec3.Dot(normal, point) - planeOffset;
            depth = separation < 0f ? -separation : 0f;
            return separation < 0f;
        }

        bool TrySweep(Vec3 current, out Vec3 normal, out float offset, out float depth) {
            normal = Vec3.Zero;
            offset = 0f;
            depth = 0f;
            if(Motion.SqrLength < 1e-10f || Radius <= 1e-8f) return false;
            Vec3 ahead = current + Motion;
            if(!FirstHit(current, ahead, out float time)) return false;
            Vec3 hit = current + (ahead - current) * time;
            Vec3 closest = Closest(hit);
            Vec3 outward = hit - closest;
            float distance = outward.Length;
            if(distance < 1e-8f) return false;
            normal = outward * (1f / distance);
            offset = Vec3.Dot(normal, closest + normal * Radius);
            return Behind(current, normal, offset, out depth);
        }

        public bool TryNearestExit(Vec3 point, out Vec3 normal, out float offset, out float depth) {
            Vec3 closest = Closest(point);
            var sphere = new CollisionSphere { Center = closest, Radius = Radius };
            return sphere.TryNearestExit(point, out normal, out offset, out depth);
        }

        bool FirstHit(Vec3 previous, Vec3 current, out float time) {
            time = 2f;
            bool hit = false;
            var start = new CollisionSphere { Center = Start, Radius = Radius };
            var end = new CollisionSphere { Center = End, Radius = Radius };
            if(start.TryContact(previous, current, out _, out _, out _) && start.FirstHit(previous, current, out float startTime) && startTime < time) {
                time = startTime;
                hit = true;
            }

            if(end.TryContact(previous, current, out _, out _, out _) && end.FirstHit(previous, current, out float endTime) && endTime < time) {
                time = endTime;
                hit = true;
            }

            if(CylinderHit(previous, current, out float shaft) && shaft < time) {
                time = shaft;
                hit = true;
            }

            return hit;
        }

        bool CylinderHit(Vec3 previous, Vec3 current, out float time) {
            time = 0f;
            Vec3 axis = End - Start;
            float length = axis.Length;
            if(length < 1e-6f || Radius <= 1e-8f) return false;
            Vec3 direction = axis * (1f / length);
            Vec3 delta = current - previous;
            Vec3 offset = previous - Start;
            Vec3 deltaSide = delta - direction * Vec3.Dot(delta, direction);
            Vec3 offsetSide = offset - direction * Vec3.Dot(offset, direction);
            float a = deltaSide.SqrLength;
            float b = 2f * Vec3.Dot(offsetSide, deltaSide);
            float c = offsetSide.SqrLength - Radius * Radius;
            float discriminant = b * b - 4f * a * c;
            if(a < 1e-12f || discriminant < 0f) return false;
            float root = (float)Math.Sqrt(discriminant);
            float enter = (-b - root) / (2f * a);
            if(enter < 0f || enter > 1f) return false;
            Vec3 hit = previous + delta * enter;
            float along = Vec3.Dot(hit - Start, direction);
            if(along < 0f || along > length) return false;
            time = enter;
            return true;
        }

        Vec3 Closest(Vec3 point) {
            Vec3 axis = End - Start;
            float lengthSquared = axis.SqrLength;
            if(lengthSquared < 1e-12f) return Start;
            float along = Vec3.Dot(point - Start, axis) / lengthSquared;
            if(along < 0f) along = 0f;
            if(along > 1f) along = 1f;
            return Start + axis * along;
        }
    }

    public struct CollisionOrientedBox {
        public Vec3 Center;
        public Vec3 AxisX;
        public Vec3 AxisY;
        public Vec3 AxisZ;
        public Vec3 Motion;
        public ColliderPose Pose;
        public int UseSurface;
        public float SurfaceFriction;
        public float SurfaceRestitution;

        public bool TryContact(Vec3 previous, Vec3 current, out Vec3 normal, out float offset, out float depth) {
            if(TryNearestExit(current, out normal, out offset, out depth)) {
                if(TryNearestExit(previous, out Vec3 exitNormal, out float exitOffset, out _) && Behind(current, exitNormal, exitOffset, out depth)) {
                    normal = exitNormal;
                    offset = exitOffset;
                    return true;
                }

                if(EntryPlane(previous, current, out Vec3 entryNormal, out float entryOffset) && Behind(current, entryNormal, entryOffset, out depth)) {
                    normal = entryNormal;
                    offset = entryOffset;
                    return true;
                }

                return true;
            }

            if(Coordinates(previous, out float x0, out float y0, out float z0)
                && Coordinates(current, out float x1, out float y1, out float z1)
                && Enters(x0, y0, z0, x1, y1, z1, out float time)) {
                Vec3 inside = previous + (current - previous) * Math.Min(time + 1e-4f, 1f);
                if(TryNearestExit(inside, out normal, out offset, out _) && Behind(current, normal, offset, out depth)) return true;
            }

            return TrySweep(current, out normal, out offset, out depth);
        }

        static bool Behind(Vec3 point, Vec3 normal, float planeOffset, out float depth) {
            float separation = Vec3.Dot(normal, point) - planeOffset;
            depth = separation < 0f ? -separation : 0f;
            return separation < 0f;
        }

        bool EntryPlane(Vec3 previous, Vec3 current, out Vec3 normal, out float offset) {
            normal = Vec3.Zero;
            offset = 0f;
            if(!Coordinates(previous, out float x0, out float y0, out float z0)) return false;
            if(!Coordinates(current, out float x1, out float y1, out float z1)) return false;
            float entry = 0f;
            float exit = 1f;
            int face = -1;
            if(!SlabFace(x0, x1 - x0, 0, ref entry, ref exit, ref face)
                || !SlabFace(y0, y1 - y0, 2, ref entry, ref exit, ref face)
                || !SlabFace(z0, z1 - z0, 4, ref entry, ref exit, ref face))
                return false;
            if(entry < 0f || entry > 1f || face < 0) return false;
            return FacePlane(face, out normal, out offset);
        }

        static bool SlabFace(float start, float delta, int axis, ref float entry, ref float exit, ref int face) {
            if(Math.Abs(delta) < 1e-8f) return start >= -1f && start <= 1f;
            float inverse = 1f / delta;
            float negative = (-1f - start) * inverse;
            float positive = (1f - start) * inverse;
            float near;
            int nearFace;
            float far;
            if(negative < positive) {
                near = negative;
                far = positive;
                nearFace = axis + 1;
            } else {
                near = positive;
                far = negative;
                nearFace = axis;
            }

            if(near > entry) {
                entry = near;
                face = nearFace;
            }

            if(far < exit) exit = far;
            return entry <= exit;
        }

        bool FacePlane(int face, out Vec3 normal, out float offset) {
            if(face == 0) return Outward(AxisX, AxisY, AxisZ, out normal, out offset);
            if(face == 1) return Outward(AxisX * -1f, AxisY, AxisZ, out normal, out offset);
            if(face == 2) return Outward(AxisY, AxisZ, AxisX, out normal, out offset);
            if(face == 3) return Outward(AxisY * -1f, AxisZ, AxisX, out normal, out offset);
            if(face == 4) return Outward(AxisZ, AxisX, AxisY, out normal, out offset);
            return Outward(AxisZ * -1f, AxisX, AxisY, out normal, out offset);
        }

        bool Outward(Vec3 axis, Vec3 edgeU, Vec3 edgeV, out Vec3 normal, out float offset) {
            Vec3 raw = Vec3.Cross(edgeU, edgeV);
            float length = raw.Length;
            normal = Vec3.Zero;
            offset = 0f;
            if(length < 1e-10f) return false;
            normal = raw * (1f / length);
            if(Vec3.Dot(normal, axis) < 0f) normal = normal * -1f;
            offset = Vec3.Dot(normal, Center + axis);
            return true;
        }

        bool TrySweep(Vec3 current, out Vec3 normal, out float offset, out float depth) {
            normal = Vec3.Zero;
            offset = 0f;
            depth = 0f;
            if(Motion.SqrLength < 1e-10f) return false;
            Vec3 ahead = current + Motion;
            return EntryPlane(current, ahead, out normal, out offset) && Behind(current, normal, offset, out depth);
        }

        public bool TryNearestExit(Vec3 point, out Vec3 normal, out float offset, out float depth) {
            normal = Vec3.Zero;
            offset = 0f;
            depth = 0f;
            if(!Coordinates(point, out float x, out float y, out float z)) return false;
            if(Math.Abs(x) > 1f || Math.Abs(y) > 1f || Math.Abs(z) > 1f) return false;

            depth = float.MaxValue;
            Consider(1f - x, AxisX, AxisY, AxisZ, ref normal, ref offset, ref depth);
            Consider(1f + x, AxisX * -1f, AxisY, AxisZ, ref normal, ref offset, ref depth);
            Consider(1f - y, AxisY, AxisZ, AxisX, ref normal, ref offset, ref depth);
            Consider(1f + y, AxisY * -1f, AxisZ, AxisX, ref normal, ref offset, ref depth);
            Consider(1f - z, AxisZ, AxisX, AxisY, ref normal, ref offset, ref depth);
            Consider(1f + z, AxisZ * -1f, AxisX, AxisY, ref normal, ref offset, ref depth);
            return depth < float.MaxValue && depth > 1e-8f;
        }

        bool Coordinates(Vec3 point, out float x, out float y, out float z) {
            Vec3 delta = point - Center;
            float determinant = Vec3.Dot(AxisX, Vec3.Cross(AxisY, AxisZ));
            x = y = z = 0f;
            if(Math.Abs(determinant) < 1e-10f) return false;
            float inverse = 1f / determinant;
            x = Vec3.Dot(delta, Vec3.Cross(AxisY, AxisZ)) * inverse;
            y = Vec3.Dot(delta, Vec3.Cross(AxisZ, AxisX)) * inverse;
            z = Vec3.Dot(delta, Vec3.Cross(AxisX, AxisY)) * inverse;
            return true;
        }

        void Consider(float localDistance, Vec3 axis, Vec3 edgeU, Vec3 edgeV, ref Vec3 normal, ref float offset, ref float depth) {
            Vec3 raw = Vec3.Cross(edgeU, edgeV);
            float rawLength = raw.Length;
            if(rawLength < 1e-10f || localDistance < 0f) return;
            float altitude = Math.Abs(Vec3.Dot(axis, raw)) / rawLength;
            float world = localDistance * altitude;
            if(world >= depth) return;
            Vec3 faceNormal = raw * (1f / rawLength);
            if(Vec3.Dot(faceNormal, axis) < 0f) faceNormal = faceNormal * -1f;
            depth = world;
            normal = faceNormal;
            offset = Vec3.Dot(faceNormal, Center + axis);
        }

        static bool Enters(float x0, float y0, float z0, float x1, float y1, float z1, out float time) {
            time = 0f;
            float entry = 0f;
            float exit = 1f;
            if(!Slab(x0, x1 - x0, ref entry, ref exit)
                || !Slab(y0, y1 - y0, ref entry, ref exit)
                || !Slab(z0, z1 - z0, ref entry, ref exit))
                return false;
            if(entry < 0f || entry > 1f) return false;
            time = entry;
            return true;
        }

        static bool Slab(float start, float delta, ref float entry, ref float exit) {
            if(Math.Abs(delta) < 1e-8f) return start >= -1f && start <= 1f;
            float inverse = 1f / delta;
            float near = (-1f - start) * inverse;
            float far = (1f - start) * inverse;
            if(near > far) {
                float swap = near;
                near = far;
                far = swap;
            }

            if(near > entry) entry = near;
            if(far < exit) exit = far;
            return entry <= exit;
        }
    }

    public struct CollisionTriangle {
        public Vec3 A;
        public Vec3 B;
        public Vec3 C;
        public Vec3 Motion;
        public ColliderPose Pose;
        public int Closed;
        public int UseSurface;
        public float SurfaceFriction;
        public float SurfaceRestitution;

        public bool Misses(Vec3 from, Vec3 to) {
            const float pad = 1f;
            float minX = Smallest(A.X, B.X, C.X) - pad;
            float minY = Smallest(A.Y, B.Y, C.Y) - pad;
            float minZ = Smallest(A.Z, B.Z, C.Z) - pad;
            float maxX = Largest(A.X, B.X, C.X) + pad;
            float maxY = Largest(A.Y, B.Y, C.Y) + pad;
            float maxZ = Largest(A.Z, B.Z, C.Z) + pad;
            if((from.X < minX && to.X < minX) || (from.X > maxX && to.X > maxX)) return true;
            if((from.Y < minY && to.Y < minY) || (from.Y > maxY && to.Y > maxY)) return true;
            if((from.Z < minZ && to.Z < minZ) || (from.Z > maxZ && to.Z > maxZ)) return true;
            return false;
        }

        public bool TryContact(Vec3 previous, Vec3 current, out Vec3 normal, out float offset, out float depth) {
            normal = Vec3.Zero;
            offset = 0f;
            depth = 0f;
            if(!Plane(out Vec3 winding, out float planeOffset)) return false;
            float currentSide = Vec3.Dot(winding, current) - planeOffset;
            float previousSide = Vec3.Dot(winding, previous) - planeOffset;
            bool inside = Contains(current - winding * currentSide);
            if(!inside && Hits(previous, current, winding, planeOffset, out float time))
                inside = Contains(previous + (current - previous) * time);
            if(!inside) return false;

            if(Closed != 0) {
                bool entered = currentSide < 0f && previousSide > 1e-4f;
                bool shallow = currentSide < 0f && -currentSide <= 0.35f;
                if(entered || shallow) {
                    normal = winding;
                    offset = planeOffset;
                    depth = -currentSide;
                    return true;
                }

                if(currentSide > 0f && previousSide <= 0f) {
                    normal = winding * -1f;
                    offset = -planeOffset;
                    depth = currentSide;
                    return true;
                }

                return false;
            }

            const float embed = 0.1f;
            bool throughBack = currentSide < 0f && previousSide >= -1e-4f;
            bool throughFront = currentSide > 0f && previousSide <= embed && currentSide > previousSide + 1e-4f;
            bool onBack = currentSide < 0f && -currentSide <= embed;
            bool onFront = currentSide > 0f && currentSide <= embed;
            bool rising = currentSide > previousSide + 0.002f;
            if(throughBack || (onBack && !rising)) {
                normal = winding;
                offset = planeOffset;
                depth = -currentSide;
                return true;
            }

            if(throughFront || onFront) {
                normal = winding * -1f;
                offset = -planeOffset;
                depth = currentSide;
                return true;
            }

            return false;
        }

        static float Smallest(float a, float b, float c) { return Math.Min(a, Math.Min(b, c)); }

        static float Largest(float a, float b, float c) { return Math.Max(a, Math.Max(b, c)); }

        bool Plane(out Vec3 normal, out float offset) {
            Vec3 raw = Vec3.Cross(B - A, C - A);
            float length = raw.Length;
            normal = Vec3.Zero;
            offset = 0f;
            if(length < 1e-10f) return false;
            normal = raw * (1f / length);
            offset = Vec3.Dot(normal, A);
            return true;
        }

        bool Contains(Vec3 point) {
            Vec3 ab = B - A;
            Vec3 ac = C - A;
            Vec3 ap = point - A;
            float d00 = Vec3.Dot(ab, ab);
            float d01 = Vec3.Dot(ab, ac);
            float d11 = Vec3.Dot(ac, ac);
            float d20 = Vec3.Dot(ap, ab);
            float d21 = Vec3.Dot(ap, ac);
            float denom = d00 * d11 - d01 * d01;
            if(Math.Abs(denom) < 1e-12f) return false;
            float v = (d11 * d20 - d01 * d21) / denom;
            float w = (d00 * d21 - d01 * d20) / denom;
            float u = 1f - v - w;
            const float edge = -1e-4f;
            return u >= edge && v >= edge && w >= edge;
        }

        static bool Hits(Vec3 previous, Vec3 current, Vec3 normal, float planeOffset, out float time) {
            time = 0f;
            Vec3 delta = current - previous;
            float approach = Vec3.Dot(normal, delta);
            if(Math.Abs(approach) < 1e-8f) return false;
            time = (planeOffset - Vec3.Dot(normal, previous)) / approach;
            return time >= 0f && time <= 1f;
        }
    }

    public struct ColliderPose {
        public int HasBasis;
        public Vec3 Origin;
        public Vec3 X;
        public Vec3 Y;
        public Vec3 Z;
        public Vec3 PreviousOrigin;
        public Vec3 PreviousX;
        public Vec3 PreviousY;
        public Vec3 PreviousZ;

        public Vec3 At(Vec3 worldPoint, Vec3 translation) {
            if(HasBasis == 0) return translation;
            float determinant = Vec3.Dot(X, Vec3.Cross(Y, Z));
            if(Math.Abs(determinant) < 1e-10f) return translation;
            float previousDeterminant = Vec3.Dot(PreviousX, Vec3.Cross(PreviousY, PreviousZ));
            if(Math.Abs(previousDeterminant) < 1e-10f) return translation;
            Vec3 delta = worldPoint - PreviousOrigin;
            float inverse = 1f / previousDeterminant;
            float x = Vec3.Dot(delta, Vec3.Cross(PreviousY, PreviousZ)) * inverse;
            float y = Vec3.Dot(delta, Vec3.Cross(PreviousZ, PreviousX)) * inverse;
            float z = Vec3.Dot(delta, Vec3.Cross(PreviousX, PreviousY)) * inverse;
            Vec3 carried = Origin + X * x + Y * y + Z * z;
            return carried - worldPoint;
        }
    }
}
