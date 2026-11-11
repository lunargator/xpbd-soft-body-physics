using System;

namespace SoftBodyPhysics.Core {
    public struct Vec3 {
        public float X;
        public float Y;
        public float Z;

        public Vec3(float x, float y, float z) {
            X = x;
            Y = y;
            Z = z;
        }

        public static Vec3 Zero => new Vec3(0f, 0f, 0f);

        public float SqrLength => X * X + Y * Y + Z * Z;

        public float Length => (float)Math.Sqrt(SqrLength);

        public static Vec3 operator +(Vec3 a, Vec3 b) { return new Vec3(a.X + b.X, a.Y + b.Y, a.Z + b.Z); }

        public static Vec3 operator -(Vec3 a, Vec3 b) { return new Vec3(a.X - b.X, a.Y - b.Y, a.Z - b.Z); }

        public static Vec3 operator *(Vec3 value, float scale) { return new Vec3(value.X * scale, value.Y * scale, value.Z * scale); }

        public static Vec3 operator *(float scale, Vec3 value) { return value * scale; }

        public static float Dot(Vec3 a, Vec3 b) { return a.X * b.X + a.Y * b.Y + a.Z * b.Z; }

        public static Vec3 Cross(Vec3 a, Vec3 b) {
            return new Vec3(
                a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X);
        }

        public bool HasNaN() { return float.IsNaN(X) || float.IsNaN(Y) || float.IsNaN(Z); }
    }
}
