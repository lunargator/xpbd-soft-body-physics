using System.IO;
using NUnit.Framework;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Constraints;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Geometry;
using UnityEngine;

namespace SoftBodyPhysics.Tests {
    public class ShellLimitTests {
        [Test]
        public void ACrushedBodyCanSpreadPastTheOriginalShell() {
            float tight = RadialRatio(1.05f, out float tightMean);
            float loose = RadialRatio(1.2f, out float looseMean);
            string path = Path.Combine(Directory.GetParent(Application.dataPath).FullName, "Temp", "HoldShellCompare.txt");
            File.WriteAllText(path, "tight " + tight.ToString("0.000") + " mean " + tightMean.ToString("0.000")
                + " loose " + loose.ToString("0.000") + " mean " + looseMean.ToString("0.000"));

            Assert.AreEqual(1.05f, new CollisionWorld().ShellScale, 1e-5f);
            Assert.LessOrEqual(tight, 1.07f, "tight " + tight);
            Assert.Greater(loose, 1.07f, "loose " + loose);
            Assert.Greater(looseMean, 0.55f, "mean " + looseMean);
        }

        static float RadialRatio(float scale, out float meanRatio) {
            BodyMaterial material = BodyMaterial.Default;
            material.Gravity = new Vec3(0f, -9.81f, 0f);
            material.Damping = 0.8f;
            material.Friction = 0.4f;
            material.Restitution = 0f;
            material.Compliance = 0.02f;
            DeformableBody body = SphereGeometry.Create(0.4f, 5, 3, material, new Vec3(0f, 0.4f, 0f));
            body.AddConstraint(new VolumeConstraint(body, 0.00005f));
            var world = new PhysicsWorld(new PhysicsSettings {
                FixedDeltaTime = 1f / 60f,
                SolverIterations = 4,
                MaxStepsPerFrame = 1
            });
            if(scale > 0f) world.Collision.ShellScale = scale;
            world.Collision.AddPlane(CollisionPlane.Horizontal(0f));
            world.Collision.AddPlane(CollisionPlane.FromNormalAndPoint(new Vec3(0f, -1f, 0f), new Vec3(0f, 0.45f, 0f)));
            world.AddBody(body);
            for (int i = 0; i < 50; i++) world.Step(1f / 60f);

            float max = 0f;
            float sum = 0f;
            int count = 0;
            Vec3 center = body.CenterOfMass();
            Vec3 restCenter = Vec3.Zero;
            for (int i = 0; i < body.RestPositions.Length; i++) restCenter += body.RestPositions[i];
            restCenter = restCenter * (1f / body.RestPositions.Length);
            for (int i = 0; i < body.Particles.Length; i++) {
                Assert.IsFalse(body.Particles[i].Position.HasNaN());
                float rest = (body.RestPositions[i] - restCenter).Length;
                if(rest < 1e-3f) continue;
                float ratio = (body.Particles[i].Position - center).Length / rest;
                if(ratio > max) max = ratio;
                sum += ratio;
                count++;
            }

            meanRatio = count == 0 ? 0f : sum / count;
            return max;
        }
    }
}
