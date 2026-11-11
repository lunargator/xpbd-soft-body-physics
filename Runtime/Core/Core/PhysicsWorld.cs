using System;
using System.Collections.Generic;
using System.Diagnostics;
using SoftBodyPhysics.Collision;
using SoftBodyPhysics.Constraints;

namespace SoftBodyPhysics.Core {
    public sealed class PhysicsWorld {
        readonly List<DeformableBody> bodies = new List<DeformableBody>();

        public PhysicsSettings Settings { get; }
        public CollisionWorld Collision { get; } = new CollisionWorld();
        public float Accumulator { get; private set; }

        public float DroppedTime { get; private set; }

        public float LastDroppedTime { get; private set; }
        public IReadOnlyList<DeformableBody> Bodies => bodies;
        public StepProfile LastProfile { get; private set; }

        public PhysicsWorld(PhysicsSettings settings) {
            Settings = settings ?? new PhysicsSettings();
        }

        public void AddBody(DeformableBody body) {
            if(body == null || bodies.Contains(body)) return;
            bodies.Add(body);
        }

        public bool RemoveBody(DeformableBody body) {
            return body != null && bodies.Remove(body);
        }

        public void Tick(float frameDelta) {
            if(frameDelta <= 0f || float.IsNaN(frameDelta) || float.IsInfinity(frameDelta)) return;

            float stepSeconds = Settings.FixedDeltaTime;
            if(stepSeconds <= 0f) return;

            int maxSteps = Settings.MaxStepsPerFrame < 1 ? 1 : Settings.MaxStepsPerFrame;
            Accumulator += frameDelta;
            float backlog = stepSeconds * maxSteps;
            float dropped = 0f;
            if(Accumulator > backlog) {
                dropped = Accumulator - backlog;
                Accumulator = backlog;
            }

            LastDroppedTime = dropped;
            DroppedTime += dropped;

            int steps = 0;
            while (Accumulator >= stepSeconds && steps < maxSteps) {
                Step(stepSeconds);
                Accumulator -= stepSeconds;
                steps++;
            }
        }

        public void Step(float stepSeconds) {
            if(stepSeconds <= 0f) return;

            long predict = 0;
            long constraints = 0;
            long collision = 0;
            long velocity = 0;
            long mark = Stopwatch.GetTimestamp();

            for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++) Predict(bodies[bodyIndex], stepSeconds);
            predict += TicksSince(ref mark);

            int iterations = Settings.SolverIterations < 0 ? 0 : Settings.SolverIterations;
            Collision.BeginStep();
            for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++) ResetConstraints(bodies[bodyIndex]);
            constraints += TicksSince(ref mark);

            for (int iteration = 0; iteration < iterations; iteration++) {
                Collision.Collect(bodies);
                Collision.Solve(bodies, stepSeconds);
                collision += TicksSince(ref mark);
                for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++) SolveBody(bodies[bodyIndex], stepSeconds);
                constraints += TicksSince(ref mark);
                Collision.HoldShells(bodies);
                Collision.Collect(bodies);
                Collision.Solve(bodies, stepSeconds);
                collision += TicksSince(ref mark);
            }

            Collision.Collect(bodies);
            Collision.Solve(bodies, stepSeconds);
            Collision.HoldShells(bodies);
            Collision.PinOpenSheets(bodies);
            collision += TicksSince(ref mark);

            for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++) UpdateVelocities(bodies[bodyIndex], stepSeconds);
            velocity += TicksSince(ref mark);
            Collision.ApplyRestitution(bodies);
            collision += TicksSince(ref mark);

            int particles = 0;
            for (int bodyIndex = 0; bodyIndex < bodies.Count; bodyIndex++) {
                if(bodies[bodyIndex].Particles != null) particles += bodies[bodyIndex].Particles.Length;
            }

            LastProfile = new StepProfile {
                PredictMilliseconds = Milliseconds(predict),
                ConstraintMilliseconds = Milliseconds(constraints),
                CollisionMilliseconds = Milliseconds(collision),
                VelocityMilliseconds = Milliseconds(velocity),
                BodyCount = bodies.Count,
                ParticleCount = particles
            };
        }

        static long TicksSince(ref long mark) {
            long now = Stopwatch.GetTimestamp();
            long elapsed = now - mark;
            mark = now;
            return elapsed;
        }

        static float Milliseconds(long ticks) { return (float)(ticks * 1000.0 / Stopwatch.Frequency); }

        static void Predict(DeformableBody body, float stepSeconds) {
            Particle[] particles = body.Particles;
            Vec3 gravity = body.Material.Gravity;
            float damping = (float)Math.Exp(-body.Material.Damping * stepSeconds);
            for (int i = 0; i < particles.Length; i++) {
                Particle particle = particles[i];
                if(particle.InverseMass <= 0f) {
                    particle.Force = Vec3.Zero;
                    particle.Previous = particle.Position;
                    particles[i] = particle;
                    continue;
                }

                particle.Force += gravity * (1f / particle.InverseMass);
                particle.Velocity += particle.Force * (particle.InverseMass * stepSeconds);
                particle.Velocity *= damping;
                particle.Previous = particle.Position;
                particle.Position += particle.Velocity * stepSeconds;
                particle.Force = Vec3.Zero;
                particles[i] = particle;
            }
        }

        static void UpdateVelocities(DeformableBody body, float stepSeconds) {
            Particle[] particles = body.Particles;
            float inverseDt = 1f / stepSeconds;
            for (int i = 0; i < particles.Length; i++) {
                Particle particle = particles[i];
                if(particle.InverseMass <= 0f) {
                    particle.Velocity = Vec3.Zero;
                    particle.Force = Vec3.Zero;
                    particles[i] = particle;
                    continue;
                }

                particle.Velocity = (particle.Position - particle.Previous) * inverseDt;
                particle.Force = Vec3.Zero;
                particles[i] = particle;
            }
        }

        static void ResetConstraints(DeformableBody body) {
            List<IConstraint> constraints = body.Constraints;
            for (int i = 0; i < constraints.Count; i++) constraints[i].ResetLambda();
        }

        static void SolveBody(DeformableBody body, float stepSeconds) {
            List<IConstraint> constraints = body.Constraints;
            for (int i = 0; i < constraints.Count; i++) constraints[i].Solve(body, stepSeconds);
        }
    }
}
