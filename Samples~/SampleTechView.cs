using System.Text;
using SoftBodyPhysics.Core;
using SoftBodyPhysics.Unity;
using UnityEngine;
using UnityEngine.InputSystem;

namespace SoftBodyPhysics.Samples {

    public sealed class SampleTechView : MonoBehaviour {
        public bool visible;

        PhysicsDebugRenderer lines;
        readonly StringBuilder buffer = new StringBuilder(256);
        string shown = "T  technical view";
        int shownParticles = -1;
        int shownConstraints = -1;
        int shownContacts = -1;
        int shownIterations = -1;
        int shownMilliseconds = -1;
        int shownFlags = -1;

        public static void Ensure(MonoBehaviour owner) {
            if(owner == null) return;
            if(owner.GetComponentInChildren<SampleTechView>(true) != null) return;

            var host = new GameObject("Technical View");
            host.transform.SetParent(owner.transform, false);
            host.hideFlags = HideFlags.DontSave;
            host.AddComponent<SampleTechView>();
        }

        void Update() {
            if(!Application.isPlaying) return;
            Keyboard keyboard = Keyboard.current;
            if(keyboard != null && keyboard[Key.T].wasPressedThisFrame) visible = !visible;
            if(lines != null) {
                lines.enabled = visible;
                MeshRenderer renderer = lines.GetComponent<MeshRenderer>();
                if(renderer != null) renderer.enabled = visible;
            }
        }

        void LateUpdate() {
            if(!Application.isPlaying || !visible) return;
            SoftBodySimulation simulation = SoftBodySimulation.Active;
            if(simulation == null) return;
            EnsureLines();
            lines.enabled = true;
            lines.Publish(simulation.World);
            Refresh(simulation);
        }

        void OnGUI() {
            if(!Application.isPlaying || !visible) return;
            GUI.Box(new Rect(Screen.width - 312f, 12f, 300f, 204f), shown);
        }

        void EnsureLines() {
            if(lines != null) return;
            lines = GetComponent<PhysicsDebugRenderer>();
            if(lines == null) lines = gameObject.AddComponent<PhysicsDebugRenderer>();
            lines.ShowKeyList = false;
            lines.ShowParticles = true;
            lines.ShowDistanceConstraints = false;
            lines.ShowCenterOfMass = false;
            lines.ShowContacts = true;
            lines.ShowContactNormals = true;
            lines.ShowWireframe = false;
        }

        void Refresh(SoftBodySimulation simulation) {
            PhysicsWorld world = simulation.World;
            StepProfile profile = world.LastProfile;
            int constraints = 0;
            for (int i = 0; i < world.Bodies.Count; i++) {
                DeformableBody body = world.Bodies[i];
                if(body != null) constraints += body.Constraints.Count;
            }

            int contacts = world.Collision.ContactCount;
            int milliseconds = (int)(profile.TotalMilliseconds * 1000f);
            int flags = (lines.ShowParticles ? 1 : 0)
                | (lines.ShowDistanceConstraints ? 2 : 0)
                | (lines.ShowCenterOfMass ? 4 : 0)
                | (lines.ShowContacts ? 8 : 0)
                | (lines.ShowContactNormals ? 16 : 0)
                | (lines.ShowWireframe ? 32 : 0);
            if(shownParticles == profile.ParticleCount
                && shownConstraints == constraints
                && shownContacts == contacts
                && shownIterations == simulation.solverIterations
                && shownMilliseconds == milliseconds
                && shownFlags == flags)
                return;

            shownParticles = profile.ParticleCount;
            shownConstraints = constraints;
            shownContacts = contacts;
            shownIterations = simulation.solverIterations;
            shownMilliseconds = milliseconds;
            shownFlags = flags;
            buffer.Clear();
            buffer.Append("Technical view");
            buffer.Append('\n');
            buffer.Append("1 particles   ");
            buffer.Append(lines.ShowParticles ? "on" : "off");
            buffer.Append('\n');
            buffer.Append("2 distances   ");
            buffer.Append(lines.ShowDistanceConstraints ? "on" : "off");
            buffer.Append('\n');
            buffer.Append("3 center      ");
            buffer.Append(lines.ShowCenterOfMass ? "on" : "off");
            buffer.Append('\n');
            buffer.Append("4 contacts    ");
            buffer.Append(lines.ShowContacts ? "on" : "off");
            buffer.Append('\n');
            buffer.Append("5 normals     ");
            buffer.Append(lines.ShowContactNormals ? "on" : "off");
            buffer.Append('\n');
            buffer.Append("6 wireframe   ");
            buffer.Append(lines.ShowWireframe ? "on" : "off");
            buffer.Append('\n');
            buffer.Append("Iterations ");
            buffer.Append(simulation.solverIterations);
            buffer.Append('\n');
            buffer.Append("Particles ");
            buffer.Append(profile.ParticleCount);
            buffer.Append("   constraints ");
            buffer.Append(constraints);
            buffer.Append('\n');
            buffer.Append("Contacts ");
            buffer.Append(contacts);
            buffer.Append('\n');
            buffer.Append("Step ");
            buffer.Append(profile.TotalMilliseconds.ToString("0.000"));
            buffer.Append(" ms");
            shown = buffer.ToString();
        }
    }
}
