# Soft-body physics

A particle XPBD solver for Unity 6 and the Universal Render Pipeline. Add Soft Body Simulation, add Soft Body, and press Play.

The material type is `BodyMaterial`.

## Install

Add `com.softbodyphysics` as a local package, then import Samples from the Package Manager. Open a scene in that sample's `Scenes` folder and press Play. 

Requires Universal Render Pipeline 17.6.0. Importing the samples also needs Input System 1.20.0. The solver does not.

The publisher is lunargator. Support is lunakhudman@gmail.com.

## Samples

The scenes are Fish Bowl, Plate, and Material Comp. Press **T** in any of them for the technical view. Keys 1 through 6 toggle each layer while that view is open.

**Fish Bowl.** **A** and **D** orbit. **W** and **S** tilt. **F** shoves the bodies. **R** toggles the smooth surface.

**Plate.** **A** and **E** orbit. **W** and **S** tilt. **D** drops one sphere, **F** drops three, and **G** drops ten. **R** toggles the smooth surface.

**Material Comp.** The preset spheres and a jelly cube drop onto the floor. **F** switches between full speed and 0.25x. **R** starts the drop again.

## Quick start

1. Add **Soft Body Simulation** to any GameObject and set stats (every body in the scene shares these). The sample prefab `SoftBodySimulation` already uses 4 iterations, one step per frame, and a 1/60 second step.
2. Create a Unity cube, scale it into a floor. A collider created after play starts is copied after `SoftBodyColliders.Invalidate`.
3. Create another GameObject and add **Soft Body**, or drag in a sample prefab (e.g. `SoftBodySphere` or `SoftBodyCube`). 
4. Press Play. The body builds itself and falls under the simulation gravity. Smooth Surface draws a Catmull-Rom skin through the same particles. 
5. Press **T** in a sample scene for particles, distance constraints, contacts, normals, and the last step time. Keys 1 through 6 toggle each layer while that view is open.

`AddForce` and `AddImpulse` are the total for the whole body, split across the moving particles.

**Soft Body**

- Shape is Sphere, Cube, or Mesh. Sphere and cube show the size. Mesh asks for a readable mesh.
- Material is a preset, or Custom. A preset fills stiffness, damping, friction, and bounce. Stiffness is 0 (soft) through 1 (firm).
- Surface Override writes friction and bounce through preset or code. 
- Simulation is mass, simulate, kinematic, and this body's gravity scale. Simulate turns the body off without removing the component. Kinematic holds it in place.
- Reset On Enable rebuilds the body when play starts.

**Soft Body Simulation**

Every body in the scene shares these: 

- Gravity is the world acceleration. 
- Solver iterations are the constraint passes inside one step (defaults to 4). 
- Fixed timestep is the solver step in seconds. The default is 1/60.
- Max steps per frame is how many of those steps one rendered frame may run (defaults to 1). 
- The frame rate is the player's frame rate.

A rendered frame spends its time in fixed steps, up to the step cap. 

## Presets

| Preset | Stiffness | Damping | Friction | Bounce |
| --- | --- | --- | --- | --- |
| Firm Elastic | 0.98 | 0.20 | 0.45 | 0.25 |
| Rubber | 0.80 | 0.05 | 0.70 | 0.85 |
| Gelatin | 0.42 | 0.08 | 0.15 | 0.32 |
| Soft Foam | 0.18 | 0.82 | 0.55 | 0.00 |
| Jelly | 0.05 | 0.12 | 0.05 | 0.08 |
| Slime | 0.00 | 0.70 | 0.85 | 0.00 |

## Runtime API

On `SoftBody`:

- `Initialize` builds the particles.
- `ApplyPreset`, `SetStiffness`, `SetDamping`, `SetFriction`, `SetRestitution` change the material. The setters switch the preset to Custom.
- `AddForce` and `AddImpulse` take a total force or a total impulse.
- `SetSimulate` and `SetKinematic` change participation.
- `ResetSimulation` returns particles to the rest pose.
- `GetParticlePositions` reads the current shape.

`SoftBodySimulation.World` is the shared solver. `Pause` and `Resume` stop and start stepping.

## Mesh bodies

Set the shape to Mesh and assign a readable mesh. Vertices become particles. Duplicate vertices weld. Mesh Cell Size welds nearby vertices into the same particle; 0 welds duplicates only. Turn on Read/Write on the model importer. UVs and materials stay on the source mesh and are copied onto the simulated surface.

A closed mesh keeps its volume while an open mesh does not. 

## Colliders

Static colliders and kinematic rigidbodies are copied into the soft-body world. Triggers, non-kinematic rigidbodies, and colliders on a soft body are ignored. Supported shapes are box, sphere, capsule, and mesh. A mesh collider needs a readable mesh. 

Assign Collider Root on Soft Body Simulation to copy only that hierarchy. With the root empty, the first sync caches every collider in the scene and later syncs reuse that list. Call `SoftBodyColliders.Invalidate` after a collider is created or destroyed. A mesh is read once and again only when its transform or mesh changes. Moving a kinematic collider still updates its pose every sync.

