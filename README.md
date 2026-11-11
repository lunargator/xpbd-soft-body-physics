# Soft Body Physics

A surface-based **Extended Position Based Dynamics (XPBD)** soft-body physics solver for Unity 6. Implements deformable-body simulation using distance, shape-matching, and volume constraints, with collision handling between soft bodies and Unity colliders.

[![Soft Body Physics — Trailer](https://img.youtube.com/vi/pPrl9QOgUbo/maxresdefault.jpg)](https://www.youtube.com/watch?v=pPrl9QOgUbo)

*Watch the demonstration trailer on YouTube.*

## Features

- **XPBD Solver** — Particle-based deformable-body simulation with configurable solver iterations and timestep.
- **Distance Constraints** — Maintain structural relationships between particles.
- **Shape Matching** — Preserve body shape while allowing deformation.
- **Volume Constraints** — Resist unwanted changes in enclosed volume.
- **Collision Detection** — Supports planes, boxes, spheres, capsules, meshes, and interactions between soft bodies.
- **Unity Integration** — Reusable simulation components, collider integration, and deformable mesh rendering.
- **Technical Visualization** — Built-in visualization tools for inspecting simulation behavior.

## Architecture

The physics solver is separated into an engine-independent core and a Unity integration layer.

- **Core** — Particle simulation, XPBD constraints, and physics calculations.
- **Constraints** — Distance, shape-matching, and volume constraint implementations.
- **Collision** — Collision detection and response for supported geometry.
- **Unity Integration** — Simulation components, collider integration, and deformable mesh rendering.

## Demonstrations

| Scene | Description |
|---|---|
| Fish Bowl | Soft bodies interacting within a container. |
| Plate | Deformable bodies interacting with a surface. |
| Material Comparison | Comparison of different soft-body material properties. |

The included samples provide three reusable prefabs:

- `SoftBodySimulation` — Simulation controller.
- `SoftBodySphere` — Default soft-body sphere.
- `SoftBodyCube` — Default soft-body cube.

The default simulation uses 4 solver iterations and a timestep of 1/60 second.

Press **T** in a sample scene to toggle technical visualization.

## Requirements

- Unity 6000.0.0f1 or newer.
- Universal Render Pipeline (URP) 17.6.0.

The solver and Soft Body components compile independently of URP and the Input System.

The included samples require Input System 1.20.0 for keyboard controls. This dependency is not included in `package.json`, as it is only required by the samples.

## Installation

1. Add `com.softbodyphysics` as a local package in Unity.
2. Open **Window → Package Manager**.
3. Locate Soft Body Physics.
4. Import the included samples.
5. Open a sample scene and press Play.

## Editor Tools

The **Tools → Soft Body Physics** menu provides an option to rebuild a sample scene alongside the imported sample scripts. This does not modify Unity's Editor Build Settings.

## Documentation

- [Component Manual](Documentation~/Manual.md)
- [Changelog](CHANGELOG.md)
- [License](Documentation~/License.md)

## Support

**Publisher:** lunargator

**Contact:** lunakhudman@gmail.com