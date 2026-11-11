# Soft Body Physics

A surface XPBD solver for Unity 6. Distance, shape matching, and volume constraints hold a particle surface together. The body collides with planes, boxes, spheres, capsules, meshes, and other soft bodies.

Unity 6000.0.0f1 or newer. The Universal Render Pipeline 17.6.0 is the only package dependency. The solver and the Soft Body components compile without it and without the Input System. Importing the samples needs Input System 1.20.0 as well, because Fish Bowl, Plate, and Material Comp read the keyboard through it. Unity has no way to attach that requirement to the sample alone, so it stays out of `package.json`.

## Install

Add `com.softbodyphysics` as a local package, then import the Samples entry from the Package Manager. The scenes are Fish Bowl, Plate, and Material Comp, in that sample's `Scenes` folder. Open one and press Play. The sample prefabs `SoftBodySimulation`, `SoftBodySphere`, and `SoftBodyCube` are a simulation and two bodies with the default step of 4 iterations and 1/60 second. Press T in a sample for the technical view.

The menu **Tools/Soft Body Physics** can rebuild a sample scene beside the imported scripts. It does not edit Editor Build Settings.

## Benchmarks

These times are one solver step. The body is a single sphere with no colliders. The step is 1/60 second, with 8 warmup steps and an average of the next 30. Measured in the editor on an Apple M5 Max, 18 cores, 64 GB, macOS 27.0.1, Unity 6000.6.5f1.

More particles cost more. At 4 iterations: 5 particles and 11 constraints take 0.019 ms, 20 and 56 take 0.080 ms, 42 and 122 take 0.167 ms, and 86 and 254 take 0.347 ms.

More constraints cost more at the same 20 particles and 4 iterations. The constraint pass is 0.053 ms at 56 constraints, 0.073 ms at 96, and 0.118 ms at 176.

More iterations cost more at the same 20 particles and 56 constraints. One iteration takes 0.031 ms, two take 0.046 ms, four take 0.078 ms, and eight take 0.147 ms.

## Support

Publisher: lunargator. lunakhudman@gmail.com

## Docs

`Documentation~/Manual.md` covers the components. `Documentation~/License.md` covers the license. `CHANGELOG.md` is the version history.


