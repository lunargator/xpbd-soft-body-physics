# Soft Body Physics

A surface XPBD solver for Unity 6. Distance, shape matching, and volume constraints hold a particle surface together. The body collides with planes, boxes, spheres, capsules, meshes, and other soft bodies.

Unity 6000.0.0f1 or newer. The Universal Render Pipeline 17.6.0 is a package dependency. The runtime compiles without the Input System. The samples use it.

## Install

Add `com.softbodyphysics` as a local package, then import the Samples entry from the Package Manager. The scenes are Fish Bowl and Materials, in that sample's `Scenes` folder. Open one and press Play. The sample prefabs `SoftBodySimulation`, `SoftBodySphere`, and `SoftBodyCube` are a simulation and two bodies with the default step of 4 iterations and 1/60 second. Press T in a sample for the technical view.

The menu **Tools/Soft Body Physics** can rebuild a sample scene beside the imported scripts. It does not edit Editor Build Settings.

## Support

Publisher: lunargator. lunakhudman@gmail.com

## Docs

`Documentation~/Manual.md` covers the components. `Documentation~/License.md` covers the license. `CHANGELOG.md` is the version history.


