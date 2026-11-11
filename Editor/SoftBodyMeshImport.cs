using SoftBodyPhysics.Unity;
using UnityEditor;
using UnityEngine;

namespace SoftBodyPhysics.Editor {

    public sealed class SoftBodyMeshImport : AssetPostprocessor {
        void OnPreprocessModel() {
            var importer = (ModelImporter)assetImporter;
            importer.isReadable = true;
        }

        [InitializeOnLoadMethod]
        static void Hook() { SoftBodyColliders.PrepareMesh = EnableReadWrite; }

        static bool EnableReadWrite(Mesh mesh) {
            string path = AssetDatabase.GetAssetPath(mesh);
            if(string.IsNullOrEmpty(path)) return false;
            var importer = AssetImporter.GetAtPath(path) as ModelImporter;
            if(importer == null) return false;
            if(importer.isReadable) return true;
            importer.isReadable = true;
            importer.SaveAndReimport();
            return true;
        }
    }
}
