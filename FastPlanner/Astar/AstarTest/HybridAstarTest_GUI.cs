using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(HybridAstarTest)), CanEditMultipleObjects]
class HybridAstarTest_GUI : Editor
{

    private void OnEnable()
    {
    }
    public override void OnInspectorGUI()
    {
        base.DrawDefaultInspector();
        if (GUILayout.Button("Import"))
        {
            ((HybridAstarTest)target).Import();
        }
        if (GUILayout.Button("Test"))
        {
            ((HybridAstarTest)target).Test();
        }
    }
}