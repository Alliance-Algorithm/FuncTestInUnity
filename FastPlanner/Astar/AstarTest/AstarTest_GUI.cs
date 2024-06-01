using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(AstarTest)), CanEditMultipleObjects]
class AstarTest_GUI : Editor
{

    private void OnEnable()
    {
    }
    public override void OnInspectorGUI()
    {
        base.DrawDefaultInspector();
        if (GUILayout.Button("Import"))
        {
            ((AstarTest)target).Import();
        }
        if (GUILayout.Button("Test"))
        {
            ((AstarTest)target).Test();
        }
    }
}