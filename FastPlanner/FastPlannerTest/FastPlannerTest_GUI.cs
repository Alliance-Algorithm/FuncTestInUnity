using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(FastPlannerTest)), CanEditMultipleObjects]
class FastPlannerTest_GUI : Editor
{

    private void OnEnable()
    {
    }
    public override void OnInspectorGUI()
    {
        base.DrawDefaultInspector();
        if (GUILayout.Button("Import"))
        {
            ((FastPlannerTest)target).Import();
        }
        if (GUILayout.Button("Test"))
        {
            ((FastPlannerTest)target).Test();
        }
    }
}