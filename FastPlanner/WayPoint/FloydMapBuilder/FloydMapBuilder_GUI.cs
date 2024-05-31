
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(FloydMapBuilder)), CanEditMultipleObjects]
class FloydMapBuilder_GUI : Editor
{
    private void OnEnable()
    {
    }
    public override void OnInspectorGUI()
    {
        base.DrawDefaultInspector();
        if (GUILayout.Button("Bake"))
        {
            ((FloydMapBuilder)target).Build();
        }
        if (GUILayout.Button("Test"))
        {
            ((FloydMapBuilder)target).Test();
        }
    }

}