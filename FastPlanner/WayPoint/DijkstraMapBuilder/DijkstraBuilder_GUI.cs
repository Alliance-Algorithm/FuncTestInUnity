
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(DijkstraBuilder)), CanEditMultipleObjects]
class DijkstraBuilder_GUI : Editor
{
    private void OnEnable()
    {
    }
    public override void OnInspectorGUI()
    {
        base.DrawDefaultInspector();
        if (GUILayout.Button("Bake"))
        {
            ((DijkstraBuilder)target).Build();
        }
        if (GUILayout.Button("Test"))
        {
            ((DijkstraBuilder)target).Test();
        }
    }

}