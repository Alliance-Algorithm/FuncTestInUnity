using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

class WayPoints : MonoBehaviour
{
    public List<WayPoints> ToPoints;

    public bool DebugShow = true;

    void OnDrawGizmosSelected()
    {
        if (!DebugShow)
            return;
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(transform.position, 0.4f);

        // Gizmos.color = Color.red;
        // foreach (var i in ToPoints)
        // {
        //     Gizmos.DrawSphere(i.transform.position, 0.4f);
        //     Gizmos.DrawLine(transform.position, i.transform.position);
        //     Gizmos.DrawSphere(transform.position + (i.transform.position - transform.position) * 0.1f, 0.1f);
        // }
        Gizmos.color = Color.blue;

        foreach (var i in ToPoints)
        {
            Gizmos.DrawLine(transform.position, i.transform.position);
            Gizmos.DrawSphere(transform.position + (i.transform.position - transform.position) * 0.1f, 0.1f);
        }
    }
    void OnDrawGizmos()
    {
        Handles.Label(transform.position + new Vector3(0.5f, 0, 0.5f), name);
        Gizmos.color = Color.white;
        Gizmos.DrawWireSphere(transform.position, 0.5f);

    }
}