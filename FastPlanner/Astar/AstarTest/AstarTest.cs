using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Numerics;
using System.Runtime.Serialization.Json;
using UnityEngine;
using UnityEngine.AI;

[RequireComponent(typeof(LocalGridMapBuilder))]
class AstarTest : MonoBehaviour
{
    public string ESDF_Path;
    public Transform From;
    public Transform To;
    Astar astar;
    List<System.Numerics.Vector2> Path;
    public bool Show = true;

    public void Import()
    {
        astar = new Astar(ESDF_Path) { };
    }

    public void Test()
    {
        astar.PathFinderWithDownSample(
            new System.Numerics.Vector2(From.position.z, -From.position.x),
            new System.Numerics.Vector2(To.position.z, -To.position.x),
            50,
            out Path,
            0.2f
        );
    }

    void OnDrawGizmosSelected()
    {
        if (!Show)
            return;
        if (astar == null)
            return;
        if (astar.CostMap == null)
            return;
        for (int i = 0; i < astar.CostMap.SizeX; i++)
        {
            for (int j = 0; j < astar.CostMap.SizeY; j++)
            {
                var p = astar.CostMap.XY2Vector2(i, j);
                var p1 = new UnityEngine.Vector3(-p.Y, 5, p.X);
                Gizmos.color = UnityEngine.Color.red + new UnityEngine.Color(-1, 1, 0, 0) * astar.CostMap[i, j] / 100.0f;
                Gizmos.DrawCube(p1, astar.CostMap.Resolution * UnityEngine.Vector3.one);
            }
        }
        if (Path == null)
            return;
        for (int i = 0; i < Path.Count; i++)
        {
            Gizmos.DrawSphere(new UnityEngine.Vector3(-Path[i].Y, 6, Path[i].X), 0.05f);
        }

    }
}