using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Numerics;
using System.Runtime.Serialization.Json;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.AI;
class FastPlannerTest : MonoBehaviour
{
    public string ESDF_Path;
    public string Dijkstar_Path;
    public Transform From;
    public Transform To;
    FastPlanner wayPointUpdater;
    List<System.Numerics.Vector2> Path;
    public bool Show = true;

    public void Import()
    {
        wayPointUpdater = new(ESDF_Path, Dijkstar_Path);
    }

    public void Test()
    {
        wayPointUpdater.PointFinder(
            new System.Numerics.Vector2(From.position.z, -From.position.x),
            new System.Numerics.Vector2(To.position.z, -To.position.x),
            out Path
        );
    }
    void Start()
    {
        Import();
    }
    void Update()
    {
        Test();
    }

    void OnDrawGizmosSelected()
    {
        if (!Show)
            return;
        if (wayPointUpdater == null)
            return;
        if (wayPointUpdater.Astar.CostMap == null)
            return;
        for (int i = 0; i < wayPointUpdater.Astar.CostMap.SizeX; i++)
        {
            for (int j = 0; j < wayPointUpdater.Astar.CostMap.SizeY; j++)
            {
                var p = wayPointUpdater.Astar.CostMap.XY2Vector2(i, j);
                var p1 = new UnityEngine.Vector3(-p.Y, 5, p.X);
                Gizmos.color = UnityEngine.Color.red + new UnityEngine.Color(-1, 1, 0, 0) * wayPointUpdater.Astar.CostMap[i, j] / 100.0f;
                Gizmos.DrawCube(p1, wayPointUpdater.Astar.CostMap.Resolution * UnityEngine.Vector3.one);
            }
        }

    }
    void OnDrawGizmos()
    {

        if (Path == null)
            return;
        for (int i = 0; i < Path.Count; i++)
        {
            Gizmos.DrawSphere(new UnityEngine.Vector3(-Path[i].Y, 6, Path[i].X), 0.2f);
        }
    }
}