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
    public ChassisController From;
    public Transform To;
    FastPlanner fastPlanner;
    List<System.Numerics.Vector2> Path;
    public float time;
    public bool Show = true;
    public float DownSampleParam;
    float localTime_;

    public void Import()
    {
        fastPlanner = new(ESDF_Path, Dijkstar_Path);
    }

    public void Test()
    {
        var t = fastPlanner.GetVelocity(localTime_);
        From.TargetVel = new UnityEngine.Vector3(-t.Y, 0, t.X);
        t = fastPlanner.GetAcc(localTime_);
        From.TargetAcc = new UnityEngine.Vector3(-t.Y, 0, t.X);
        localTime_ += Time.deltaTime;
        if (!fastPlanner.Update(localTime_))
        {
            fastPlanner.PathFinder(
                new System.Numerics.Vector2(From.transform.position.z, -From.transform.position.x),
                new System.Numerics.Vector2(To.position.z, -To.position.x),
                new System.Numerics.Vector2(From.CurrentVelocity.z, -From.CurrentVelocity.x),
                new System.Numerics.Vector2(From.CurrentAcc.z, -From.CurrentAcc.x),
                out Path,
                DownSampleParam
            );
            localTime_ = Time.deltaTime;
        }

    }
    void Start()
    {
        Import();
        fastPlanner.PathFinder(
            new System.Numerics.Vector2(From.transform.position.z, -From.transform.position.x),
            new System.Numerics.Vector2(To.position.z, -To.position.x),
            new System.Numerics.Vector2(From.CurrentVelocity.z, -From.CurrentVelocity.x),
            new System.Numerics.Vector2(From.CurrentAcc.z, -From.CurrentAcc.x),
            out Path,
            DownSampleParam
        );
        localTime_ = Time.deltaTime;
    }
    void Update()
    {
        fastPlanner.time = time;
        Test();
    }

    void OnDrawGizmosSelected()
    {
        if (!Show)
            return;
        if (fastPlanner == null)
            return;
        if (fastPlanner.Astar_.CostMap == null)
            return;
        for (int i = 0; i < fastPlanner.Astar_.CostMap.SizeX; i++)
        {
            for (int j = 0; j < fastPlanner.Astar_.CostMap.SizeY; j++)
            {
                var p = fastPlanner.Astar_.CostMap.XY2Vector2(i, j);
                var p1 = new UnityEngine.Vector3(-p.Y, 5, p.X);
                Gizmos.color = UnityEngine.Color.red + new UnityEngine.Color(-1, 1, 0, 0) * fastPlanner.Astar_.CostMap[i, j] / 100.0f;
                Gizmos.DrawCube(p1, fastPlanner.Astar_.CostMap.Resolution * UnityEngine.Vector3.one);
            }
        }

    }
    void OnDrawGizmos()
    {

        if (Path == null)
            return;
        for (int i = 0; i < Path.Count; i++)
        {
            Gizmos.DrawSphere(new UnityEngine.Vector3(-Path[i].Y, 6, Path[i].X), 0.05f);
        }
        for (int i = 1; i < Path.Count; i++)
        {
            Gizmos.DrawLine(new UnityEngine.Vector3(-Path[i].Y, 6, Path[i].X), new UnityEngine.Vector3(-Path[i - 1].Y, 6, Path[i - 1].X));
        }
    }
}