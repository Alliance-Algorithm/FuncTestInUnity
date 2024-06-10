using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Runtime.Serialization.Json;

class FastPlanner
{
    public HybridAstar Astar_;
    public Dijkstar Dijkstra_;
    public Optimizer.BsplineOptimizer BspLine;
    public List<Vector2> WayPoints;

    public Vector2 To;
    public float time;

    public FastPlanner(string ESDF_Path, string Dijkstar_Path)
    {
        Astar_ = new HybridAstar(ESDF_Path);
        DataContractJsonSerializer serializer = new DataContractJsonSerializer(typeof(Dijkstar));
        var f = File.OpenRead(Dijkstar_Path);
        Dijkstra_ = (Dijkstar)serializer.ReadObject(f);
        Dijkstra_.VornoiBuild(Astar_.CostMap);
        BspLine = new(ESDF_Path);
        f.Close();
    }

    public void PathFinder(Vector2 From, Vector2 To, Vector2 FromVelocity, Vector2 FromAcc, out List<Vector2> WayPoints, float DownSampleParam, bool Force = false)
    {
        WayPoints = null;
        Dijkstra_.Search(From, To, out var path, Astar_.CostMap.Vector22XY);
        var w = path;
        Astar_.SearchWithDownSample((From, float.MaxValue), (w[1], float.MaxValue), (From - w[1]).Length() * 1.5f, out var path1, DownSampleParam);
        w.RemoveAt(0);
        path1.RemoveAt(path1.Count - 1);
        path1.AddRange(w);
        // WayPoints = path1;
        // w.AddRange(list_vec2[index.b]);
        // if (w.Count > 1)
        //     path1.Add(w[1]);
        this.WayPoints = WayPoints = BspLine.Build(path1, new List<Vector2> { FromVelocity, new(0, 0), FromAcc, new(0, 0) }, 4, 0.1f);
    }

    public bool Update(float t, Vector2 Position)
    {
        var k = BspLine.Update(t, Position);
        if (k.Safe)
        {
            // Dijkstra_.Obstacle[0] = new(100, 100);
            // Dijkstra_.Obstacle[1] = new(100, 100);
            return true;
        }

        // Dijkstra_.Obstacle[0] = WayPoints[k.Id];
        // Dijkstra_.Obstacle[1] = WayPoints[k.Id + 1];
        return false;
    }

    public Vector3 GetVelocity(float t) => BspLine.TargetVeclocity(t);
    public Vector3 GetAcc(float t) => BspLine.TargetAcc(t);



    // public void PointFinder(Vector2 From, Vector2 To, out List<Vector2> WayPoints, bool Force = false)
    // {
    //     SortedList<float, (int, Vector2)> priorityQueue1 = new();
    //     SortedList<float, (int, Vector2)> priorityQueue2 = new();
    //     WayPoints = new();
    //     priorityQueue1.Add((From - To).Length(), (-1, From));
    //     for (int i = 0, k = Floyd.Points.Length; i < k; i++)
    //     {
    //         priorityQueue1.Add((From - Floyd.Points[i]).Length(), (i, Floyd.Points[i]));
    //         priorityQueue2.Add((To - Floyd.Points[i]).Length(), (i, Floyd.Points[i]));
    //     }
    //     (int idx, Vector2 vec)[] vec1 = {
    //     priorityQueue1.Values[0],
    //     priorityQueue1.Values[1],
    //     priorityQueue1.Values[2],
    //     priorityQueue1.Values[3]};

    //     (int idx, Vector2 vec)[] vec2 = {
    //     priorityQueue2.Values[0],
    //     priorityQueue2.Values[1],
    //     priorityQueue2.Values[2],
    //     priorityQueue2.Values[3]};

    //     bool[] b1 = new bool[4];
    //     bool[] b2 = new bool[4];

    //     List<Vector2>[] list_vec1 = new List<Vector2>[4];
    //     List<Vector2>[] list_vec2 = new List<Vector2>[4];

    //     (int a, int b) index = (-1, -1);
    //     while (++index.a < 4 && !Astar.PathFinderWithDownSample(From, vec1[index.a].vec, 1.2f * (vec1[index.a].vec - From).Length(), out list_vec1[index.a])) ;
    //     while (++index.b < 4 && !Astar.PathFinderWithDownSample(vec2[index.b].vec, To, 1.2f * (vec2[index.b].vec - To).Length(), out list_vec2[index.b])) ;

    //     #region NOTUSE 
    //     while (index.a < 4 && index.b < 4)
    //     {
    //         Floyd.GetPath(out var path, vec1[index.a].idx, vec2[index.b].idx);

    //         var c1 = list_vec1[index.a].Count;
    //         var c2 = path.Count;
    //         var dir11 = list_vec1[index.a][c1 - 2] - list_vec1[index.a][c1 - 1];
    //         var dir21 = list_vec2[index.b][1] - list_vec2[index.b][0];
    //         var dir12 = path[1] - path[0];
    //         var dir22 = path[c2 - 2] - path[c2 - 1];
    //         dir11 = dir11 / dir11.Length();
    //         dir12 = dir12 / dir12.Length();
    //         dir21 = dir21 / dir21.Length();
    //         dir22 = dir22 / dir22.Length();
    //         if (Vector2.Dot(dir11, dir12) > 0f)
    //             while (++index.a < 4 && !Astar.PathFinderWithDownSample(From, vec1[index.a].vec, 1.2f * (vec1[index.a].vec - From).Length(), out list_vec1[index.a])) ;
    //         else if (Vector2.Dot(dir21, dir22) > 0f)
    //             while (++index.b < 4 && !Astar.PathFinderWithDownSample(vec2[index.b].vec, To, 1.2f * (vec2[index.b].vec - To).Length(), out list_vec2[index.b])) ;
    //         else
    //         {
    //             var w = path;
    //             // w.AddRange(path);
    //             // w.AddRange(list_vec2[index.b]);
    //             WayPoints = w;
    //             break;
    //         }
    //     }
    //     #endregion
    // }

}