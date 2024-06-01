using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Runtime.Serialization.Json;
using UnityEngine.Windows.WebCam;

class FastPlanner
{
    public Astar Astar;
    public Floyd Floyd;

    public Vector2 To;
    public FastPlanner(string ESDF_Path, string FloydPath)
    {
        Astar = new Astar(ESDF_Path);
        DataContractJsonSerializer serializer = new DataContractJsonSerializer(typeof(Floyd));
        var f = File.OpenRead(FloydPath);
        Floyd = (Floyd)serializer.ReadObject(f);
        Floyd.VornoiBuild(Astar.CostMap);
        f.Close();
    }

    public void PointFinder(Vector2 From, Vector2 To, out List<Vector2> WayPoints, bool Force = false)
    {
        WayPoints = null;
        SortedList<float, (int idx, Vector2 pos)> priorityQueue1 = new();
        SortedList<float, (int idx, Vector2 pos)> priorityQueue2 = new();
        var from = Astar.CostMap.Vector22XY(From);
        var to = Astar.CostMap.Vector22XY(To);
        for (int i = 0, f = Floyd.Voronoi[from.x, from.y], t = Floyd.Voronoi[to.x, to.y], k = Floyd.Points.Length; i < k; i++)
        {
            if (Floyd.Access[f, i])
                priorityQueue1.Add((From - Floyd.Points[i]).Length(), (i, Floyd.Points[i]));
            if (Floyd.Access[i, t])
                priorityQueue2.Add((To - Floyd.Points[i]).Length(), (i, Floyd.Points[i]));
        }

        (int a, int b) index = new(-1, -1);
        var list_vec1 = priorityQueue1.Values;
        var list_vec2 = priorityQueue2.Values;
        var lc1 = priorityQueue1.Values.Count;
        var lc2 = priorityQueue2.Values.Count;
        bool upa = true, upb = true;
        List<Vector2> path = new();
        List<Vector2> pathf = new();
        List<Vector2> patht = new();
        while (index.a < lc1 && index.b < lc2)
        {
            while (upa && ++index.a < lc1 && !Astar.PathFinderWithDownSample(From, list_vec1[index.a].pos, 1.2f * (list_vec1[index.a].pos - From).Length(), out pathf)) ;
            while (upb && ++index.b < lc2 && !Astar.PathFinderWithDownSample(list_vec2[index.b].pos, To, 1.2f * (list_vec2[index.b].pos - To).Length(), out patht)) ;
            upa = upb = false;
            if (index.a >= lc1 || index.b >= lc2)
            {
                if (priorityQueue1.Count > 0 && priorityQueue2.Count > 0)
                    Floyd.GetPath(out path, priorityQueue1.Values[0].idx, priorityQueue2.Values[0].idx);
                break;
            }

            Floyd.GetPath(out path, list_vec1[index.a].idx, list_vec2[index.b].idx);

            var c1 = pathf.Count;
            var c2 = path.Count;
            var dir11 = pathf[c1 - 2] - pathf[c1 - 1];
            var dir21 = patht[1] - patht[0];
            var dir12 = path[1] - path[0];
            var dir22 = path[c2 - 2] - path[c2 - 1];
            dir11 = dir11 / dir11.Length();
            dir12 = dir12 / dir12.Length();
            dir21 = dir21 / dir21.Length();
            dir22 = dir22 / dir22.Length();
            if (Vector2.Dot(dir11, dir12) > 0f) upa = true;
            else if (Vector2.Dot(dir21, dir22) > 0f) upb = true;
            else
            {
                break;
            }
        }
        var w = path;
        // w.AddRange(path);
        // w.AddRange(list_vec2[index.b]);
        WayPoints = w;
    }

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