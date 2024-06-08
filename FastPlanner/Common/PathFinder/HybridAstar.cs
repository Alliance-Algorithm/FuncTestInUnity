using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using System.Runtime.Serialization.Json;
using ROS2;
using Unity.VisualScripting;
public class HybridAstar
{
    PriorityQueue<Node3, float> openList;
    bool[,] closeGrid;

    public ESDFMap CostMap;

    public HybridAstar(string ESDF_Path)
    {
        DataContractJsonSerializer serializer = new DataContractJsonSerializer(typeof(ESDFMap));
        var f = File.OpenRead(ESDF_Path);
        CostMap = (ESDFMap)serializer.ReadObject(f);
        f.Close();
        CostMap.Build();
    }
    public void DownSample(float DownSampleParam, ref List<Vector2> Path)
    {
        List<Vector2> tmp = new() { Path[0] };
        for (int i = 5, l = 4, k = Path.Count - 5; i < k; i++)
        {
            var k1_t = Path[i - 5] + Path[i - 4] + Path[i - 3] - Path[i - 2] - Path[i - 1] - Path[i];
            var k1 = Math.Atan(k1_t.Y / k1_t.X) + (k1_t.X < 0 ? 180 : 0);
            var k2_t = Path[i] + Path[i + 1] + Path[i + 2] - Path[i + 3] - Path[i + 4] - Path[i + 5];
            var k2 = Math.Atan(k2_t.Y / k2_t.X) + (k2_t.X < 0 ? 180 : 0);
            k1 = k1 > 180 ? 360 - k1 : k1;
            k2 = k2 > 180 ? 360 - k2 : k2;
            k1 = k1 < 0 ? 360 + k1 : k1;
            k2 = k2 < 0 ? 360 + k2 : k2;
            var k3 = Math.Abs(k1 - k2) * 57.29578f;
            k3 = k3 > 180 ? 360 - k3 : k3;
            if (k3 < DownSampleParam)
            {
                if (i != l + 1)
                {
                    var v = Path[l];
                    for (int j = l + 1; j < i; j++)
                        v += Path[j];
                    tmp.Add(v / (i - l));
                }
                l = i;
                continue;
            }
        }
        tmp.Add(Path[Path.Count - 1]);
        Path = tmp;
    }


    public bool Search((Vector2 Pos, float Theta) From, (Vector2 Pos, float Theta) To, float MaxDistance, out List<Vector2> Path)
    {
        Path = new();
        closeGrid = new bool[CostMap.SizeX, CostMap.SizeY];
        openList = new();
        Node3 from = new(From.Pos, From.Theta, null, 0);
        Node3 to = new(To.Pos, To.Theta, null, 0);
        openList.Enqueue(from, 0);
        while (openList.Count > 0)
        {
            var current = openList.Dequeue();
            var xy = CostMap.Vector22XY(current.Pos);
            if (closeGrid[xy.x, xy.y])
                continue;
            closeGrid[xy.x, xy.y] = true;
            UnityEngine.Debug.DrawLine(new(-current.Pos.Y, 6.1f, current.Pos.X), new(-current.Pos.Y + 0.1f, 6.1f, current.Pos.X + 0.1f), UnityEngine.Color.red, 1);
            if (current == to)
            {
                to.Parent = current.Parent;
                break;
            }
            var children = current.ChildsGen();
            foreach (var child in children)
            {
                if (child.G > MaxDistance)
                    continue;
                xy = CostMap.Vector22XY(child.Pos);
                if (closeGrid[xy.x, xy.y])
                    continue;
                if (CostMap[xy.x, xy.y] <= 0)
                    continue;
                openList.Enqueue(child, child.CalcF(to, CostMap));
            }
        }

        if (to.Parent == null)
            return false;
        else
        {
            var k = to;
            Path.Add(to.Pos);
            while (k.Parent != null)
            {
                k = k.Parent;
                Path.Add(k.Pos);
            }
            Path.Reverse();
            return true;
        }
    }

    public bool SearchWithDownSample((Vector2 Pos, float Theta) From, (Vector2 Pos, float Theta) To, float MaxDistance, out List<Vector2> Path, float DownSampleParam)
    {
        if (Search(From, To, MaxDistance, out Path))
            DownSample(DownSampleParam, ref Path);
        else return false;
        return true;
    }
}
