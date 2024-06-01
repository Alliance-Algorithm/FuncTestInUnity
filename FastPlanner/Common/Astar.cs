using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Numerics;
using System.Runtime.Serialization.Json;

class Astar
{
    public ESDFMap CostMap;

    class Priority : IComparable<Priority>
    {
        public Priority(float a)
        {
            value = a;
            t = T++;
        }
        public float value;
        int t;
        static int T = 0;

        public int CompareTo(Priority other)
        {
            var i = value.CompareTo(other.value);
            return i == 0 ? t.CompareTo(other.t) : i;
        }
    }

    public Astar(string ESDF_Path)
    {
        DataContractJsonSerializer serializer = new DataContractJsonSerializer(typeof(ESDFMap));
        var f = File.OpenRead(ESDF_Path);
        CostMap = (ESDFMap)serializer.ReadObject(f);
        f.Close();
        CostMap.Build();
    }
    public void DownSample(ref List<Vector2> Path)
    {
        List<Vector2> tmp = new() { Path[0] };
        for (int i = 5, l = 4, k = Path.Count - 5; i < k; i++)
        {
            var k1_t = Path[i - 5] + Path[i - 4] + Path[i - 3] - Path[i - 2] - Path[i - 1] - Path[i];
            var k1 = Math.Atan(k1_t.Y / k1_t.X);
            var k2_t = Path[i] + Path[i + 1] + Path[i + 2] - Path[i + 3] - Path[i + 4] - Path[i + 5];
            var k2 = Math.Atan(k2_t.Y / k2_t.X);
            if (k1 - k2 < 0.1)
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

    public bool PathFinder(Vector2 From, Vector2 To, float MaxDistance, out List<Vector2> Path)
    {
        Path = new();
        bool[,] Colord = new bool[CostMap.SizeX, CostMap.SizeY];
        Vector2?[,] Parent = new Vector2?[CostMap.SizeX, CostMap.SizeY];

        var from = CostMap.Vector22XY(From);
        var to = CostMap.Vector22XY(To);

        // SortedList<Priority, (int x, int y, int g, Vector2? p)> priorityQueue = new();
        PriorityQueue<(int x, int y, int g, Vector2? p), float> priorityQueue = new();
        // priorityQueue.Add(new(0), (from.x, from.y, 0, null));
        priorityQueue.Enqueue((from.x, from.y, 0, null), 0);
        Action f = () =>
        {

            while (priorityQueue.Count > 0)
            {
                // var v = priorityQueue.Values[0];
                // priorityQueue.RemoveAt(0);
                var v = priorityQueue.Dequeue();
                var v2 = CostMap.XY2Vector2(v.x, v.y);
                UnityEngine.Debug.DrawLine(new(-v2.Y, 6.1f, v2.X), new(-v2.Y + 0.1f, 6.1f, v2.X + 0.1f), UnityEngine.Color.red, 5);
                if (Colord[v.x, v.y])
                    continue;
                Parent[v.x, v.y] = v.p;
                Colord[v.x, v.y] = true;
                for (int i = -1; i <= 1; i++)
                    for (int j = -1; j <= 1; j++)
                    {
                        var _x = v.x + i;
                        var _y = v.y + j;
                        if (i == 0 && j == 0)
                            continue;
                        if (_x == to.x && _y == to.y)
                        {
                            Parent[to.x, to.y] = new(v.x, v.y);
                            return;
                        }
                        if (Colord[_x, _y])
                            continue;
                        if (CostMap[_x, _y] == 0)
                            continue;
                        var _g = v.g + Math.Sqrt(i * i + j * j);
                        var _f = (_g + Math.Sqrt(Math.Pow(to.x - _x, 2) + Math.Pow(to.y - _y, 2))) * CostMap.Resolution;
                        if (_f > MaxDistance)
                            continue;
                        // priorityQueue.Add(new((float)_f), ((int x, int y, int g, Vector2? p))(_x, _y, _g, new Vector2(v.x, v.y)));
                        priorityQueue.Enqueue(((int x, int y, int g, Vector2? p))(_x, _y, _g, new Vector2(v.x, v.y)), (float)_f - CostMap[_x, _y] * CostMap.Resolution);
                    }
            }
        };
        f();

        if (Parent[to.x, to.y] == null)
            return false;
        else
        {
            var v = To;
            Vector2? k = Parent[to.x, to.y];
            Path.Add(v);
            while (Parent[(int)k.Value.X, (int)k.Value.Y] != null && k != Parent[(int)k.Value.X, (int)k.Value.Y])
            {
                k = Parent[(int)k.Value.X, (int)k.Value.Y];
                v = k.Value;
                Path.Add(CostMap.XY2Vector2((int)v.X, (int)v.Y));
            }
            Path.RemoveAt(Path.Count - 1);
            Path.Add(From);
            Path.Reverse();
            return true;
        }
    }

    public bool PathFinderWithDownSample(Vector2 From, Vector2 To, float MaxDistance, out List<Vector2> Path)
    {
        if (PathFinder(From, To, MaxDistance, out Path))
            DownSample(ref Path);
        else return false;
        return true;
    }
}