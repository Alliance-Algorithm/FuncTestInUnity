using System;
using System.Collections.Generic;
using System.Numerics;

class Astar
{
    public ESDFMap CostMap;

    public void DownSample(ref List<Vector2> Path)
    {
        List<Vector2> tmp = new() { Path[0] };
        for (int i = 5, k = Path.Count - 5; i < k; i++)
        {
            var k1_t = Path[i - 5] + Path[i - 4] + Path[i - 3] - Path[i - 2] - Path[i - 1] - Path[i];
            var k1 = Math.Atan(k1_t.Y / k1_t.X);
            var k2_t = Path[i] + Path[i + 1] + Path[i + 2] - Path[i + 3] - Path[i + 4] - Path[i + 5];
            var k2 = Math.Atan(k2_t.Y / k2_t.X);
            if (k1 - k2 < 0.15)
                continue;
            Path.Add(Path[i]);
        }
        Path.Add(Path[Path.Count - 1]);
    }

    public bool PathFinder(Vector2 From, Vector2 To, float MaxDistance, out List<Vector2> Path)
    {
        Path = new();
        bool[,] Colord = new bool[CostMap.SizeX, CostMap.SizeY];
        Vector2?[,] Parent = new Vector2?[CostMap.SizeX, CostMap.SizeY];

        var from = CostMap.Vector22XY(From);
        var to = CostMap.Vector22XY(To);


        PriorityQueue<(int x, int y, double g, Vector2? p), float> priorityQueue = new();
        priorityQueue.Enqueue((from.x, from.y, 0, null), 0);
        Action f = () =>
        {

            while (priorityQueue.Count > 0)
            {
                var v = priorityQueue.Dequeue();
                if (Colord[v.item.x, v.item.y])
                    continue;
                Parent[v.item.x, v.item.y] = v.item.p;
                Colord[v.item.x, v.item.y] = true;
                for (int i = -1; i <= 1; i++)
                    for (int j = -1; j <= 1; j++)
                    {
                        var _x = v.item.x + i;
                        var _y = v.item.y + j;
                        if (_x == to.x && _y == to.y)
                        {
                            Parent[to.x, to.y] = new(_x, _y);
                            return;
                        }
                        if (Colord[_x, _y])
                            continue;
                        var _g = v.item.g + Math.Sqrt(i * i + j * j);
                        var _f = _g + Math.Sqrt(Math.Pow(to.x - _x, 2) + Math.Pow(to.y - _y, 2));
                        if (_f > MaxDistance)
                            continue;
                        priorityQueue.Enqueue((_x, _y, _g, new Vector2(_x, _y)), _f);
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
                Path.Add(v);
            }
            Path.RemoveAt(Path.Count - 1);
            Path.Add(From);
            Path.Reverse();
            return true;
        }
    }

    public void PathFinderWithDownSample(Vector2 From, Vector2 To, float MaxDistance, out List<Vector2> Path)
    {
        PathFinder(From, To, MaxDistance, out Path);
        DownSample(ref Path);
    }
}