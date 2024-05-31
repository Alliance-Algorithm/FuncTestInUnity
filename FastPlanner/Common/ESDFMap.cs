
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;

[Serializable]
class ESDFMap
{
    public float MaxLength = 0.8f;
    public int SizeX;
    public int SizeY;
    public float Resolution;
    public sbyte[] StaticMapForSave;
    [NonSerialized] sbyte[,] StaticMap;
    [NonSerialized] sbyte[,] Map;
    [NonSerialized] bool[,] Colord;
    [NonSerialized] int[,,] StaticObs;
    [NonSerialized] int[,,] Obs;
    public sbyte[,] OutputMap => Map;

    public sbyte this[int x, int y]
    {
        get => Map == null ? (sbyte)-1 : Map[x, y];
        set
        {
            if (Map == null)
            {
                Map = new sbyte[SizeX, SizeY];
                StaticObs = new int[SizeX, SizeY, 2];
                Colord = new bool[SizeX, SizeY];
            }
            Map[x, y] = value;
        }
    }

    public Vector2 XY2Vector2(int x, int y) => new Vector2(-y * Resolution + (SizeY - 1) * Resolution / 2.0f, -x * Resolution + (SizeX - 1) * Resolution / 2.0f);
    public (int x, int y) Vector22XY(Vector2 pos)
    {
        return new((int)Math.Round(-(pos.Y - ((SizeX - 1) * Resolution / 2.0f)) / Resolution),
         (int)Math.Round(-(pos.X - ((SizeY - 1) * Resolution / 2.0f)) / Resolution));
    }

    public void Build()
    {
        Queue<(int x, int y)> openList = new Queue<(int x, int y)>();
        if (StaticMap == null)
        {
            StaticMap = new sbyte[SizeX, SizeY];
            Map = new sbyte[SizeX, SizeY];
            StaticObs = new int[SizeX, SizeY, 2];
            Obs = new int[SizeX, SizeY, 2];
            Colord = new bool[SizeX, SizeY];
            Colord = new bool[SizeX, SizeY];
        }
        for (int i = 0; i < SizeX; i++)
            for (int j = 0; j < SizeY; j++)
            {
                StaticMap[i, j] = StaticMapForSave[i * SizeY + j];
            }
        for (int i = 0; i < SizeX; i++)
            for (int j = 0; j < SizeY; j++)
            {
                if (StaticMap[i, j] == 0)
                {
                    StaticObs[i, j, 0] = i;
                    StaticObs[i, j, 1] = j;
                    Colord[i, j] = true;
                    openList.Enqueue(new(i, j));
                }
                else
                {
                    StaticObs[i, j, 0] = -1;
                    StaticObs[i, j, 1] = -1;
                    StaticMap[i, j] = 100;
                }
            }

        while (openList.Count > 0)
        {
            var c = openList.Dequeue();
            for (int i = -1; i <= 1; i++)
                for (int j = -1; j <= 1; j++)
                {
                    var ci = c.x + i;
                    var cj = c.y + j;
                    if (i == j)
                        continue;
                    if (ci < 0 || ci >= SizeX)
                        continue;
                    if (cj < 0 || cj >= SizeY)
                        continue;

                    // for calculate
                    (int x, int y) cObs = (StaticObs[ci, cj, 0], StaticObs[ci, cj, 1]);
                    if (cObs.x != -1)
                    {
                        var m = (sbyte)(Math.Clamp(Math.Round(Math.Sqrt(Math.Pow(cObs.x - c.x, 2) + Math.Pow(cObs.y - c.y, 2))) * Resolution / MaxLength, 0, 1) * 100);
                        if (StaticMap[c.x, c.y] > m)
                        {
                            StaticMap[c.x, c.y] = m;
                            StaticObs[c.x, c.y, 0] = cObs.x;
                            StaticObs[c.x, c.y, 1] = cObs.y;
                        }

                    }
                    // for enqueue
                    if (Colord[ci, cj])
                        continue;

                    Colord[ci, cj] = true;
                    openList.Enqueue((ci, cj));
                }
        }
    }

    public void Update(int offsetX, int offsetY, sbyte[,] dynamicMap)
    {
        Buffer.BlockCopy(StaticMap, 0, Map, 0, Map.Length * sizeof(sbyte));
        Buffer.BlockCopy(StaticObs, 0, Obs, 0, Obs.Length * sizeof(int));
        Colord = new bool[SizeX, SizeY];

        Queue<(int x, int y)> openList = new Queue<(int x, int y)>();


        for (int i = 0, k = dynamicMap.GetLength(0); i < k; i++)
            for (int j = 0; j < k; j++)
            {
                var x = i + offsetX - k / 2;
                var y = j + offsetY - k / 2;
                if (x < 0 || x >= SizeX)
                    continue;
                if (y < 0 || y >= SizeY)
                    continue;
                if (dynamicMap[i, j] != 0 && StaticMap[x, y] != 0)
                    continue;
                Map[x, y] = 0;
                Obs[x, y, 0] = x;
                Obs[x, y, 1] = y;
                Colord[x, y] = true;
                openList.Enqueue(new(x, y));
            }

        while (openList.Count > 0)
        {
            var c = openList.Dequeue();
            for (int i = -1; i <= 1; i++)
                for (int j = -1; j <= 1; j++)
                {
                    var ci = c.x + i;
                    var cj = c.y + j;
                    if (i == j)
                        continue;
                    if (ci < 0 || ci >= SizeX)
                        continue;
                    if (cj < 0 || cj >= SizeY)
                        continue;

                    // for calculate
                    var cObsx = Obs[ci, cj, 0];
                    var cObsy = Obs[ci, cj, 1];
                    if (cObsx != -1)
                    {
                        var m = (sbyte)(Math.Clamp(Math.Round(Math.Sqrt(Math.Pow(cObsx - c.x, 2) + Math.Pow(cObsy - c.y, 2))) * Resolution / MaxLength, 0, 1) * 100);
                        if (Map[c.x, c.y] > m)
                        {
                            Map[c.x, c.y] = m;
                            Obs[c.x, c.y, 0] = cObsx;
                            Obs[c.x, c.y, 1] = cObsy;
                        }

                    }
                    // for enqueue
                    if (Colord[ci, cj])
                        continue;
                    if (Obs[ci, cj, 0] == ci && Obs[ci, cj, 0] == cj)
                        continue;
                    Colord[ci, cj] = true;
                    openList.Enqueue((ci, cj));
                }
        }
    }
}