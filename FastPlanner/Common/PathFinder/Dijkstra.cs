
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;

[Serializable]
class Dijkstar
{
    public Vector2[] Points;
    bool[] AccessForSave;
    public float[] MapForSave;
    [NonSerialized] public int[,] Voronoi;
    [NonSerialized] public bool[,] Access;
    [NonSerialized] public int[] Paths;
    [NonSerialized] public float[,] Maps;


    public delegate (int x, int y) Vector22XY(Vector2 pos);


    public void VornoiBuild(ESDFMap map)
    {
        if (Access == null)
        {
            Access = new bool[Points.Length, Points.Length];
            for (int i = 0; i < Points.Length; i++)
                for (int j = 0; j < Points.Length; j++)
                    Access[i, j] = AccessForSave[i * Points.Length + j];
        }
        var Colord = new bool[map.SizeX, map.SizeY];
        Voronoi = new int[map.SizeX, map.SizeY];
        Queue<(int i, int j)> values = new Queue<(int i, int j)>();
        for (int i = 0, k = Points.Length; i < k; i++)
        {
            var j = map.Vector22XY(Points[i]);
            Voronoi[j.x, j.y] = i;
            values.Enqueue(j);
            Colord[j.x, j.y] = true;
        }

        while (values.Count > 0)
        {
            var q = values.Dequeue();
            for (int i = -1; i <= 1; i++)
                for (int j = -1; j <= 1; j++)
                {
                    var qi = q.i + i;
                    var qj = q.j + j;
                    if (qi < 0 || qi >= map.SizeX || qj < 0 || qj >= map.SizeY)
                        continue;
                    if (Colord[qi, qj])
                        continue;
                    Colord[qi, qj] = true;

                    if (map[qi, qj] == 0)
                        continue;

                    Voronoi[qi, qj] = Voronoi[q.i, q.j];
                    values.Enqueue((qi, qj));
                }
        }
    }
    public void Build()
    {
        MapForSave = new float[Maps.Length];
        AccessForSave = new bool[Points.Length * Points.Length];
        for (int i = 0; i < Points.Length; i++)
            for (int j = 0; j < Points.Length; j++)
            {
                AccessForSave[i * Points.Length + j] = Access[i, j];
                Maps[i, j] = Access[i, j] ? Maps[i, j] : float.MaxValue;
            }
        Buffer.BlockCopy(Maps, 0, MapForSave, 0, Maps.Length * sizeof(float));
    }
    public void GetPath(Vector2 Begin, Vector2 End, out List<Vector2> Path, Vector22XY vector22XY)
    {
        Path = new() { };
        int k = Points.Length;
        var Colord = new bool[k];
        Paths = new int[k];
        if (Maps == null)
            Maps = new float[k, k];
        Buffer.BlockCopy(MapForSave, 0, Maps, 0, MapForSave.Length * sizeof(float));
        var SMap = new float[k];
        var a = vector22XY(Begin);
        var b = vector22XY(End);
        int From = Voronoi[a.x, a.y];
        int To = Voronoi[b.x, b.y];
        Colord[From] = true;
        for (int i = 0; i < k; i++)
        {
            Maps[From, i] = Maps[i, From] = i != From ? (Begin - Points[i]).Length() : 0;
            Maps[To, i] = Maps[i, To] = i != To ? (End - Points[i]).Length() : 0;
            if (Access[From, i])
            {
                Paths[i] = From;
                SMap[i] = Maps[From, i];
            }
            else SMap[i] = float.MaxValue;
        }
        while (!Colord[To])
        {
            int t = -1;
            float min = float.MaxValue;
            for (int j = 0; j < k; j++)
            {
                if (Colord[j])
                    continue;
                if (min > SMap[j])
                {
                    min = SMap[j];
                    t = j;
                }
            }
            if (t == -1)
            {
                return;
            }
            Colord[t] = true;
            for (int j = 0; j < k; j++)
            {
                if (Colord[j])
                    continue;
                if (!Access[t, j])
                    continue;
                if (SMap[j] > min + Maps[t, j])
                {
                    SMap[j] = min + Maps[t, j];
                    Paths[j] = t;
                }
            }
        }
        var tmp = Paths[To];
        Path.Add(End);
        while (tmp != From)
        {
            Path.Add(Points[tmp]);
            tmp = Paths[tmp];
        }
        Path.Add(Begin);
        Path.Reverse();
        return;
    }
}