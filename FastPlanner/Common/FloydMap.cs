
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;

[Serializable]
class FloydMap
{
    public Vector2[] Points;
    int[] PathsForSave;
    float[] MapsForSave;

    [NonSerialized] public float[,] Maps;
    [NonSerialized] public int[,] Voronoi;
    [NonSerialized] public bool[,] Access;
    [NonSerialized] int[,] Paths;
    [NonSerialized] bool[,] Colord;

    [NonSerialized] int SizeX;
    [NonSerialized] int SizeY;

    public delegate (int x, int y) Vector22XY(Vector2 pos);

    [NonSerialized] public Vector22XY vector22XY;

    void VornoiBuild()
    {
        Colord = new bool[SizeX, SizeY];
        Voronoi = new int[SizeX, SizeY];
        Queue<(int i, int j)> values = new Queue<(int i, int j)>();
        for (int i = 0, k = Points.Length; i < k; i++)
        {
            var j = vector22XY(Points[i]);
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
                    var qj = q.j + i;
                    if (qi < 0 || qi >= SizeX || qj < 0 || qj >= SizeY)
                        continue;
                    if (Colord[qi, qj])
                    {
                        Access[Voronoi[qi, qj], Voronoi[q.i, q.j]] = Access[Voronoi[q.i, q.j], Voronoi[qi, qj]] = true;
                        continue;
                    }

                    Colord[qi, qj] = true;
                    Voronoi[qi, qj] = Voronoi[q.i, q.j];
                    values.Enqueue((qi, qj));
                }
        }
    }
    public void Build()
    {
        Paths = new int[Maps.GetLength(0), Maps.GetLength(0)];
        for (int i = 0; i < Paths.GetLength(0); i++)
            for (int j = 0; j < Paths.GetLength(0); j++)
                Paths[i, j] = i;

        for (int k = 0; k < Paths.GetLength(0); k++)        //     Gizmos.DrawLine(transform.position, i.transform.position);

        {
            for (int i = 0; i < Paths.GetLength(0); i++)

                for (int j = 0; j < Paths.GetLength(0); j++)
                {
                    if (j == k)
                        continue;
                    if (Maps[i, j] > Maps[i, k] + Maps[k, j])
                    {
                        Maps[i, j] = Maps[i, k] + Maps[k, j];
                        int tk = k;
                        int ttk = Paths[i, k];
                        while (Paths[i, ttk] != ttk)
                        {
                            tk = ttk;
                            ttk = Paths[i, ttk];
                        }
                        Paths[i, j] = tk;
                    }
                }
        }
        PathsForSave = new int[Points.Length * Points.Length];
        for (int i = 0; i < Points.Length; i++)
            for (int j = 0; j < Points.Length; j++)
                PathsForSave[i * Points.Length + j] = Paths[i, j];


    }

    public int this[int a, int b]
    {
        get => Paths[a, b];
        set => Paths[a, b] = value;
    }

    public void GetPath(out List<Vector2> Path, int Form, int To)
    {
        if (Paths == null)
        {
            Paths = new int[Points.Length, Points.Length];
            for (int i = 0; i < Points.Length; i++)
                for (int j = 0; j < Points.Length; j++)
                    Paths[i, j] = PathsForSave[i * Points.Length + j];
        }
        Path = new() { };
        int t = Form;
        int t_pre = -1;
        while (t != t_pre)
        {
            t_pre = t;
            Path.Add(Points[t]);
            t = Paths[t, To];
        }
        Path.Add(Points[To]);
        return;
    }
}