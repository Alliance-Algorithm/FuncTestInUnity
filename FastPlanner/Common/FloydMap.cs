
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;

[Serializable]
class FloydMap
{
    public Vector2[] Points;
    int[] PathsForSave;
    [NonSerialized] public float[,] Maps;
    [NonSerialized] int[,] Paths;
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