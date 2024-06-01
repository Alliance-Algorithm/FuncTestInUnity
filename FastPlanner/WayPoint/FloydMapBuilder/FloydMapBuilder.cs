
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Json;
using UnityEngine;

class FloydMapBuilder : MonoBehaviour
{
    public Vector2Int FormTo;
    public string FilePath;

    protected FloydMap map_;
    List<System.Numerics.Vector2> path_;

    public bool DebugShow = true;
    public void Build()
    {
        FloydMap map = new();
        var wayPoints = GetComponentsInChildren<WayPoints>();
        List<System.Numerics.Vector2> temp = new();
        foreach (var i in wayPoints)
            temp.Add(new System.Numerics.Vector2(i.transform.position.z, -i.transform.position.x));
        map.Points = temp.ToArray();
        map.Maps = new float[wayPoints.Length, wayPoints.Length];
        map.Access = new bool[wayPoints.Length, wayPoints.Length];
        for (int i = 0; i < wayPoints.Length; i++)
            for (int j = 0; j < wayPoints.Length; j++)
                map.Maps[i, j] = float.MaxValue;
        foreach (var i in wayPoints)
        {
            map.Maps[int.Parse(i.name), int.Parse(i.name)] = 0;
            map.Access[int.Parse(i.name), int.Parse(i.name)] = true;
            foreach (var k in i.ToPoints)
            {
                map.Maps[int.Parse(i.name), int.Parse(k.name)] = (i.transform.position - k.transform.position).magnitude;
                map.Access[int.Parse(i.name), int.Parse(k.name)] = true;
            }
        }

        map.Build();

        DataContractJsonSerializer serializer = new DataContractJsonSerializer(typeof(FloydMap));
        File.Delete(FilePath);
        var f = File.Open(FilePath, FileMode.OpenOrCreate);

        serializer.WriteObject(f, map);
        f.Close();
    }

    public void Test()
    {
        DataContractJsonSerializer serializer = new DataContractJsonSerializer(typeof(FloydMap));
        var f = File.OpenRead(FilePath);
        map_ = (FloydMap)serializer.ReadObject(f);
        f.Close();
    }
    void OnDrawGizmosSelected()
    {
        int k = 0;
        foreach (var i in GetComponentsInChildren<WayPoints>())
        {
            i.name = k++.ToString();
            i.DebugShow = DebugShow;
        }

    }
    void OnDrawGizmos()
    {
        if (!DebugShow)
            return;
        Gizmos.color = Color.yellow;
        if (path_ == null)
            return;
        for (int i = 0; i < path_.Count; i++)
        {
            Gizmos.DrawSphere(new Vector3(-path_[i].Y, 5, path_[i].X), 0.5f);
            if (i != 0)
            {
                Gizmos.DrawLine(new Vector3(-path_[i].Y, 5, path_[i].X), new Vector3(-path_[i - 1].Y, 5, path_[i - 1].X));
            }
        }
    }
}