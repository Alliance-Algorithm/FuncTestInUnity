using System.Drawing;
using System.IO;
using System.Runtime.Serialization.Json;
using UnityEngine;
using UnityEngine.AI;

[RequireComponent(typeof(LocalGridMapBuilder))]
class ESDFBuilder : MonoBehaviour
{
    int SizeX = 15;
    int SizeY = 28;
    public string FilePath;
    public float Resolution = 0.05f;
    public Transform Sentry;
    ESDFMap map_;
    public void Bake()
    {
        SizeX = Mathf.RoundToInt(SizeX / Resolution);
        SizeY = Mathf.RoundToInt(SizeY / Resolution);
        var halfBoxsizeX = SizeX / 2.0f;
        var halfBoxsizeY = SizeY / 2.0f;
        ESDFMap map = new ESDFMap
        {
            SizeX = SizeX + 1,
            SizeY = SizeY + 1,
            Resolution = Resolution
        };
        for (int x = 0, k = SizeX + 1; x < k; x += 1)
        {
            for (int y = 0, l = SizeY + 1; y < l; y += 1)
            {
                Vector3 p1 = new Vector3(-x * Resolution + 7.5f, 2.5f, y * Resolution - 14f);
                // Vector3 p1 = transform.position + transform.right * (x * Resolution - halfBoxsizeX) + Vector3.up * 2 + transform.forward * (-y * Resolution + halfBoxsizeY);
                // Vector3 p1 = transform.position + new Vector3(x * Resolution - halfBoxsize, 2, y * Resolution - halfBoxsize);

                if (!Physics.Raycast(p1, Vector3.down, out var hitInfo, 5))
                    map[x, y] = 0;
                else if (!NavMesh.SamplePosition(hitInfo.point, out var hit, 0.1f, NavMesh.AllAreas))
                    map[x, y] = 0;
                else
                    map[x, y] = -1;
            }
        }
        map.StaticMapForSave = new sbyte[(SizeX + 1) * (SizeY + 1)];
        for (int i = 0; i < SizeX + 1; i++)
            for (int j = 0; j < SizeY + 1; j++)
            {
                map.StaticMapForSave[i * SizeY + i + j] = map[i, j];
            }
        try
        {
            File.Delete(FilePath);
        }
        catch { }
        DataContractJsonSerializer serializer = new DataContractJsonSerializer(typeof(ESDFMap));
        var f = File.OpenWrite(FilePath);
        serializer.WriteObject(f, map);
        f.Close();

        SizeX = 15;
        SizeY = 28;
    }

    public void Test()
    {
        DataContractJsonSerializer serializer = new DataContractJsonSerializer(typeof(ESDFMap));
        var f = File.OpenRead(FilePath);
        map_ = (ESDFMap)serializer.ReadObject(f);
        f.Close();
        map_.Build();
        var a = Sentry.GetComponent<LocalGridMapBuilder>();
        a.Resolution = Resolution;
        a.Start();
        a.Build(out var m);
        var b = map_.Vector22XY(new System.Numerics.Vector2(Sentry.position.z, -Sentry.position.x));
        map_.Update(b.x, b.y, m);
    }

    void OnDrawGizmosSelected()
    {
        if (map_ == null)
            return;
        for (int i = 0; i < map_.SizeX; i++)
        {
            for (int j = 0; j < map_.SizeY; j++)
            {
                var p = map_.XY2Vector2(i, j);
                var p1 = new Vector3(-p.Y, 5, p.X);
                Gizmos.color = UnityEngine.Color.red + new UnityEngine.Color(-1, 1, 0, 0) * map_[i, j] / 100.0f;
                Gizmos.DrawCube(p1, map_.Resolution * Vector3.one);
            }
        }
    }
}