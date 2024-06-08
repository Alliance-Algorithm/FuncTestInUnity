using System;

internal class Octree<T> where T : class, IThreeDimensional, IEquatable<T>, ICopyAble<T>, new()
{
    internal const float Resolution = 0.1f;
    internal const int D = 8;
    internal const float Bound = 28;

    internal Octree<T>[] subTrees = new Octree<T>[8];

    internal bool minimal = true;
    internal int d = 0;

    internal T data;
    internal T center;

    public Octree()
    {
        center = new T() { X = 0, Y = 0, Z = 0 };
        data = new T() { X = 0, Y = 0, Z = 0 };
    }

    public Octree(T Center, T Data, int D, int Index)
    {
        var x = Index & 1;
        var y = (Index >> 1) & 1;
        var z = (Index >> 2) & 1;
        float offset = (float)(Bound / Math.Pow(2, D));
        Center.X += offset * x;
        Center.Y += offset * y;
        Center.Z += 10 * offset * z;
        minimal = true;
        center = Center;
        data = Data;
        d = D;
    }

    public bool Find(T Obj, out T Current)
    {
        Current = null;
        if (!minimal)
            return subTrees[center.Index(Obj)] != null && subTrees[center.Index(Obj)].Find(Obj, out Current);
        var b = d == D || Obj == data;
        if (b)
            Current = data;
        return b;
    }

    public bool Insert(T obj)
    {
        if (!minimal && subTrees[data.Index(obj)] != null)
            return subTrees[data.Index(obj)].Insert(obj);
        minimal = false;
        if (d == D || data.Equals(obj))
        {
            data = obj;
            return false;
        }
        var k1 = center.Index(data);
        var k2 = center.Index(obj);
        if (k1 == k2)
        {
            subTrees[k1] = new Octree<T>(center.Copy(), data, d + 1, k1);
            return subTrees[k1].Insert(obj);
        }
        subTrees[k1] = new Octree<T>(center.Copy(), data, d + 1, k1);
        subTrees[k2] = new Octree<T>(center.Copy(), obj, d + 1, k2);
        return true;
    }

}

public interface ICopyAble<T> where T : class
{
    public T Copy();
}