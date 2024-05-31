using System;
using System.Collections.Generic;
using System.Numerics;

class WayPointUpdater
{
    public ESDFMap ESDFMap;
    public FloydMap FloydMap;

    public void PointFinder(Vector2 From, Vector2 To, List<Vector2> WayPoints)
    {
        PriorityQueue<Vector2, float> priorityQueue1 = new PriorityQueue<Vector2, float>();
        PriorityQueue<Vector2, float> priorityQueue2 = new PriorityQueue<Vector2, float>();

        foreach (var i in FloydMap.Points)
        {
            priorityQueue1.Enqueue(i, (From - i).Length());
            priorityQueue2.Enqueue(i, (To - i).Length());
        }
        for (int i = 0; i < 3; i++)
        {

            var v1 = priorityQueue1.Dequeue();
            for (int j = 0; j < 3; j++)
            {
                var v2 = priorityQueue1.Dequeue();
            }
        }
    }
}