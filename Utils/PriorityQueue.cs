using System;
using System.Collections.Generic;
using System.Numerics;

public class PriorityQueue<T, TPriority> where TPriority : IComparable<TPriority>
{
    public int Count => priorityQueue.Count;
    private List<(T item, TPriority priority)> priorityQueue = new List<(T, TPriority)>();
    private IComparer<TPriority> comparer;

    public PriorityQueue(IComparer<TPriority> comparer = null)
    {
        this.comparer = comparer ?? Comparer<TPriority>.Default;
    }

    public void Enqueue(T item, TPriority priority)
    {
        priorityQueue.Add((item, priority));
        UpAdjust(priorityQueue.Count - 1);
    }

    public (T item, TPriority priority) Dequeue()
    {
        if (priorityQueue == null || priorityQueue.Count == 0)
        {
            Console.WriteLine("The queue is empty!");
            return (default(T), default(TPriority));
        }

        var head = priorityQueue[0];
        Remove(head);
        return head;
    }

    public void Remove((T item, TPriority priority) key)
    {
        if (priorityQueue == null)
        {
            Console.WriteLine("The queue is empty!");
            return;
        }

        if (priorityQueue.Contains(key))
        {
            int removeIndex = priorityQueue.FindIndex(item => item.Equals(key));
            int endIndex = priorityQueue.Count - 1;

            priorityQueue[removeIndex] = priorityQueue[endIndex];
            priorityQueue.RemoveAt(endIndex);

            if (priorityQueue.Count > 1)
                DownAdjust(removeIndex, priorityQueue.Count - 1);
        }
    }

    private void UpAdjust(int childIndex)
    {
        int parentIndex = (childIndex - 1) / 2;
        var temp = priorityQueue[childIndex];

        while (childIndex > 0)
        {
            int cmp = comparer.Compare(temp.priority, priorityQueue[parentIndex].priority);
            if (cmp > 0)
            {
                priorityQueue[childIndex] = priorityQueue[parentIndex];
                childIndex = parentIndex;
                parentIndex = (parentIndex - 1) / 2;
            }
            else
                break;
        }

        priorityQueue[childIndex] = temp;
    }

    private void DownAdjust(int parentIndex, int length)
    {
        int leftChildPos = parentIndex * 2 + 1;
        int rightChildPos = parentIndex * 2 + 2;
        var temp = priorityQueue[parentIndex];

        while (leftChildPos <= length)
        {
            int pos = comparer.Compare(priorityQueue[leftChildPos].priority, priorityQueue[rightChildPos].priority) > 0 ? leftChildPos : rightChildPos;
            if (pos <= length)
            {
                int cmp = comparer.Compare(priorityQueue[pos].priority, temp.priority);
                if (cmp > 0)
                {
                    priorityQueue[parentIndex] = priorityQueue[pos];
                    parentIndex = pos;
                    leftChildPos = leftChildPos * 2 + 1;
                    rightChildPos = leftChildPos + 1;
                }
                else
                    break;
            }
            else
                break;
        }

        priorityQueue[parentIndex] = temp;
    }

    internal void Enqueue((int _x, int _y, double _g) value, double v)
    {
        throw new NotImplementedException();
    }

    internal void Enqueue((int _x, int _y, double _g, Vector2) value, double f)
    {
        throw new NotImplementedException();
    }
}

class Program
{
    static void Main(string[] args)
    {
        // Example usage
        var priorityQueue = new PriorityQueue<int, double>();
        priorityQueue.Enqueue(3, 0.5);
        priorityQueue.Enqueue(5, 0.8);
        priorityQueue.Enqueue(9, 0.3);
        priorityQueue.Enqueue(7, 0.6);
        priorityQueue.Enqueue(1, 0.9);

        var (item, priority) = priorityQueue.Dequeue();
        Console.WriteLine($"Dequeue: Item = {item}, Priority = {priority}");
    }
}
