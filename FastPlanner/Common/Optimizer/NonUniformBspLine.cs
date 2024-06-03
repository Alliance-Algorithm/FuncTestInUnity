using MathNet.Numerics.LinearAlgebra.Single;
using Mono.Cecil.Cil;
using System;
using System.Collections.Generic;
using System.Numerics;
namespace BspLine
{
    public class NonUniformBspLine
    {
        private DenseMatrix controlPoints_;
        private int n_;
        private int m_;
        private int p_;
        private float interval_;
        private DenseVector u_;

        private const float limit_vel = 5 + 1e-4f;
        private const float limit_acc = 5 + 1e-4f;
        private const float limit_ratio = 50f + 1e-4f;
        public List<Vector2> Build(float TimeAll, List<Vector2> WayPoints, List<Vector2> StartEndDerivative, int Order = 3, float interval_ = 1)
        {
            ParametersToControlPoints(TimeAll, WayPoints, StartEndDerivative);
            SetUniformBSpline(Order, interval_);
            var k = 0;
            while (!ReallocateTime())
            {
                if (++k > 100)
                    break;
            }
            var ret = new List<Vector2>();
            for (float tu = u_[p_]; tu < u_[m_]; tu += 0.1f)
            {
                ret.Add(EvaluateTimePos(tu));
            }
            return ret;
        }

        private void ParametersToControlPoints(float TimeAll, List<Vector2> WayPoints, List<Vector2> StartEndDerivative)
        {
            var K = WayPoints.Count;

            var prow = new DenseVector(new float[] { 1, 4, 1 }) / 6;
            var vrow = new DenseVector(new float[] { -3, 0, 3 }) / 6;
            var arow = new DenseVector(new float[] { 3, -6, 3 }) / 6;

            var A = new DenseMatrix(K + 4, K + 2);

            for (int i = 0; i < K; i++)
            {
                A[i, i] = prow[0];
                A[i, i + 1] = prow[1];
                A[i, i + 2] = prow[2];
            }

            A[K, 0] = A[K + 1, K - 1] = 1 / TimeAll * vrow[0];
            A[K, 1] = A[K + 1, K] = 1 / TimeAll * vrow[1];
            A[K, 2] = A[K + 1, K + 1] = 1 / TimeAll * vrow[2];

            A[K + 2, 0] = A[K + 3, K - 1] = 2 / TimeAll / TimeAll * arow[0];
            A[K + 2, 1] = A[K + 3, K] = 2 / TimeAll / TimeAll * arow[1];
            A[K + 2, 2] = A[K + 3, K + 1] = 2 / TimeAll / TimeAll * arow[2];

            var b = new DenseMatrix(K + 4, 3);

            for (int i = 0; i < K; i++)
            {
                b[i, 0] = WayPoints[i].X;
                b[i, 1] = WayPoints[i].Y;
            }

            for (int i = 0; i < 4; i++)
            {
                b[K + i, 0] = StartEndDerivative[i].X;
                b[K + i, 1] = StartEndDerivative[i].Y;
            }

            controlPoints_ = (DenseMatrix)A.Solve(b);
        }
        private void SetUniformBSpline(int Order, float Interval)
        {
            p_ = Order;

            n_ = controlPoints_.RowCount - 1;
            interval_ = Interval;
            m_ = p_ + n_ + 1;

            u_ = new DenseVector(m_ + 1);

            for (int i = 0; i <= m_; i++)
                u_[i] = (i - p_) * interval_;
        }
        private Vector2 EvaluateTimePos(float u)
        {
            var ub = Math.Min(Math.Max(u_[p_], u), u_[m_ - p_]);

            int k = p_;
            while (u_[k + 1] < ub) k++;

            List<DenseVector> d = new List<DenseVector>();

            for (int i = 0; i <= p_; i++)
            {
                d.Add((DenseVector)controlPoints_.Row(k - p_ + i));
            }

            for (int r = 1; r <= p_; ++r)
            {
                for (int i = p_; i >= r; --i)
                {
                    var alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
                    d[i] = (1 - alpha) * d[i - 1] + (alpha * d[i]);
                }
            }
            return new(d[p_][0], d[p_][1]);
        }
        private bool ReallocateTime()
        {
            bool fea = true;

            DenseMatrix P = controlPoints_;
            int dimension = controlPoints_.ColumnCount;
            int rowCount = controlPoints_.RowCount;

            float max_acc = -1, max_vel = -1;

            for (int i = 0; i < rowCount - 1; i++)
            {
                var vel = p_ * (P.Row(i + 1) - P.Row(i)) / (u_[i + p_ + 1] - u_[i + 1]);

                if (Math.Abs(vel[0]) > limit_vel || Math.Abs(vel[1]) > limit_vel)
                {
                    fea = false;
                    max_vel = -1;
                    for (int j = 0; j < dimension; j++)
                        max_vel = Math.Max(max_vel, Math.Abs(vel[j]));

                    var ratio = max_vel / limit_vel;
                    if (ratio > limit_ratio) ratio = limit_ratio;

                    var time_ori = u_[i + p_ + 1] - u_[i + 1];
                    var time_new = ratio * time_ori;
                    var delta_t = time_new - time_ori;
                    var time_inc = delta_t / p_;

                    for (int j = i + 2; j <= i + p_ + 1; j++)
                        u_[j] += (j - i - 1) * time_inc;

                    for (int j = i + p_ + 2; j < u_.Count; j++)
                        u_[j] += delta_t;
                }
            }
            for (int i = 0; i < rowCount - 2; i++)
            {
                var acc = p_ * (p_ - 1) * ((P.Row(i + 2) - P.Row(i + 1)) / (u_[i + p_ + 2] - u_[i + 2]) - (P.Row(i + 1) - P.Row(i)) / (u_[i + p_ + 1] - u_[i + 1])) / (u_[i + p_ + 1] - u_[i + 2]);

                if (Math.Abs(acc[0]) > limit_acc || Math.Abs(acc[1]) > limit_acc)
                {
                    fea = false;
                    max_acc = -1;
                    for (int j = 0; j < dimension; j++)
                        max_acc = Math.Max(max_acc, Math.Abs(acc[j]));

                    var ratio = max_acc / limit_vel;
                    if (ratio > limit_ratio) ratio = limit_ratio;

                    var time_ori = u_[i + p_ + 1] - u_[i + 2];
                    var time_new = ratio * time_ori;
                    var delta_t = time_new - time_ori;
                    var time_inc = delta_t / (p_ - 1);

                    if (i == 1 || i == 2)
                    {
                        for (int j = 2; j <= 5; j++)
                            u_[j] = (j - 1) * time_inc;
                        for (int j = 6; j < u_.Count; j++)
                            u_[j] += 4 * time_inc;
                    }
                    else
                    {
                        for (int j = i + 3; j <= i + p_ + 1; j++)
                            u_[j] += (j - i - 2) * time_inc;
                        for (int j = i + p_ + 2; j < u_.Count; j++)
                            u_[j] += delta_t;
                    }
                }

            }
            return fea;
        }
    }
}