using MathNet.Numerics.LinearAlgebra.Single;
using Mono.Cecil.Cil;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Numerics;
namespace Optimizer
{
    public class NonUniformBspline
    {
        public DenseMatrix controlPoints_;
        public int n_;
        private int m_;
        private int p_;
        private float interval_;
        private DenseVector t_;

        private DenseMatrix m_k;

        float t = 0;
        private Vector2 beginPoint_;

        public const float limit_vel = BsplineOptimizer.limit_vel;
        public const float limit_acc = BsplineOptimizer.limit_acc;
        public const float limit_ratio = BsplineOptimizer.limit_ratio;

        public List<Vector2> Build(float TimeAll, List<Vector2> WayPoints, List<Vector2> StartEndDerivative, int Order = 3, float interval_ = 1)
        {
            beginPoint_ = WayPoints[0];
            ParametersToControlPoints(TimeAll, WayPoints, StartEndDerivative);

            SetUniformBSpline(Order, interval_);
            var k = 0;
            while (!ReallocateTime())
            {
                if (++k > 100)
                    break;
            }
            var ret = new List<Vector2>();
            for (float tu = t_[p_]; tu < t_[m_]; tu += 0.1f)
            {
                ret.Add(EvaluateTimePos(tu));
            }
            return ret;
        }

        internal void ParametersToControlPoints(float TimeAll, List<Vector2> WayPoints, List<Vector2> StartEndDerivative, int K = 4, float TimeInterval = 2)
        {
            beginPoint_ = WayPoints[0];

            n_ = WayPoints.Count;
            p_ = K - 1;

            m_ = p_ + n_;

            t_ = new DenseVector(m_);

            for (int i = 0; i < t_.Count; i++)
                t_[i] = i * TimeInterval;
            DenseVector prow = new(K);
            DenseVector vrow = new(K);
            DenseVector arow = new(K);
            DenseVector jrow = new(K);
            DenseVector srow = new(K);
            if (K == 4)
            {
                prow = new DenseVector(new float[] { 1, 4, 1, 0 }) / 6;
                vrow = new DenseVector(new float[] { -3, 0, 3, 0 }) / 6;
                arow = new DenseVector(new float[] { 3, -6, 3, 0 }) / 6;
                jrow = new DenseVector(new float[] { -1, 3, -3, 1 }) / 6;
                m_k = new DenseMatrix(K, K);
                m_k.SetRow(0, prow);
                m_k.SetRow(1, vrow);
                m_k.SetRow(2, arow);
                m_k.SetRow(3, jrow);

            }
            else if (K == 5)
            {
                prow = new DenseVector(new float[] { 1, 11, 11, 1, 0 }) / 24;
                vrow = new DenseVector(new float[] { -4, -12, 12, 4, 0 }) / 24;
                arow = new DenseVector(new float[] { 6, -6, -6, 6, 0 }) / 24;
                jrow = new DenseVector(new float[] { -4, 12, -12, 4, 0 }) / 24;
                srow = new DenseVector(new float[] { 1, -4, 6, -4, 1 }) / 24;
                m_k = new DenseMatrix(K, K);
                m_k.SetRow(0, prow);
                m_k.SetRow(1, vrow);
                m_k.SetRow(2, arow);
                m_k.SetRow(3, jrow);
                m_k.SetRow(4, srow);
            }
            var A = new DenseMatrix(m_, m_);

            var t_inv1 = 1;
            var t_inv2 = 2;
            var t_inv3 = 6;
            // constant
            if (K == 4)
            {
                for (int i = 0; i < n_; i++)
                {
                    // position
                    A[i, i + 0] = prow[0];
                    A[i, i + 1] = prow[1];
                    A[i, i + 2] = prow[2];
                }

                A[n_, 0] = vrow[0] * t_inv1;
                A[n_, 1] = vrow[1] * t_inv1;
                A[n_, 2] = vrow[2] * t_inv1;
                A[n_ + 1, n_ - 1] = vrow[0] * t_inv1;
                A[n_ + 1, n_] = vrow[1] * t_inv1;
                A[n_ + 1, n_ + 1] = vrow[2] * t_inv1;

                // A[n_ + 2, 0] = arow[0] * t_inv2;
                // A[n_ + 2, 1] = arow[1] * t_inv2;
                // A[n_ + 2, 2] = arow[2] * t_inv2;

                A[n_ + 2, n_ - 1] = jrow[0];
                A[n_ + 2, n_] = jrow[1];
                A[n_ + 2, n_ + 1] = jrow[2];
                A[n_ + 2, n_ + 2] = jrow[3];
            }
            else if (K == 5)
            {
                for (int i = 0; i < n_; i++)
                {
                    // position
                    A[i, i + 0] = prow[0];
                    A[i, i + 1] = prow[1];
                    A[i, i + 2] = prow[2];
                    A[i, i + 3] = prow[3];
                }

                A[n_, 0] = vrow[0];
                A[n_, 1] = vrow[1];
                A[n_, 2] = vrow[2];
                A[n_, 3] = vrow[3];
                A[n_ + 1, n_ - 1] = vrow[0];
                A[n_ + 1, n_] = vrow[1];
                A[n_ + 1, n_ + 1] = vrow[2];
                A[n_ + 1, n_ + 2] = vrow[3];

                A[n_ + 2, n_ - 1] = arow[0];
                A[n_ + 2, n_] = arow[1];
                A[n_ + 2, n_ + 1] = arow[2];
                A[n_ + 2, n_ + 2] = arow[3];

                A[n_ + 3, n_ - 1] = srow[0];
                A[n_ + 3, n_] = srow[1];
                A[n_ + 3, n_ + 1] = srow[2];
                A[n_ + 3, n_ + 2] = srow[3];
                A[n_ + 3, n_ + 3] = srow[4];
            }

            var b = new DenseMatrix(m_, 3);

            for (int i = 0; i < n_; i++)
            {
                // position
                b[i, 0] = WayPoints[i].X;
                b[i, 1] = WayPoints[i].Y;
            }
            var k1 = StartEndDerivative[0].Length();
            var k2 = StartEndDerivative[1].Length();
            var k3 = StartEndDerivative[2].Length();
            if (k1 != 0)
                StartEndDerivative[0] = StartEndDerivative[0] / k1 * (k1 < limit_vel ? k1 : limit_vel);
            if (k2 != 0)
                StartEndDerivative[1] = StartEndDerivative[1] / k2 * (k2 < limit_vel ? k2 : limit_vel);
            if (k3 != 0)
                StartEndDerivative[2] = StartEndDerivative[2] / k3 * (k3 > limit_acc ? k3 : limit_acc);
            b[n_, 0] = StartEndDerivative[0].X;
            b[n_, 1] = StartEndDerivative[0].Y;
            b[n_ + 1, 0] = StartEndDerivative[1].X;
            b[n_ + 1, 1] = StartEndDerivative[1].Y;
            // b[n_ + 2, 0] = StartEndDerivative[2].X;
            // b[n_ + 2, 1] = StartEndDerivative[2].Y;
            // position
            // var t = new DenseMatrix(m_ + 1, m_ + 3);
            // for (int i = 0; i < m_ + 1; i++)
            //     t.SetColumn(i, A.Column(i));
            // for (int i = 0; i < 3; i++)
            //     t.SetColumn(i + m_, b.Column(i));
            controlPoints_ = (DenseMatrix)A.Solve(b);
        }
        internal void SetUniformBSpline(int Order, float Interval)
        {
            p_ = Order - 1;

            n_ = controlPoints_.RowCount - 1;
            interval_ = Interval;
            m_ = p_ + n_ + 1;

            t_ = new DenseVector(m_ + p_);

            for (int i = 0; i <= t_.Count; i++)
                t_[i] = (i - p_) * interval_;
        }

        public void Relocate(ESDFMap costMap)
        {
            var k = controlPoints_.RowCount;

            var p2 = new Vector2(controlPoints_[1, 0], controlPoints_[1, 1]);
            costMap.RelocatedToNoneCollision(beginPoint_, ref p2);
            controlPoints_[1, 0] = p2.X;
            controlPoints_[1, 1] = p2.Y;
            for (int i = 2; i < n_; i++)
            {
                p2 = new Vector2(controlPoints_[i, 0], controlPoints_[i, 1]);
                costMap.RelocatedToNoneCollision(new Vector2(controlPoints_[i - 1, 0], controlPoints_[i - 1, 1]), ref p2);
                controlPoints_[i, 0] = p2.X;
                controlPoints_[i, 1] = p2.Y;
            }
        }

        public List<Vector2> CalculatePos()
        {
            var ret = new List<Vector2>();
            for (float tu = t_[p_]; tu <= t_[m_ - 1]; tu += 0.1f)
            {
                // if (t_[m_ - 1] > 100)
                //     break;
                ret.Add(EvaluateTimePos(tu));
            }

            // for (int tu = p_; tu < m_ - 1; tu += 1)
            // {
            //     ret.Add(EvaluateTimePos(t_[tu]));
            // }
            return ret;

        }
        public (int Id, bool Safe) Check(float t, ESDFMap CostMap)
        {
            var tb = Math.Min(Math.Max(t_[p_], t), t_[m_ - 1]);
            int k1 = p_;
            while (t_[k1 + 1] < tb) k1++;
            for (float tu = t_[k1]; tu <= t_[k1 + 1]; tu += 0.1f)
            {
                var p = CostMap.Vector22XY(EvaluateTimePos(tu));
                if (CostMap[p.x, p.y] != 0)
                    continue;
                else
                {
                    return (k1, false);
                }
            }
            return (k1, true);
        }
        private Vector2 EvaluateTimePos(float t)
        {
            var tb = Math.Min(Math.Max(t_[p_], t), t_[m_ - 1]);

            int k = p_;
            while (t_[k + 1] < tb) k++;
            var u_ = (tb - t_[k]) / (t_[k + 1] - t_[k]);
            DenseMatrix d = new DenseMatrix(p_ + 1, 3);

            for (int i = 0; i <= p_; i++)
            {
                d.SetRow(i, (DenseVector)controlPoints_.Row(k - p_ + i));
            }
            DenseMatrix u = new DenseMatrix(1, p_ + 1);
            u[0, 0] = 1;
            u[0, 1] = u_;
            u[0, 2] = u[0, 1] * u_;
            u[0, 3] = u[0, 2] * u_;
            if (p_ == 4)
                u[0, 4] = u[0, 3] * u_;

            var rst = u * m_k * d;

            return new(rst[0, 0], rst[0, 1]);
        }
        internal bool ReallocateTime()
        {
            bool fea = true;
            for (int i = 0; i < n_ - 1; i++)
            {
                var t1 = new Vector2(controlPoints_[i, 0], controlPoints_[i, 1]);
                var t2 = new Vector2(controlPoints_[i + 1, 0], controlPoints_[i + 1, 1]);
                var l = p_ * (t2 - t1).Length();
                var t_origin = t_[i + p_ + 1] - t_[i + 1];
                l /= t_origin;
                if (l > limit_vel)
                {
                    var ratio = l / limit_vel;
                    if (ratio > limit_ratio)
                        ratio = limit_ratio;
                    fea = false;
                    var t_new = t_origin * ratio;
                    var delta = t_new - t_origin;
                    var t_inc = delta / p_;
                    for (int j = i + 2; j <= i + p_ + 1; j++)
                        t_[j] += (j - i - 1) * t_inc;
                    for (int j = i + p_ + 2; j < m_; j++)
                        t_[j] += delta;
                }
            }
            for (int i = 0; i < n_ - 2; i++)
            {
                var t11 = new Vector2(controlPoints_[i, 0], controlPoints_[i, 1]);
                var t21 = new Vector2(controlPoints_[i + 1, 0], controlPoints_[i + 1, 1]);
                var t12 = new Vector2(controlPoints_[i + 1, 0], controlPoints_[i + 1, 1]);
                var t22 = new Vector2(controlPoints_[i + 2, 0], controlPoints_[i + 2, 1]);
                var l = p_ * (p_ + 1) * ((t22 - t12) / (t_[i + p_ + 2] - t_[i + 2]) - (t21 - t11) / (t_[i + p_ + 1] - t_[i + 1])).Length() / (t_[i + p_ + 2] - t_[i + 1]);
                var t_origin = t_[i + p_ + 2] - t_[i + 1];
                if (l > limit_acc)
                {
                    var ratio = l / limit_acc;
                    if (ratio > limit_ratio)
                        ratio = limit_ratio;
                    fea = false;
                    var t_new = t_origin * ratio;
                    var delta = t_new - t_origin;
                    var t_inc = delta / (p_ + 1);
                    if (i == 1 || i == 2)
                    {
                        for (int j = 2; j <= 5; j++)
                            t_[j] += (j - 1) * t_inc;
                        for (int j = 6; j < m_; j++)
                            t_[j] += 4 * t_inc;
                    }
                    else
                    {
                        for (int j = i + 1; j <= i + p_ + 2; j++)
                            t_[j] += (j - i - 1) * t_inc;
                        for (int j = i + p_ + 3; j < m_; j++)
                            t_[j] += delta;
                    }
                }
            }
            for (int i = 0; i < m_; i++)
                t_[i] -= t_[p_];
            return fea;
        }

        internal Vector3 TargetVerlocity(float t)
        {

            var tb = Math.Min(Math.Max(t_[p_], t), t_[m_ - 1]);

            int k = p_;
            while (t_[k + 1] < tb) k++;
            var u_ = (tb - t_[k]) / (t_[k + 1] - t_[k]);
            DenseMatrix d = new DenseMatrix(p_ + 1, 3);

            for (int i = 0; i <= p_; i++)
            {
                d.SetRow(i, (DenseVector)controlPoints_.Row(k - p_ + i));
            }
            DenseMatrix u = new DenseMatrix(1, p_ + 1);
            u[0, 0] = 0;
            u[0, 1] = 1;
            u[0, 2] = 2 * u_;
            u[0, 3] = 3 * u_ * u_;
            if (p_ == 4)
                u[0, 4] = 4 * u_ * u_ * u_;

            var rst = u * m_k * d / (t_[k + 1] - t_[k]);
            var temp = EvaluateTimePos(t);
            UnityEngine.Debug.DrawLine(new(-temp.Y, 7, temp.X), new(-temp.Y - rst[0, 1], 7, temp.X + rst[0, 0]), UnityEngine.Color.blue);
            // TargetAcc(t);
            return new(rst[0, 0], rst[0, 1], 0);
        }

        internal Vector3 TargetAcc(float t)
        {
            var tb = Math.Min(Math.Max(t_[p_], t), t_[m_ - 1]);

            int k = p_;
            while (t_[k + 1] < tb) k++;
            var u_ = (tb - t_[k]) / (t_[k + 1] - t_[k]);
            DenseMatrix d = new DenseMatrix(p_ + 1, 3);

            for (int i = 0; i <= p_; i++)
            {
                d.SetRow(i, (DenseVector)controlPoints_.Row(k - p_ + i));
            }
            DenseMatrix u = new DenseMatrix(1, p_ + 1);
            u[0, 0] = 0;
            u[0, 1] = 0;
            u[0, 2] = 2;
            u[0, 3] = 6 * u_;
            if (p_ == 4)
                u[0, 4] = 12 * u_ * u_;

            var rst = u * m_k * d / (t_[k + 1] - t_[k]) / (t_[k + 1] - t_[k]) * 2;
            var temp = EvaluateTimePos(t);
            UnityEngine.Debug.DrawLine(new(-temp.Y, 7, temp.X), new(-temp.Y - rst[0, 1], 7, temp.X + rst[0, 0]), UnityEngine.Color.red);
            return new(rst[0, 0], rst[0, 1], 0);
        }
    }
}