using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Numerics;
using System.Runtime.Serialization.Json;
using MathNet.Numerics.LinearAlgebra.Complex;
using MathNet.Numerics.Optimization;

namespace Optimizer
{
    class BsplineOptimizer
    {
        public const float limit_vel = 5 + 1e-4f;
        public const float limit_acc = 5 + 1e-4f;
        public const float limit_ratio = 4f + 1e-4f;
        public const float lambda1 = 0f;
        public const float lambda2 = 1f;
        public const float lambda3 = 0;
        public int order;
        public NonUniformBspline nonUniformBspline = new();
        public ESDFMap CostMap;
        public BsplineOptimizer(string ESDF_Path)
        {
            DataContractJsonSerializer serializer = new DataContractJsonSerializer(typeof(ESDFMap));
            var f = File.OpenRead(ESDF_Path);
            CostMap = (ESDFMap)serializer.ReadObject(f);
            f.Close();
            CostMap.Build();
        }
        public BsplineOptimizer() { }

        public List<Vector2> Build(float TimeAll, List<Vector2> WayPoints, List<Vector2> StartEndDerivative, int Order = 4, float interval_ = 0.5f)
        {
            this.order = Order;
            nonUniformBspline.ParametersToControlPoints(TimeAll, WayPoints, StartEndDerivative, Order, interval_);
            // nonUniformBspline.SetUniformBSpline(Order, interval_);
            // nonUniformBspline.Relocate(CostMap);
            // var o = Optimizer();

            // var l = nonUniformBspline.n_ - order - order;
            // for (int i = 0; i < l; i += 2)
            // {
            //     nonUniformBspline.controlPoints_[i + order, 0] = (float)o[2 * i];
            //     nonUniformBspline.controlPoints_[i + order, 1] = (float)o[2 * i + 1];
            // }

            var k = 0;
            while (!nonUniformBspline.ReallocateTime())
            {
                if (++k > 100)
                    break;
            }
            return nonUniformBspline.CalculatePos();
        }

        public (int Id, bool Safe) Update(float t)
        {
            return nonUniformBspline.Check(t, CostMap);
        }

        public Vector3 TargetVeclocity(float t) => nonUniformBspline.TargetVerlocity(t);

        MathNet.Numerics.LinearAlgebra.Vector<Double> Optimizer()
        {
            var l = nonUniformBspline.n_ - order - order;
            var obj = ObjectiveFunction.Gradient(CalCost);
            var solve = new BfgsMinimizer(1e-4, 1, 4);
            var init = new MathNet.Numerics.LinearAlgebra.Double.DenseVector(2 * l);
            for (int i = 0; i < l; i += 2)
            {
                init[2 * i] = nonUniformBspline.controlPoints_[i + order, 0];
                init[2 * i + 1] = nonUniformBspline.controlPoints_[i + order, 1];
            }

            return solve.FindMinimum(obj, init).MinimizingPoint;
        }


        (float cost, List<Vector3> gradient) CalSmoothnessCost(List<Vector3> Points)
        {
            var gradient = new List<Vector3>();
            var cost = 0f;

            Vector3 jerk = new(), tmp_j = new();

            for (int i = 0, k = Points.Count; i < k; i++)
            {
                gradient.Add(new Vector3());
            }
            for (int i = 0, k = Points.Count - order; i < k; i++)
            {
                jerk = Points[i + 3] - 3 * Points[i + 2] + 3 * Points[i + 1] - Points[i];
                cost += jerk.LengthSquared();
                tmp_j = 2 * jerk;

                gradient[i + 0] += -tmp_j;
                gradient[i + 1] += 3.0f * tmp_j;
                gradient[i + 2] += -3.0f * tmp_j;
                gradient[i + 3] += tmp_j;
            }

            return (cost, gradient);
        }

        (float cost, List<Vector3> gradient) CalDistanceCost(List<Vector3> Points)
        {
            var gradient = new List<Vector3>();

            var cost = 0f;

            var dist = 0f;
            Vector3 dist_grad = new Vector3();//, g_zero = new Vector3();

            var end_idx = Points.Count - order;

            for (int i = 0; i < Points.Count; i++)
            {
                gradient.Add(new Vector3());
            }

            for (int i = 0; i < end_idx; i++)
            {
                var rst = CostMap.CalCostWithGrad(Points[i], -1);

                if (dist_grad.Length() > 1e-4) dist_grad = Vector3.Normalize(dist_grad);

                if (dist < 0.4f)
                {
                    cost += (float)Math.Pow(dist - 0.4f, 2);
                    gradient[i] += 2.0f * (dist - 0.4f) * dist_grad;
                }
            }
            return (cost, gradient);
        }

        (float cost, List<Vector3> gradient) CalFeasibilityCost(List<Vector3> Points)
        {
            var cost = 0f;
            var gradient = new List<Vector3>();

            for (int i = 0; i < Points.Count; i++)
            {
                gradient.Add(new Vector3());
            }

            Vector3 zero = new Vector3();

            float ts, vm2, am2, ts_inv2, ts_inv4;
            vm2 = limit_vel * limit_vel;
            am2 = limit_acc * limit_acc;

            ts = 10f;
            ts_inv2 = 1 / ts / ts;
            ts_inv4 = ts_inv2 * ts_inv2;

            // vel
            for (int i = 0; i < Points.Count - 1; i++)
            {
                Vector3 vi = Points[i + 1] - Points[i];
                float vx = vi.X * vi.X * ts_inv2 - vm2;
                float vy = vi.Y * vi.Y * ts_inv2 - vm2;
                float vz = vi.Z * vi.Z * ts_inv2 - vm2;
                float temp_x = 0;
                float temp_y = 0;
                float temp_z = 0;
                if (vx > 0.0)
                {
                    cost += (float)Math.Pow(vx, 2);
                    temp_x = 4.0f * vx * ts_inv2;
                }
                if (vy > 0.0)
                {
                    cost += (float)Math.Pow(vy, 2);
                    temp_y = 4.0f * vy * ts_inv2;
                }
                if (vz > 0.0)
                {
                    cost += (float)Math.Pow(vz, 2);
                    temp_z = 4.0f * vz * ts_inv2;
                }

                gradient[i + 0] -= new Vector3(temp_x * vi.X, temp_y * vi.Y, temp_z * vi.Z);
                gradient[i + 1] += new Vector3(temp_x * vi.X, temp_y * vi.Y, temp_z * vi.Z);
            }

            //acc
            for (int i = 0; i < Points.Count - 2; i++)
            {
                Vector3 ai = Points[i + 2] - 2 * Points[i + 1] + Points[i];
                float ax = ai.X * ai.X * ts_inv4 - am2;
                float ay = ai.Y * ai.Y * ts_inv4 - am2;
                float az = ai.Z * ai.Z * ts_inv4 - am2;
                float temp_x = 0;
                float temp_y = 0;
                float temp_z = 0;
                if (ax > 0.0)
                {
                    cost += (float)Math.Pow(ax, 2);
                    temp_x = 4.0f * ax * ts_inv4;
                }
                if (ay > 0.0)
                {
                    cost += (float)Math.Pow(ay, 2);
                    temp_y = 4.0f * ay * ts_inv4;
                }
                if (az > 0.0)
                {
                    cost += (float)Math.Pow(az, 2);
                    temp_z = 4.0f * az * ts_inv4;
                }

                gradient[i + 0] += new Vector3(temp_x * ai.X, temp_y * ai.Y, temp_z * ai.Z);
                gradient[i + 1] += -2 * new Vector3(temp_x * ai.X, temp_y * ai.Y, temp_z * ai.Z);
                gradient[i + 2] += new Vector3(temp_x * ai.X, temp_y * ai.Y, temp_z * ai.Z);
            }

            return (cost, gradient);
        }

        (double, MathNet.Numerics.LinearAlgebra.Vector<double>) CalCost(MathNet.Numerics.LinearAlgebra.Vector<double> input)
        {
            var gq = new List<Vector3>();
            for (int i = 0; i < order; i++)
            {
                gq.Add(new(nonUniformBspline.controlPoints_[i, 0], nonUniformBspline.controlPoints_[i, 1], 0));
            }
            var l = nonUniformBspline.n_ - order - order;
            l *= 2;
            for (int i = 0; i < l; i += 2)
            {
                gq.Add(new Vector3((float)input[i], (float)input[i + 1], 0));
            }
            for (int i = nonUniformBspline.n_ - order; i <= nonUniformBspline.n_; i++)
            {
                gq.Add(new Vector3(nonUniformBspline.controlPoints_[i, 0], nonUniformBspline.controlPoints_[i, 1], 0));
            }

            var f_combine = 0.0f;
            var grad = new double[l];


            var k = CalSmoothnessCost(gq);
            f_combine += lambda1 * k.cost;
            for (int i = 0; i < l; i += 2)
            {
                grad[i] += lambda1 * k.gradient[i / 2 + order].X;
                grad[i + 1] += lambda1 * k.gradient[i / 2 + order].Y;
            }

            k = CalDistanceCost(gq);
            f_combine += lambda2 * k.cost;
            for (int i = 0; i < l; i += 2)
            {
                grad[i] += lambda2 * k.gradient[i / 2 + order].X;
                grad[i + 1] += lambda2 * k.gradient[i / 2 + order].Y;
            }

            k = CalFeasibilityCost(gq);
            f_combine += lambda3 * k.cost;
            for (int i = 0; i < l; i += 2)
            {
                grad[i] += lambda3 * k.gradient[i / 2 + order].X;
                grad[i + 1] += lambda3 * k.gradient[i / 2 + order].Y;
            }
            return (f_combine, new MathNet.Numerics.LinearAlgebra.Double.DenseVector(grad));
        }

        internal Vector3 TargetAcc(float t) => nonUniformBspline.TargetAcc(t);
    }
}