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
        public const float limit_acc = 2f + 1e-4f;
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

        public List<Vector2> Build(List<Vector2> WayPoints, List<Vector2> StartEndDerivative, int Order = 4, float Interval = 0.5f)
        {
            this.order = Order;
            nonUniformBspline.ParametersToControlPoints(WayPoints, StartEndDerivative);
            nonUniformBspline.BuildTimeLine(Interval);
            return nonUniformBspline.GetPath();
        }

        public (int Id, bool Safe) Update(float t, Vector2 Position)
        {
            return nonUniformBspline.Check(t, CostMap, Position);
        }

        public Vector3 TargetVeclocity(float t) => nonUniformBspline.GetVelocity(t);

        internal Vector3 TargetAcc(float t) => nonUniformBspline.GetAcceleration(t);
    }
}