using System;
using MathNet.Numerics.LinearAlgebra;

namespace AutonomyTestbed.Fusion
{
    abstract public class MeasurementModel
    {
        abstract public Vector Evaluate(Vector state, DateTime time);
        abstract public Matrix GetJacobian(Vector state, DateTime time);
    }
}
