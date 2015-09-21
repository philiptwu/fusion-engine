using System;
using MathNet.Numerics.LinearAlgebra;

namespace AutonomyTestbed.Fusion
{
    abstract public class MeasurementNoiseModel
    {
        abstract public Matrix Evaluate(DateTime time);
    }
}
