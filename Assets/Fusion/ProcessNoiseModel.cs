using System;
using MathNet.Numerics.LinearAlgebra;

namespace AutonomyTestbed.Fusion
{
    abstract public class ProcessNoiseModel
    {
        abstract public Matrix Evaluate(Vector state, DateTime stateTime, DateTime targetTime);
    }
}
