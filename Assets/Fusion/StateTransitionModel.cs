using System;
using MathNet.Numerics.LinearAlgebra;

namespace AutonomyTestbed.Fusion
{
    abstract public class StateTransitionModel
    {
        abstract public Vector Evaluate(Vector state, DateTime stateTime, DateTime targetTime);
        abstract public Matrix GetJacobian(Vector state, DateTime stateTime, DateTime targetTime);
    }
}
