using System;
using MathNet.Numerics.LinearAlgebra;

namespace AutonomyTestbed.Fusion
{
    public class ConstantVelocityModel : StateTransitionModel
    {
        public override Vector Evaluate(Vector state, DateTime stateTime, DateTime targetTime)
        {
            // Compute dt
            TimeSpan ts = targetTime.Subtract(stateTime);
            double dt = ts.TotalSeconds;

            // Create coasted vector
            Vector coasted = state.Clone();
            coasted[0] = state[0] + state[3] * dt;
            coasted[1] = state[1] + state[4] * dt;
            coasted[2] = state[2] + state[5] * dt;
            
            // Return
            return coasted;
        }

        public override Matrix GetJacobian(Vector state, DateTime stateTime, DateTime targetTime)
        {
            // Compute dt
            TimeSpan ts = targetTime.Subtract(stateTime);
            double dt = ts.TotalSeconds;
           
            // Create Jacobian
            Matrix J = Matrix.Identity(state.Length, state.Length);
            J[0, 3] = dt;
            J[1, 4] = dt;
            J[2, 5] = dt;

            // Return
            return J;
        }
    }
}
