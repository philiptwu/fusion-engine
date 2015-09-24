using System;
using MathNet.Numerics.LinearAlgebra;

namespace AutonomyTestbed.Fusion
{
    public class RandomAccelerationModel : ProcessNoiseModel
    {
        // Noise parameter
        double q;

        // Constructor
        public RandomAccelerationModel(double q)
        {
            this.q = q;
        }

        public override Matrix Evaluate(Vector state, DateTime stateTime, DateTime targetTime)
        {
            // Compute dt
            TimeSpan ts = targetTime.Subtract(stateTime);
            double dt = ts.TotalSeconds;

            // Precompute Q matrix quantities
            Matrix Q = new Matrix(6, 6, 0);
            double q1 = q*dt;
            double q2 = q*dt*dt/2;
            double q3 = q*dt*dt*dt/3;

            // Populate main diagonal entries
            Q[0, 0] = q3;
            Q[1, 1] = q3;
            Q[2, 2] = q3;
            Q[3, 3] = q1;
            Q[4, 4] = q1;
            Q[5, 5] = q1;

            // Populate off diagonal entries
            Q[0,3] = q2;
            Q[1,4] = q2;
            Q[2,5] = q2;
            Q[3,0] = q2;
            Q[4,1] = q2;
            Q[5,2] = q2;

            // Return
            return Q;
        }
    }
}
