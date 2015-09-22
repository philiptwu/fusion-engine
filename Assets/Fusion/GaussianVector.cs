using System;
using MathNet.Numerics.LinearAlgebra;

// Contents describe a Gaussian distribution only

namespace AutonomyTestbed.Fusion
{
    public class GaussianVector
    {
        // Information
        public Vector mean;
        public Matrix covariance;

        // Contents of a Measurement object are only specified once and cannot be changed after
        public GaussianVector(Vector mean, Matrix covariance)
        {
            // Information contained in measurement should be immutable
            this.mean = mean.Clone();
            this.covariance = covariance.Clone();
        }

        // Create a copy
        public GaussianVector Clone()
        {
            return new GaussianVector(mean, covariance);
        }
    }
}
