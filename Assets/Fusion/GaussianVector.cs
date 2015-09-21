using System;
using MathNet.Numerics.LinearAlgebra;

namespace AutonomyTestbed.Fusion
{
    public class GaussianVector
    {
        // Information
        public string creator;
        public DateTime dateTime;
        public Coordinate.Type coordinateType;
        public Vector mean;
        public Matrix covariance;

        // Contents of a Measurement object are only specified once and cannot be changed after
        public GaussianVector(string creator, DateTime dateTime, Coordinate.Type coordinateType, Vector mean, Matrix covariance)
        {
            // Information contained in measurement should be immutable
            this.creator = creator;
            this.dateTime = dateTime; // DateTime is a value type, not a reference
            this.coordinateType = coordinateType;
            this.mean = mean.Clone();
            this.covariance = covariance.Clone();
        }

        // Create a copy
        public GaussianVector Clone()
        {
            return new GaussianVector(creator, dateTime, coordinateType, mean, covariance);
        }
    }
}
