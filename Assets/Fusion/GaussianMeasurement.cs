using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;

// The Gaussian measurement class holds a Gaussian distribution, state, time, and creator information

namespace AutonomyTestbed.Fusion
{
    public class GaussianMeasurement
    {
        // Creator Information
        public readonly Vector creatorUnityReference;

        // Data
        public readonly GaussianVector gaussianVector;
        public readonly Coordinate.Type coordinateType;
        public readonly DateTime dateTime;

        // Constructor
        public GaussianMeasurement(Vector creatorUnityReference, GaussianVector gaussianVector, 
            Coordinate.Type coordinateType, DateTime dateTime)
        {
            this.creatorUnityReference = creatorUnityReference.Clone();
            this.gaussianVector = gaussianVector.Clone();
            this.coordinateType = coordinateType;
            this.dateTime = dateTime;
        }
    }
}
