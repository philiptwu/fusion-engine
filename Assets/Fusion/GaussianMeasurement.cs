using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

// The Gaussian measurement class holds a Gaussian distribution, state, time, and creator information

namespace AutonomyTestbed.Fusion
{
    public class GaussianMeasurement
    {
        // Creator Information
        public readonly GameObject creator;

        // Data
        public readonly GaussianVector gaussianVector;
        public readonly Coordinate.Type coordinateType;
        public readonly DateTime dateTime;

        // Constructor
        public GaussianMeasurement(GameObject creator, GaussianVector gaussianVector, 
            Coordinate.Type coordinateType, DateTime dateTime)
        {
            this.creator = creator;
            this.gaussianVector = gaussianVector.Clone();
            this.coordinateType = coordinateType;
            this.dateTime = dateTime;
        }
    }
}
