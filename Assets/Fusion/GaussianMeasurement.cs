using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;

// The Gaussian measurement class holds a Gaussian distribution, state, time, and creator information

namespace AutonomyTestbed.Fusion
{
    public class GaussianMeasurement
    {
        // Identification
        public readonly uint platformID;
        public readonly uint sensorID;
        public readonly ulong scanNumber;

        // Creator location
        public readonly Vector creatorUnityReference;

        // Data
        public readonly GaussianVector gaussianVector;
        public readonly Coordinate.Type coordinateType;
        public readonly DateTime dateTime;

        // Constructor
        public GaussianMeasurement(uint platformID, uint sensorID, ulong scanNumber, 
            Vector creatorUnityReference, GaussianVector gaussianVector, 
            Coordinate.Type coordinateType, DateTime dateTime)
        {
            this.platformID = platformID;
            this.sensorID = sensorID;
            this.scanNumber = scanNumber;
            this.creatorUnityReference = creatorUnityReference.Clone();
            this.gaussianVector = gaussianVector.Clone();
            this.coordinateType = coordinateType;
            this.dateTime = dateTime;
        }
    }
}
