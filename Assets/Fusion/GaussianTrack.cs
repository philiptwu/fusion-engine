using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;

namespace AutonomyTestbed.Fusion
{
    public class GaussianTrack
    {
        // Identification
        public ulong trackID;

        // Track information
        public GaussianVector gaussianVector;
        public Coordinate.Type coordinateType;
        public DateTime dateTime;

        // Bookkeeping
        public SortedList<DateTime, GaussianMeasurement> associatedMeasurements;

        // Creating an uninitialized track with an associated measurement
        public GaussianTrack(ulong trackID, GaussianVector gaussianVector, Coordinate.Type coordinateType, DateTime dateTime)
        {
            // Save track initial information
            this.trackID = trackID;
            this.gaussianVector = gaussianVector.Clone();
            this.coordinateType = coordinateType;
            this.dateTime = dateTime;

            // Initialize database
            associatedMeasurements = new SortedList<DateTime, GaussianMeasurement>();
        }

        // Add to list
        public void AssociateMeasurement(GaussianMeasurement m)
        {
            lock (associatedMeasurements)
            {
                associatedMeasurements.Add(m.dateTime, m);
            }
        }

        // Returns the current track's distribution coasted to target dateTime w/o changing contents of current track
        public GaussianVector CoastTrack(StateTransitionModel stateTransitionModel, DateTime targetDateTime)
        {
            // Make a copy to hold result in
            GaussianVector coastedData = gaussianVector.Clone();

            // Save old mean
            Vector oldMean = gaussianVector.mean.Clone();

            // Predict state mean forward
            coastedData.mean = stateTransitionModel.Evaluate(oldMean, dateTime, targetDateTime);

            // Predict uncertainty using Jacobian
            Matrix F = stateTransitionModel.GetJacobian(oldMean, dateTime, targetDateTime);
            Matrix FT = F.Clone();
            FT.Transpose();
            coastedData.covariance = F * gaussianVector.covariance * FT;

            // Return to caller, leaving current track unchanged
            return coastedData;
        }
    }
}
