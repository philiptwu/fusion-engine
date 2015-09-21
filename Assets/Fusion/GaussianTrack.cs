using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;

// The track class holds state, time, and a list of unfused measurements
// for processing by fusion/filter algorithms

namespace AutonomyTestbed.Fusion
{
    public class GaussianTrack
    {
        // Information
        public ulong id;
        public GaussianVector data;
        public List<GaussianVector> associatedMeasurements;
        public bool isInitialized;

        // Constructor 1: Uninitialized state 
        public GaussianTrack(ulong id)
        {
            // Initialize associated measurements list
            associatedMeasurements = new List<GaussianVector>();

            // Save track ID
            this.id = id;

            // Not initialized
            isInitialized = false;
        }

        // Constructor 2: Caller specifies initial state
        public GaussianTrack(ulong id, GaussianVector initialState)
            : this(id)
        {
            // Initialize using specified initial state
            Initialize(initialState);
        }

        // If the track is not already initialized, try to estimate using batch estimator.  Return initialization status.
        public void Initialize(GaussianVector initialState)
        {
            // Save the initial state
            data = initialState.Clone();

            // Initialized
            isInitialized = true;
        }

        // Coasts the current track to target dateTime without changing contents of current track
        public GaussianVector CoastTrack(StateTransitionModel stateTransitionModel, DateTime targetDateTime)
        {
            // Make a copy to hold result in
            GaussianVector coastedData = data.Clone();

            // Save old mean
            Vector oldMean = data.mean.Clone();

            // Predict state mean forward
            coastedData.mean = stateTransitionModel.Evaluate(oldMean, data.dateTime, targetDateTime);

            // Predict uncertainty using Jacobian
            Matrix F = stateTransitionModel.GetJacobian(oldMean, data.dateTime, targetDateTime);
            Matrix FT = F.Clone();
            FT.Transpose();
            coastedData.covariance = F * data.covariance * FT;

            // Update time stamp on track
            coastedData.dateTime = targetDateTime;

            // Return to caller, leaving current track unchanged
            return coastedData;
        }
    }
}
