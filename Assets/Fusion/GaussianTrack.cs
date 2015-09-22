using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

// The track class holds state, time, and a list of unfused measurements
// for processing by fusion/filter algorithms

namespace AutonomyTestbed.Fusion
{
    public class GaussianTrack
    {
        // Identification
        public ulong trackID;
        public GameObject creator;
        
        // Track information
        public GaussianVector gaussianVector;
        public Coordinate.Type coordinateType;
        public DateTime dateTime;

        // Bookkeeping
        public List<GaussianVector> associatedMeasurements;
        public bool isInitialized;

        // Constructor 1: Uninitialized state 
        public GaussianTrack(ulong trackID, GameObject creator)
        {
            // Initialize associated measurements list
            associatedMeasurements = new List<GaussianVector>();

            // Save identification information
            this.trackID = trackID;
            this.creator = creator;

            // Not initialized
            isInitialized = false;
        }

        // Constructor 2: Caller specifies initial state
        public GaussianTrack(ulong trackID, GameObject creator, GaussianVector initialState, Coordinate.Type coordinateType, DateTime dateTime)
            : this(trackID, creator)
        {
            // Initialize using specified initial state
            Initialize(initialState, coordinateType, dateTime);
        }

        // If the track is not already initialized, try to estimate using batch estimator.  Return initialization status.
        public void Initialize(GaussianVector initialState, Coordinate.Type coordinateType, DateTime dateTime)
        {
            // Save the information
            gaussianVector = initialState.Clone();
            this.coordinateType = coordinateType;
            this.dateTime = dateTime;

            // Initialized
            isInitialized = true;
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
