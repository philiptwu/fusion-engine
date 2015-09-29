using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
#if(UNITY_STANDALONE)
using UnityEngine;
#endif

namespace AutonomyTestbed.Fusion
{
    public class SingleTrackInitializer
    {
        private FusionEngine fusionEngine;
        private WLSQBatchEstimator batchEstimator;

        public SingleTrackInitializer(FusionEngine fusionEngine, int minBatchMeasurements)
        {
            Debug.LogWarning("Warning: Track initialization not fully implemented yet, for now will try to put all unassociated measurements into a single track");

            this.fusionEngine = fusionEngine;
            this.batchEstimator = new WLSQBatchEstimator(minBatchMeasurements, fusionEngine);
        }

        // Tries to initialize tracks
        public void InitializeTracks()
        {
            // Try to initialize a track
            GaussianTrack newTrack = batchEstimator.Estimate(fusionEngine.unassociatedMeasurements);

            // If successful, then add it to the database
            if (newTrack != null)
            {
                fusionEngine.unassociatedMeasurements.Clear();
                fusionEngine.trackDatabase.Add(newTrack);
            }
        }
    }
}
