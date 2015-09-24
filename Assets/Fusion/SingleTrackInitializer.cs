using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;

namespace AutonomyTestbed.Fusion
{
    public class SingleTrackInitializer
    {
        private FusionEngine fusionEngine;
        private WLSQBatchEstimator batchEstimator;

        public SingleTrackInitializer(FusionEngine fusionEngine)
        {
            this.fusionEngine = fusionEngine;
            this.batchEstimator = new WLSQBatchEstimator(3, fusionEngine);
        }

        // Tries to initialize tracks
        public void InitializeTracks()
        {
            // Try to initialize a track
            GaussianTrack newTrack = batchEstimator.Estimate(fusionEngine.unassociatedMeasurements);
            
            // If successful, then add it to the database
            if (newTrack != null)
            {
                fusionEngine.trackDatabase.Add(newTrack.trackID, newTrack);
            }
        }
    }
}
