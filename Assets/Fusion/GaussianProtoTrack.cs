using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;

namespace AutonomyTestbed.Fusion
{
    public class GaussianProtoTrack
    {
        // Bookkeeping
        public List<GaussianMeasurement> associatedMeasurements;

        // Creating an uninitialized track with an associated measurement
        public GaussianProtoTrack(GaussianMeasurement m)
        {
            AssociateMeasurement(m);
        }

        // Add to list
        public void AssociateMeasurement(GaussianMeasurement m)
        {
            lock (associatedMeasurements)
            {
                associatedMeasurements.Add(m);
            }
        }
    }
}
