using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
#if(UNITY_STANDALONE)
using UnityEngine;
#endif

namespace AutonomyTestbed.Fusion
{
    public class ChiSquareAssociator
    {
        // Threshold
        private float chiSquareThreshold;

        // Pointers
        private FusionEngine fusionEngine;

        // Internal storage
        //private List<GaussianTrack> protoTrackList;

        // Constructor
        public ChiSquareAssociator(FusionEngine fusionEngine, float chiSquareThreshold)
        {
            // Save pointer to the fusion engine
            this.fusionEngine = fusionEngine;
            this.chiSquareThreshold = chiSquareThreshold;

            // Initialize internal storage
            //protoTrackList = new List<GaussianTrack>();
        }

        // Associate measurements (lock assumed on unprocessedMeasurements and track database)
        public void Associate()
        {
            // Copy from unprocessed measurements to unassociated and clear list
            foreach (GaussianMeasurement gm in fusionEngine.unprocessedMeasurements)
            {
                // Only take those that are gaussian measurements
                GaussianTrack bestTrack = null;
                double lowestChiSquareDistance = double.PositiveInfinity;
                
                // Try to associate to a track
                foreach(GaussianTrack gt in fusionEngine.trackDatabase.Values)
                {
                    // Only consider those that have chi square distances below threshold
                    double tempChiSquareDistance = Compute3DimChiSquareDistance(gm, gt);
                    if (tempChiSquareDistance <= chiSquareThreshold)
                    {
                        // Keep track of best match
                        if (tempChiSquareDistance < lowestChiSquareDistance)
                        {
                            lowestChiSquareDistance = tempChiSquareDistance;
                            bestTrack = gt;
                        }
                    }
                }

                // Evaluate association result
                if (bestTrack != null)
                {
                    // If a good association found to an existing track, add measurement to it
                    bestTrack.AssociateMeasurement(gm);
                }
                else
                {
                    // Otherwise, add to list of unassociated measurements
                    fusionEngine.unassociatedMeasurements.Add(gm);
                }
            }

            // Clear the list of unprocessed measurements when done
            fusionEngine.unprocessedMeasurements.Clear();
        }
 
        // Compare against result of chi2inv(p,3) where p represents desired probability for test
        private double Compute3DimChiSquareDistance(GaussianMeasurement measurement, GaussianTrack track)
        {
            // Coast track to measurement time using state transition model
            GaussianVector coastedTrack = track.CoastTrack(fusionEngine.stateTransitionModel, measurement.dateTime);

            // Only take 3 dimensional components of coasted track
            Vector coastedMean = VectorUtilities.Resize(coastedTrack.mean,3);
            Matrix coastedCovariance = VectorUtilities.Resize(coastedTrack.covariance,3,3);

            // Convert measurement to UNITY (track) 3 dimensional coordinates
            Vector convMeasurementMean;
            Matrix convMeasurementJacobian;
            Coordinate.Convert(measurement.creatorUnityReference, measurement.coordinateType, measurement.gaussianVector.mean,
                new Vector(3), Coordinate.Type.UNITY3, out convMeasurementMean, out convMeasurementJacobian);
            Matrix convMeasurementJacobianT = convMeasurementJacobian.Clone();
            convMeasurementJacobianT.Transpose();
            Matrix convMeasurementCovariance = convMeasurementJacobian * measurement.gaussianVector.covariance * convMeasurementJacobianT;

            // Compute 3 dimensional chi-square metric 
            Matrix meanDisplacement = (coastedMean - convMeasurementMean).ToColumnMatrix();
            Matrix meanDisplacementT = meanDisplacement.Clone();
            meanDisplacementT.Transpose();
            return (meanDisplacementT*((coastedCovariance + convMeasurementCovariance).Solve(meanDisplacement)))[0,0];
        }
    }
}
