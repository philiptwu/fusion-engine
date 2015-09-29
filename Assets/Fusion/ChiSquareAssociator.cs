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

        // Algorithms
        private HungarianAlgorithm hungarianAlgorithm;

        // Internal storage
        //private List<GaussianTrack> protoTrackList;

        // Constructor
        public ChiSquareAssociator(FusionEngine fusionEngine, float chiSquareThreshold)
        {
            // Save pointer to the fusion engine
            this.fusionEngine = fusionEngine;
            this.chiSquareThreshold = chiSquareThreshold;

            // Initialize hungarian algorithm
            this.hungarianAlgorithm = new HungarianAlgorithm();

            // Initialize internal storage
            //protoTrackList = new List<GaussianTrack>();
        }

        // Associate measurements (lock assumed on unprocessedMeasurements and track database)
        public void Associate()
        {
            // For each platform ID
            foreach (uint platformID in fusionEngine.unprocessedMeasurements.Keys)
            {
                // For each sensor ID
                foreach(uint sensorID in fusionEngine.unprocessedMeasurements[platformID].Keys)
                {
                    // Get # of measurements with this platform and sensor ID
                    int numMeasurements = fusionEngine.unprocessedMeasurements[platformID][sensorID].Count;
                    if (numMeasurements == 0)
                    {
                        // Skip if no measurements to process
                        return;
                    }

                    // Create cost matrix
                    // Note: The implementation of the Hungarian algorithm that we are using requires a square cost matrix
                    int numTracks = fusionEngine.trackDatabase.Count;
                    int numTasks = numTracks + numMeasurements;
                    int[,] costMatrix = new int[numMeasurements, numTasks]; // measurement x track
                    int im=0;
                    foreach (GaussianMeasurement gm in fusionEngine.unprocessedMeasurements[platformID][sensorID])
                    {
                        // Populate entries corresponding to measurements to tracks
                        int it=0;
                        foreach (GaussianTrack gt in fusionEngine.trackDatabase)
                        {
                            double chiSquareDistance = Compute3DimChiSquareDistance(gm, gt);
                            if(chiSquareDistance > chiSquareThreshold){
                                // We are above the threshold, assign a large penalty value
                                costMatrix[im,it] = 1073741824; // 2^30
                            }else{
                                // We are within threshold, scale and assign
                                // Note: We do this because unfortunately the implementation of the
                                // Hungarian algorithm that's available only deals with integer costs
                                costMatrix[im,it] = (int)Math.Round(chiSquareDistance*1000);
                            }
                            // Increment column in cost matrix
                            it++;
                        }

                        // Populate entries corresponding to measurements to no assignments
                        while (it < numTasks)
                        {
                            costMatrix[im, it] = 536870912; // 2^29
                            it++;
                        }

                        // Increment row in cost matrix
                        im++;
                    }

                    // Run linear assignment problem
                    int[] assignment = hungarianAlgorithm.Solve(costMatrix);

                    // Make assignment based on results of Hungarian algorithm
                    for(int i=0; i < numMeasurements; i++)
                    {
                        GaussianMeasurement gm = fusionEngine.unprocessedMeasurements[platformID][sensorID][i];
                        if(assignment[i] < numTracks){
                            // Assigned to a track
                            fusionEngine.trackDatabase[assignment[i]].AssociateMeasurement(gm);
                        }else{
                            // Unassigned
                            fusionEngine.AddUnassociatedMeasurement(gm);
                        }
                    }

                    // Clear the list
                    fusionEngine.unprocessedMeasurements[platformID][sensorID].Clear();
                } // End iterating through sensor IDs
            } // End iterating through platform IDs
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
