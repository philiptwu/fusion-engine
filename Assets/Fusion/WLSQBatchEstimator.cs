using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using System.Linq;
#if(UNITY_STANDALONE)
using UnityEngine;
#endif

// Note: Hard coded for 3 state unity inputs with constant velocity model
namespace AutonomyTestbed.Fusion
{
    public class WLSQBatchEstimator
    {
        // Internal
        private int minMeasurements;
        private FusionEngine fusionEngine;

        // Constructor
        public WLSQBatchEstimator(int minMeasurements, FusionEngine fusionEngine)
        {
            this.minMeasurements = minMeasurements;
            this.fusionEngine = fusionEngine;
        }

        // Performs a Kalman update on track using measurement and overwrites track with new estimate
        public GaussianTrack Estimate(List<GaussianMeasurement> measurements)
        {
            // Understand input
            int N = measurements.Count;

            // Check if we have sufficient samples to estimate
            if (N < minMeasurements)
            {
                return null;
            }
            
            // Sort the list by datetime
            measurements.Sort((x,y) => x.dateTime.CompareTo(y.dateTime));

            // Prealloc structures
            Matrix A = new Matrix(3 * N, 6, 0);
            Matrix g = new Matrix(3 * N, 1, 0);
            Matrix R = new Matrix(3 * N, 3 * N, 0);

            // Go through each measurements
            for (int k = 0; k < N; k++)
            {
                // Compute time difference
                double dt = measurements[k].dateTime.Subtract(measurements[N - 1].dateTime).TotalSeconds;
                
                // Populate (3x6) block of A
                A[3 * k, 0] = 1;
                A[3 * k + 1, 1] = 1;
                A[3 * k + 2, 2] = 1;
                A[3 * k, 3] = dt;
                A[3 * k + 1, 4] = dt;
                A[3 * k + 2, 5] = dt;

                // Convert measurement to Unity (track) space
                Vector convMean;
                Matrix convJacobian;
                Coordinate.Convert(measurements[k].creatorUnityReference, measurements[k].coordinateType,
                    measurements[k].gaussianVector.mean, new Vector(3), Coordinate.Type.UNITY3,
                    out convMean, out convJacobian);
                Matrix convJacobianT = convJacobian.Clone();
                convJacobianT.Transpose();
                Matrix convCovariance = convJacobian * measurements[k].gaussianVector.covariance * convJacobianT;

                // Populate (3x1) block of g with converted measurement mean
                g[3 * k, 0] = convMean[0]; 
                g[3 * k + 1, 0] = convMean[1];
                g[3 * k + 2, 0] = convMean[2];

                // Copy coverted measurement covariance to (3x3) block in R matrix
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        R[3*k + i, 3*k + j] = convCovariance[i, j];
                    }
                }
            }

            // Finished building A, g, R matrices
            // Compute weighted least squares fit
            // Ps = inv(A'*inv(R)*A)
            // Xs = Ps*A'*inv(R)*g
            Matrix AT = A.Clone();
            AT.Transpose();
            Matrix Ps = (AT * R.Solve(A)).Inverse();
            Vector Xs = (Ps * AT * R.Solve(g)).GetColumnVector(0);

            // Create a new track and return
            return new GaussianTrack(fusionEngine.RequestNewTrackID(), 
                new GaussianVector(Xs, Ps), Coordinate.Type.UNITY6, 
                measurements[N - 1].dateTime);
        }
    }
}
