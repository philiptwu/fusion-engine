﻿using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
#if(UNITY_STANDALONE)
using UnityEngine;
#endif

// The recursive estimator takes in measurements, a state + uncertainty, and computes a new state estimate + uncertainty
namespace AutonomyTestbed.Fusion
{
    public class ExtendedKalmanFilter
    {
        // Models
        public StateTransitionModel stateTransitionModel;
        public ProcessNoiseModel processNoiseModel;

        public ExtendedKalmanFilter(StateTransitionModel stateTransitionModel, ProcessNoiseModel processNoiseModel)
        {
            this.stateTransitionModel = stateTransitionModel;
            this.processNoiseModel = processNoiseModel;
        }

        // Performs a Kalman update on track using measurement and overwrites track with new estimate
        public void UpdateTrack(GaussianTrack track, GaussianMeasurement measurement)
        {
            // Predict track forward to measurement time and add process noise
            Matrix Q = processNoiseModel.Evaluate(track.gaussianVector.mean, track.dateTime, measurement.dateTime);
            GaussianVector predictedGaussianVector = track.CoastTrack(stateTransitionModel, measurement.dateTime);
            predictedGaussianVector.covariance += Q;

            // Compute residual mean / covariance            
            Vector hx;
            Matrix H;
            Coordinate.Convert(new Vector(6), Coordinate.Type.UNITY6, predictedGaussianVector.mean, measurement.creatorUnityReference, measurement.coordinateType, out hx, out H);
            
            // Compute innovation and Kalman gain
            Vector y = measurement.gaussianVector.mean - hx;
            Matrix HT = H.Clone();
            HT.Transpose();
            Matrix S = H * predictedGaussianVector.covariance * HT + measurement.gaussianVector.covariance;
            Matrix K = S.SolveTranspose(predictedGaussianVector.covariance * HT);

            // Update state estimate
            int N = predictedGaussianVector.mean.Length;
            //Debug.Log("K = (" + K.RowCount + "," + K.ColumnCount + ")");
            //Debug.Log("H = (" + H.RowCount + "," + H.ColumnCount + ")");
            //Debug.Log("I-KH = (" + (Matrix.Identity(N, N) - K * H).RowCount + "," + (Matrix.Identity(N, N) - K * H).ColumnCount + ")");
            //Debug.Log("P = (" + predictedGaussianVector.covariance.RowCount + "," + predictedGaussianVector.covariance.ColumnCount + ")");
            predictedGaussianVector.covariance = (Matrix.Identity(N,N) - K*H) * predictedGaussianVector.covariance; // Problem
            predictedGaussianVector.mean = predictedGaussianVector.mean + (K * y.ToColumnMatrix()).GetColumnVector(0);

            // Write to the track
            track.gaussianVector = predictedGaussianVector;
            track.dateTime = measurement.dateTime;
        }
    }
}
