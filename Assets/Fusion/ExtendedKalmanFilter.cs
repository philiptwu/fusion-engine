﻿using System;
using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;

// The recursive estimator takes in measurements, a state + uncertainty, and computes a new state estimate + uncertainty
namespace AutonomyTestbed.Fusion
{
    public class ExtendedKalmanFilter
    {
        // Models
        public StateTransitionModel stateTransitionModel;
        public ProcessNoiseModel processNoiseModel;
        public MeasurementModel measurementModel;

        public ExtendedKalmanFilter(StateTransitionModel stateTransitionModel, ProcessNoiseModel processNoiseModel,
            MeasurementModel measurementModel)
        {
            this.stateTransitionModel = stateTransitionModel;
            this.processNoiseModel = processNoiseModel;
            this.measurementModel = measurementModel;
        }

        // Performs a Kalman update on track using measurement and overwrites track with new estimate
        public void UpdateTrack(GaussianTrack track, GaussianMeasurement measurement)
        {
            // Predict track forward to measurement time and add process noise
            Matrix Q = processNoiseModel.Evaluate(track.gaussianVector.mean, track.dateTime, measurement.dateTime);
            GaussianVector predictedGaussianVector = track.CoastTrack(stateTransitionModel, measurement.dateTime);
            predictedGaussianVector.covariance += Q;

            // Compute residual mean / covariance            
            Vector y = measurement.gaussianVector.mean - measurementModel.Evaluate(predictedGaussianVector.mean, measurement.dateTime);
            Matrix H = measurementModel.GetJacobian(predictedGaussianVector.mean, measurement.dateTime);
            Matrix HT = H.Clone();
            HT.Transpose();
            Matrix S = H * predictedGaussianVector.covariance * HT + measurement.gaussianVector.covariance;

            // Compute Kalman gain
            Matrix K = S.SolveTranspose(predictedGaussianVector.covariance * HT);

            // Update state estimate
            int N = predictedGaussianVector.mean.Length;
            predictedGaussianVector.covariance= (Matrix.Identity(N,N) - K*H)*predictedGaussianVector.covariance;
            predictedGaussianVector.mean = predictedGaussianVector.mean + (K * y.ToColumnMatrix()).GetColumnVector(0);

            // Write to the track
            track.gaussianVector = predictedGaussianVector;
            track.dateTime = measurement.dateTime;
        }
    }
}
