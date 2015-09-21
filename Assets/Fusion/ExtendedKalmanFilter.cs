using System;
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
        public MeasurementNoiseModel measurementNoiseModel;

        public ExtendedKalmanFilter(StateTransitionModel stateTransitionModel, ProcessNoiseModel processNoiseModel,
            MeasurementModel measurementModel, MeasurementNoiseModel measurementNoiseModel)
        {
            this.stateTransitionModel = stateTransitionModel;
            this.processNoiseModel = processNoiseModel;
            this.measurementModel = measurementModel;
            this.measurementNoiseModel = measurementNoiseModel;
        }

        // Performs a Kalman update on track using measurement and overwrites track with new estimate
        public void UpdateTrack(GaussianTrack track, GaussianVector measurement)
        {
            // Predict track forward to measurement time and add process noise
            Matrix Q = processNoiseModel.Evaluate(track.data.mean, track.data.dateTime, measurement.dateTime);
            GaussianVector predictedDistribution = track.CoastTrack(stateTransitionModel, measurement.dateTime);
            predictedDistribution.covariance += Q;

            // Compute residual mean / covariance            
            Vector y = measurement.mean - measurementModel.Evaluate(predictedDistribution.mean, predictedDistribution.dateTime);
            Matrix H = measurementModel.GetJacobian(predictedDistribution.mean, predictedDistribution.dateTime);
            Matrix HT = H.Clone();
            HT.Transpose();
            Matrix R = measurementNoiseModel.Evaluate(predictedDistribution.dateTime);
            Matrix S = H * predictedDistribution.covariance * HT + R;

            // Compute Kalman gain
            Matrix K = S.SolveTranspose(predictedDistribution.covariance * HT);

            // Update state estimate
            int N = predictedDistribution.mean.Length;
            predictedDistribution.covariance= (Matrix.Identity(N,N) - K*H)*predictedDistribution.covariance;
            predictedDistribution.mean = predictedDistribution.mean + (K * y.ToColumnMatrix()).GetColumnVector(0);

            // Write to the track
            track.data = predictedDistribution;
        }
    }
}
