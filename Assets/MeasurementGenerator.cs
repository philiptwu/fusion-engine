using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using AutonomyTestbed.Fusion;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Distributions;

public class MeasurementGenerator : MonoBehaviour {    
    private FusionEngine fusionEngine;
    private GameObject[] targets;
    private Matrix noiseCovariance;
    private Matrix noiseCovCholT;
    private NormalDistribution nd;
    public float updatePeriodSec;
    private float timeSinceLastUpdateSec;

	// Use this for initialization
	void Start () 
    {
        // Save a pointer to the fusion engine
        fusionEngine = GetComponentInParent<FusionEngine>();

        // Get a pointer to the target
        targets = GameObject.FindGameObjectsWithTag("Target");

        // Noise distribution
        nd = new NormalDistribution(0, 1);
        noiseCovariance = new Matrix(3, 3);
        noiseCovariance[0, 0] = 1e-3;
        noiseCovariance[1, 1] = 1e-3;
        noiseCovariance[2, 2] = 1e-3;

        noiseCovCholT = noiseCovariance.CholeskyDecomposition.TriangularFactor.Clone();
        noiseCovCholT.Transpose();

        // Reset time since the last update
        timeSinceLastUpdateSec = 0.0f;
    }
	
	// Update is called once per frame
	void Update () 
    {
		// Check if it is time to perform an update
        timeSinceLastUpdateSec += Time.deltaTime;
        if (timeSinceLastUpdateSec >= updatePeriodSec)
        {
            // Save where I am
            Vector ownshipPosUnity = new Vector(6);
            ownshipPosUnity[0] = gameObject.transform.position.x;
            ownshipPosUnity[1] = gameObject.transform.position.y;
            ownshipPosUnity[2] = gameObject.transform.position.z;
            ownshipPosUnity[3] = 0;
            ownshipPosUnity[4] = 0;
            ownshipPosUnity[5] = 0;

            // New scan
            List<GaussianMeasurement> ms = new List<GaussianMeasurement>();
            
            // Take a noisy measurement of each target
            for (int i = 0; i < targets.Length; i++)
            {
                // Get target position
                Vector targetPosUnity = GetUnityPosition(targets[i]);

                // Generate gaussian noise
                Vector gaussianNoise = new Vector(3);
                gaussianNoise[0] = nd.NextDouble();
                gaussianNoise[1] = nd.NextDouble();
                gaussianNoise[2] = nd.NextDouble();
                gaussianNoise = (noiseCovCholT * gaussianNoise.ToColumnMatrix()).GetColumnVector(0);

                // Generate measurement
                GaussianMeasurement m = new GaussianMeasurement(1, 1, ownshipPosUnity,
                    new GaussianVector(targetPosUnity + gaussianNoise, noiseCovariance),
                    Coordinate.Type.UNITY3,
                    System.DateTime.Now);
                ms.Add(m);
            }

            // Add to fusion engine to be processed
            fusionEngine.AddScanMeasurements(ms);

            // Clear time since last update
            timeSinceLastUpdateSec = 0.0f;
        }
	}

    // Get position
    private Vector GetUnityPosition(GameObject g)
    {
        Vector position = new Vector(3);
        position[0] = g.transform.position.x;
        position[1] = g.transform.position.y;
        position[2] = g.transform.position.z;
        return position;
    }
}
