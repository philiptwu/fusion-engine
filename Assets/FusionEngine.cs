using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using AutonomyTestbed.Fusion;
using MathNet.Numerics.LinearAlgebra;

public class FusionEngine : MonoBehaviour {

    // Engine properties
    public int id;
    public float updatePeriodSec;
    private float timeSinceLastUpdateSec;

    // Lists
    public List<GaussianMeasurement> unprocessedMeasurements;
    public Dictionary<ulong, GaussianTrack> trackDatabase;

	// Use this for initialization
	void Start () {
        // Initialize data structures
        unprocessedMeasurements = new List<GaussianMeasurement>();
        trackDatabase = new Dictionary<ulong, GaussianTrack>();

        // Reset time since last update
        timeSinceLastUpdateSec = 0.0f;
	}
	
	// Update is called once per frame
	void Update () {
	    // Check if it is time to perform an udpate
        timeSinceLastUpdateSec += Time.deltaTime;
        if (timeSinceLastUpdateSec >= updatePeriodSec)
        {
            // Coordinate transform measurement into tracker frame
            // Note: An alternate design can instead transform tracks
            // to the measurement frame


            // Run associator for each measurement


            // Run filter on each track


            // Run track mgmt on each track


            // Clear time since last update
            timeSinceLastUpdateSec = 0.0f;
        }
	}

    public void AddMeasurement(GaussianMeasurement measurement)
    {
        // Add to the list of unprocessed
        unprocessedMeasurements.Add(measurement);
    }
}
