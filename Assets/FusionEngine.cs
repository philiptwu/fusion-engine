using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using AutonomyTestbed.Fusion;
using MathNet.Numerics.LinearAlgebra;

public class FusionEngine : MonoBehaviour {

    // Engine properties
    public float updatePeriodSec;
    private float timeSinceLastUpdateSec;

    // Lists
    public List<GaussianMeasurement> unprocessedMeasurements;
    public Dictionary<ulong, GaussianTrack> trackDatabase;

    // Components
    ChiSquareAssociator associator;

	// Use this for initialization
	void Start () 
    {
        // Initialize data structures
        unprocessedMeasurements = new List<GaussianMeasurement>();
        trackDatabase = new Dictionary<ulong, GaussianTrack>();

        // Initialize associator
        associator = new ChiSquareAssociator(this);

        // Reset time since last update
        timeSinceLastUpdateSec = 0.0f;
	}
	
	// Update is called once per frame
	void Update () 
    {
	    // Check if it is time to perform an udpate
        timeSinceLastUpdateSec += Time.deltaTime;
        if (timeSinceLastUpdateSec >= updatePeriodSec)
        {
            // Grab lock on track database
            lock (trackDatabase)
            {
                // Step 1: Run associator for each measurement
                lock (unprocessedMeasurements)
                {
                    associator.AssociateMeasurements();
                }

                // Step 2: Run filter on each track


                // Step 3: Run track management on each track


                // Clear time since last update
                timeSinceLastUpdateSec = 0.0f;
            }
        }
	}

    // Used by external code to add to list of unprocessed measurements
    // Do not try to modify the list directly
    public void AddMeasurement(GaussianMeasurement measurement)
    {
        // Add to the list of unprocessed, can't add any when 
        lock (unprocessedMeasurements)
        {
            unprocessedMeasurements.Add(measurement);
        }
    }
}
