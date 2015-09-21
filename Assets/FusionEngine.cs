using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using AutonomyTestbed.Fusion;

public class FusionEngine : MonoBehaviour {

    // Engine properties
    public int id;
    public float updatePeriodSec;
    private float timeSinceLastUpdateSec;

    // Lists
    public Dictionary<int id, GaussianTrack> trackDatabase;

	// Use this for initialization
	void Start () {
        timeSinceLastUpdateSec = 0.0f;
	}
	
	// Update is called once per frame
	void Update () {
	    // Check if it is time to perform an udpate
        timeSinceLastUpdateSec += Time.deltaTime;
        if (timeSinceLastUpdateSec >= updatePeriodSec)
        {
            // Run associator for each measurement


            // Run filter on each track


            // Run track mgmt on each track


            // Clear time since last update
            timeSinceLastUpdateSec = 0.0f;
        }
	}

    public void AddMeasurement(GaussianVector measuremnt)
    {
        // Add to the associator
    }
}
