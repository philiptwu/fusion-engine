using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using AutonomyTestbed.Fusion;
using MathNet.Numerics.LinearAlgebra;

// TODO LIST:
// 1. Test on Unity
// 2. M of N track initialization
// 3. Track management
// 4. Cleanup / Unity-ify
// 5. Generalize

public class FusionEngine : MonoBehaviour {

    // Engine properties
    public float updatePeriodSec;
    private float timeSinceLastUpdateSec;

    // Databases
    public List<GaussianMeasurement> unprocessedMeasurements;
    public List<GaussianMeasurement> unassociatedMeasurements;
    public Dictionary<ulong, GaussianTrack> trackDatabase;
    
    // Track ID bookkeeping
    private HashSet<ulong> usedTrackIDs;
    private ulong availableTrackID;

    // Models
    public StateTransitionModel stateTransitionModel;
    public ProcessNoiseModel processNoiseModel;

    // Components
    private ChiSquareAssociator associator;
    private SingleTrackInitializer initializer;
    private ExtendedKalmanFilter ekf;

	// Use this for initialization
	void Start () 
    {
        // Initialize databases
        unprocessedMeasurements = new List<GaussianMeasurement>();
        unassociatedMeasurements = new List<GaussianMeasurement>();
        trackDatabase = new Dictionary<ulong, GaussianTrack>();

        // Initialize track ID bookkeeping
        usedTrackIDs = new HashSet<ulong>();
        availableTrackID = 0;

        // Initialize models
        stateTransitionModel = new ConstantVelocityModel();
        processNoiseModel = new RandomAccelerationModel(2);

        // Initialize components
        associator = new ChiSquareAssociator(this, 16.2662f);
        initializer = new SingleTrackInitializer(this, 5);
        ekf = new ExtendedKalmanFilter(stateTransitionModel, processNoiseModel);
                
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
                lock (unassociatedMeasurements)
                {
                    // Step 1: Try to associate measurements to released tracks
                    lock (unprocessedMeasurements)
                    {
                        associator.Associate();
                    }

                    // Step 2: Use unassociated measurements to try to update proto tracks
                    initializer.InitializeTracks();
                }
                // Step 3: Run track management on each released track
                // Note: This step can be omitted for now

                // Step 4: Run filter on each released track
                foreach (GaussianTrack gt in trackDatabase.Values)
                {
                    // Note: Associated measurements are in sorted list so update is 
                    // being performed in chronological order
                    foreach (GaussianMeasurement gm in gt.associatedMeasurements.Values)
                    {
                        ekf.UpdateTrack(gt, gm);
                    }
                    gt.associatedMeasurements.Clear();
                }

                // Clear time since last update
                timeSinceLastUpdateSec = 0.0f;

                Debug.Log("******************************");
                Debug.Log("Unassociated Measurements: " + unassociatedMeasurements.Count);
                Debug.Log("Tracks: " + trackDatabase.Values.Count);
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

    // Requests an available track ID
    public ulong RequestNewTrackID()
    {
        ulong newTrackID;

        lock (usedTrackIDs)
        {
            // Save track ID to assign
            newTrackID = availableTrackID;

            // Mark this track ID as taken
            usedTrackIDs.Add(availableTrackID);

            // Keep incrementing available track ID until one is available
            // for the next call
            while (usedTrackIDs.Contains(availableTrackID))
            {
                if (availableTrackID == ulong.MaxValue)
                {
                    // Wrap around for next
                    availableTrackID = 0;
                }
                else
                {
                    // Next is just +1
                    availableTrackID++;
                }
            }
        }

        // Return the new track ID to caller
        return newTrackID; 
    }

    // Sets a track ID as unused
    public void UnregisterTrackID(ulong trackID)
    {
        lock (usedTrackIDs)
        {
            usedTrackIDs.Remove(trackID);
        }
    }
}
