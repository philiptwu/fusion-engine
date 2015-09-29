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
    // Databases
    public Dictionary<uint, Dictionary<uint, List<GaussianMeasurement>>> unprocessedMeasurements;
    //public Dictionary<uint, Dictionary<uint, List<GaussianMeasurement>>> unassociatedMeasurements;
    public List<GaussianMeasurement> unassociatedMeasurements;
    public List<GaussianTrack> trackDatabase;
    public List<GaussianTrack> protoTrackDatabase;
    
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
        unprocessedMeasurements = new Dictionary<uint, Dictionary<uint, List<GaussianMeasurement>>>(); // platformID, sensorID, meas
        //unassociatedMeasurements = new Dictionary<uint, Dictionary<uint, List<GaussianMeasurement>>>(); // platformID, sensorID, meas
        unassociatedMeasurements = new List<GaussianMeasurement>();
        trackDatabase = new List<GaussianTrack>();
        protoTrackDatabase = new List<GaussianTrack>();

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
	}
	
	// Update is called once per frame
	void Update () 
    {
        // Do nothing
	}

    // Engine update, called when new measurements are added
    private void EngineUpdate()
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
            foreach (GaussianTrack gt in trackDatabase)
            {
                // Note: Associated measurements are in sorted list so update is 
                // being performed in chronological order
                foreach (GaussianMeasurement gm in gt.associatedMeasurements.Values)
                {
                    ekf.UpdateTrack(gt, gm);
                }
                gt.associatedMeasurements.Clear();

                Debug.Log("Track #" + gt.trackID + " mean: " + gt.gaussianVector.mean);
            }

            Debug.Log("******************************");
            Debug.Log("Unassociated Measurements: " + unassociatedMeasurements.Count);
            Debug.Log("Tracks: " + trackDatabase.Count);
        }
    }

    private void AddToTieredDictionary(Dictionary<uint, Dictionary<uint, List<GaussianMeasurement>>> d, GaussianMeasurement m)
    {
        // Create a new dictionary for this platform ID if one doesn't already exist
        if (!d.ContainsKey(m.platformID))
        {
            d[m.platformID] = new Dictionary<uint, List<GaussianMeasurement>>();
        }

        // Create a new dictionary for this sensor ID if one doesn't already exist
        if (!d[m.platformID].ContainsKey(m.sensorID))
        {
            d[m.platformID][m.sensorID] = new List<GaussianMeasurement>();
        }

        // Add to the appropriate list
        d[m.platformID][m.sensorID].Add(m);
    }

    public void AddUnassociatedMeasurement(GaussianMeasurement m)
    {
        lock (unassociatedMeasurements)
        {
            unassociatedMeasurements.Add(m);
        }
    }

    // Used by external code to add to list of unprocessed measurements
    // Do not try to modify the list directly
    public void AddScanMeasurements(List<GaussianMeasurement> measurements)
    {
        // Add to the list of unprocessed, can't add any when 
        lock (unprocessedMeasurements)
        {
            foreach (GaussianMeasurement m in measurements)
            {
                AddToTieredDictionary(unprocessedMeasurements, m);
            }
        }

        // Since new measurements have been added, update the engine as needed
        EngineUpdate();
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
