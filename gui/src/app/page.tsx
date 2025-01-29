"use client";

import { useState, useEffect } from "react";
import { useRosParam } from "./hooks/useRosParam";
import { useRosSubscriber } from "./hooks/useRosSubscriber";
import { useLocations } from "./hooks/useLocations";
import { useMoveBase } from "./hooks/useMoveBase";

type PoseWithCovarianceStamped = {
  pose: {
    pose: {
      position: {
        x: number;
        y: number;
      };
    };
    covariance: number[];
  };
};

export default function Home() {
  const {
    data: pose,
    isSubscribed: isPoseSubscribed,
    error: poseError,
  } = useRosSubscriber(
    "/rtabmap/localization_pose",
    "geometry_msgs/PoseWithCovarianceStamped"
  );

  const {
    value: mode,
    setValue: setMode,
    getValue: getMode,
    isLoading: isModeLoading,
    error: modeError,
  } = useRosParam<string>("/mode");

  const { locations, loading, error, createLocation, deleteLocation } =
    useLocations();
  const {
    sendGoal,
    cancelGoal,
    status,
    result,
    feedback,
    error: moveBaseError,
  } = useMoveBase();

  const [showModal, setShowModal] = useState(false);
  const [locationName, setLocationName] = useState("");

  useEffect(() => {
    getMode();
  }, [getMode]);

  const handleCreateLocation = async () => {
    const poseToUse = pose as PoseWithCovarianceStamped;

    if (!isPoseSubscribed || poseError) {
      alert("Pose is not subscribed or there is an error.");
      return;
    }

    if (
      poseToUse.pose.covariance[0] >= 0.1 ||
      poseToUse.pose.covariance[7] >= 0.1
    ) {
      alert("Covariance on x or y is too high.");
      return;
    }

    await createLocation({
      name: locationName,
      x: parseFloat(poseToUse.pose.pose.position.x.toFixed(2)),
      y: parseFloat(poseToUse.pose.pose.position.y.toFixed(2)),
    });
    setShowModal(false);
    setLocationName("");
  };

  const handleDeleteLocation = async (id: string, name: string) => {
    const confirmDelete = window.confirm(`Delete location ${name}?`);
    if (confirmDelete) await deleteLocation(id);
  };

  const handleSendGoal = (location: { x: number; y: number }) => {
    const goal = {
      target_pose: {
        header: {
          frame_id: "map",
        },
        pose: {
          position: {
            x: location.x,
            y: location.y,
            z: 0,
          },
          orientation: {
            x: 0,
            y: 0,
            z: 0,
            w: 1,
          },
        },
      },
    };
    sendGoal(goal);
  };

  return (
    <main>
      <div className="text-center mt-4 col-md-6 mx-auto">
        <div className="mt-4">
          <h2 id="mode-heading">Mode</h2>
          {isModeLoading ? (
            <p>Loading...</p>
          ) : (
            <div>
              <p aria-live="polite" aria-atomic="true">
                {mode}
              </p>
              <button
                className="btn btn-primary"
                onClick={() =>
                  setMode(mode === "guidance" ? "sensing" : "guidance")
                }
                aria-label={`Toggle mode, current mode is ${mode}`}
              >
                Toggle Mode
              </button>
              {modeError && <p className="text-danger">{modeError}</p>}
            </div>
          )}
        </div>

        {mode === "guidance" && (
          <div className="mt-4">
            <h2 id="locations-heading">Locations</h2>
            {loading ? (
              <p>Loading...</p>
            ) : error ? (
              <p className="text-danger">{error}</p>
            ) : (
              <table
                className="table table-striped"
                aria-labelledby="locations-heading"
              >
                <thead>
                  <tr>
                    <th scope="col">Name</th>
                    <th scope="col">X</th>
                    <th scope="col">Y</th>
                    <th scope="col">Actions</th>
                  </tr>
                </thead>
                <tbody>
                  {locations.map((location) => (
                    <tr key={location._id}>
                      <td>{location.name}</td>
                      <td>{location.x}</td>
                      <td>{location.y}</td>
                      <td className="d-flex justify-content-evenly">
                        <button
                          className="btn btn-danger"
                          onClick={() =>
                            handleDeleteLocation(location._id!, location.name)
                          }
                          aria-label={`Delete location ${location.name}`}
                        >
                          Delete
                        </button>
                        <button
                          className="btn btn-success ml-2"
                          onClick={() => handleSendGoal(location)}
                          aria-label={`Send goal to location ${location.name}`}
                        >
                          Go
                        </button>
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            )}
            <button
              className="btn btn-primary"
              onClick={() => setShowModal(true)}
              aria-label="Add new location"
            >
              Add Location
            </button>
          </div>
        )}
      </div>

      {/* Modal */}
      {showModal && (
        <div
          className="modal show d-block"
          tabIndex={-1}
          role="dialog"
          aria-labelledby="modal-label"
          aria-hidden="true"
        >
          <div className="modal-dialog" role="document">
            <div className="modal-content">
              <div className="modal-header">
                <h5 className="modal-title" id="modal-label">
                  Add New Location
                </h5>
                <button
                  type="button"
                  className="close"
                  aria-label="Close"
                  onClick={() => setShowModal(false)}
                >
                  <span aria-hidden="true">&times;</span>
                </button>
              </div>
              <div className="modal-body">
                <div className="form-group">
                  <label htmlFor="location-name">Location Name</label>
                  <input
                    type="text"
                    className="form-control"
                    id="location-name"
                    value={locationName}
                    onChange={(e) => setLocationName(e.target.value)}
                    aria-label="Location Name"
                  />
                </div>
              </div>
              <div className="modal-footer">
                <button
                  type="button"
                  className="btn btn-secondary"
                  onClick={() => setShowModal(false)}
                >
                  Close
                </button>
                <button
                  type="button"
                  className="btn btn-primary"
                  onClick={handleCreateLocation}
                >
                  Save changes
                </button>
              </div>
            </div>
          </div>
        </div>
      )}
    </main>
  );
}
