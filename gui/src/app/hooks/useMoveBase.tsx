"use client";

import { useState, useCallback, useMemo } from "react";
import ROSLIB from "roslib";
import { useRos } from "@/components/RosProvider";

interface MoveBaseGoal {
  target_pose: {
    header: {
      frame_id: string;
    };
    pose: {
      position: {
        x: number;
        y: number;
        z: number;
      };
      orientation: {
        x: number;
        y: number;
        z: number;
        w: number;
      };
    };
  };
}

interface UseMoveBaseReturn {
  sendGoal: (goal: MoveBaseGoal) => void;
  cancelGoal: () => void;
  status: string;
  result: any;
  feedback: any;
  error: string | null;
}

export function useMoveBase(): UseMoveBaseReturn {
  const ros = useRos();
  const [status, setStatus] = useState<string>("");
  const [result, setResult] = useState<any>(null);
  const [feedback, setFeedback] = useState<any>(null);
  const [error, setError] = useState<string | null>(null);

  const actionClient = useMemo(
    () =>
      new ROSLIB.ActionClient({
        ros: ros,
        serverName: "/move_base",
        actionName: "move_base_msgs/MoveBaseAction",
      }),
    [ros]
  );

  const sendGoal = useCallback(
    (goal: MoveBaseGoal) => {
      const rosGoal = new ROSLIB.Goal({
        actionClient: actionClient,
        goalMessage: goal,
      });

      rosGoal.on("status", (status) => {
        setStatus(status);
      });

      rosGoal.on("result", (result) => {
        setResult(result);
      });

      rosGoal.on("feedback", (feedback) => {
        setFeedback(feedback);
      });

      rosGoal.on("timeout", () => {
        setError("Goal timed out");
      });

      rosGoal.send();
    },
    [actionClient]
  );

  const cancelGoal = useCallback(() => {
    actionClient.cancel();
  }, [actionClient]);

  return {
    sendGoal,
    cancelGoal,
    status,
    result,
    feedback,
    error,
  };
}
