"use client";

import { useEffect, useState } from "react";
import ROSLIB from "roslib";
import { useRos } from "@/components/RosProvider";

// Base interface for ROS messages
export interface RosMessage {
  [key: string]: any;
}

export interface UseRosSubscriberReturn {
  data: ROSLIB.Message | null;
  isSubscribed: boolean;
  error: string | null;
}

export function useRosSubscriber(
  topicName: string,
  messageType: string
): UseRosSubscriberReturn {
  const ros = useRos();
  const [data, setData] = useState<ROSLIB.Message | null>(null);
  const [isSubscribed, setIsSubscribed] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (!ros) {
      setError("ROS connection not initialized");
      return;
    }

    try {
      const topic = new ROSLIB.Topic({
        ros: ros,
        name: topicName,
        messageType: messageType,
      });

      topic.subscribe((message: ROSLIB.Message) => {
        setData(message);
        setIsSubscribed(true);
        setError(null);
      });

      return () => {
        topic.unsubscribe();
        setIsSubscribed(false);
        setData(null);
      };
    } catch (err) {
      setError(err instanceof Error ? err.message : "Unknown error");
      setIsSubscribed(false);
    }
  }, [topicName, messageType, ros]);

  return { data, isSubscribed, error };
}
