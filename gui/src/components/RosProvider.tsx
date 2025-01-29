"use client";

import React, { createContext, useContext, useEffect, useState } from "react";
import ROSLIB from "roslib";

const rosBridgeUrl = process.env.NEXT_PUBLIC_ROS_URL || "ws://localhost:9090";

// Create the context
const RosContext = createContext<ROSLIB.Ros | null>(null);

// Provider component
export function RosProvider({ children }: { children: React.ReactNode }) {
  const [ros] = useState<ROSLIB.Ros>(() => new ROSLIB.Ros({}));
  const [isConnected, setIsConnected] = useState<boolean>(false);
  const [shouldReconnect, setShouldReconnect] = useState<boolean>(true);

  useEffect(() => {
    function connect() {
      ros.connect(rosBridgeUrl);
    }
    function onConnection() {
      console.log("Connected to", rosBridgeUrl);
      setIsConnected(true);
    }
    function onError(error: any) {
      console.error("ROS error:", error);
      setIsConnected(false);
      if (shouldReconnect) setTimeout(connect, 3000);
    }
    function onClose() {
      console.log("ROS closed.");
    }

    ros.on("connection", onConnection);
    ros.on("error", onError);
    ros.on("close", onClose);
    connect();

    return () => {
      setShouldReconnect(false);
      ros.off("connection", onConnection);
      ros.off("error", onError);
      ros.off("close", onClose);
      if (isConnected) ros.close();
    };
  }, [isConnected, ros, shouldReconnect]);

  return <RosContext.Provider value={ros}>{children}</RosContext.Provider>;
}

// Hook to get the Ros instance
export function useRos() {
  const context = useContext(RosContext);
  if (!context) {
    throw new Error("useRos must be used within a RosProvider");
  }
  return context;
}
