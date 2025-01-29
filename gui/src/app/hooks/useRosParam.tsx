"use client";

import { useState, useCallback, useMemo } from "react";
import ROSLIB from "roslib";
import { useRos } from "@/components/RosProvider";

interface UseRosParamReturn<T> {
  value: T | null;
  setValue: (newValue: T) => Promise<void>;
  getValue: () => Promise<void>;
  isLoading: boolean;
  error: string | null;
}

export function useRosParam<T>(paramName: string): UseRosParamReturn<T> {
  const ros = useRos();
  const [value, setValue] = useState<T | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  const param = useMemo(
    () =>
      new ROSLIB.Param({
        ros: ros,
        name: paramName,
      }),
    [paramName, ros]
  );

  const getValue = useCallback(async () => {
    setIsLoading(true);
    setError(null);

    try {
      const result = await new Promise<T>((resolve, reject) => {
        param.get((value) => {
          resolve(value as T);
        });
      });
      setValue(result);
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to get parameter");
    } finally {
      setIsLoading(false);
    }
  }, [param]);

  const setParamValue = useCallback(
    async (newValue: T) => {
      setIsLoading(true);
      setError(null);

      try {
        await new Promise<void>((resolve, reject) => {
          param.set(newValue, () => {
            setValue(newValue);
            resolve();
          });
        });
      } catch (err) {
        setError(
          err instanceof Error ? err.message : "Failed to set parameter"
        );
      } finally {
        setIsLoading(false);
      }
    },
    [param]
  );

  return {
    value,
    setValue: setParamValue,
    getValue,
    isLoading,
    error,
  };
}
