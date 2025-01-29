import { useState, useEffect } from "react";

interface Location {
  _id?: string;
  name: string;
  x: number;
  y: number;
}

interface UseLocationsReturn {
  locations: Location[];
  loading: boolean;
  error: string | null;
  createLocation: (location: Location) => Promise<void>;
  deleteLocation: (id: string) => Promise<void>;
}

export function useLocations(): UseLocationsReturn {
  const [locations, setLocations] = useState<Location[]>([]);
  const [loading, setLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const fetchLocations = async () => {
      setLoading(true);
      setError(null);
      try {
        const response = await fetch("/api/locations");
        const data = await response.json();
        if (response.ok) {
          setLocations(data.data);
        } else {
          setError(data.error);
        }
      } catch (err) {
        setError(
          err instanceof Error ? err.message : "Failed to fetch locations"
        );
      } finally {
        setLoading(false);
      }
    };

    fetchLocations();
  }, []);

  const createLocation = async (location: Location) => {
    setLoading(true);
    setError(null);
    try {
      const response = await fetch("/api/locations", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(location),
      });
      const data = await response.json();
      if (response.ok) {
        setLocations((prevLocations) => [...prevLocations, data.data]);
      } else {
        setError(data.error);
      }
    } catch (err) {
      setError(
        err instanceof Error ? err.message : "Failed to create location"
      );
    } finally {
      setLoading(false);
    }
  };

  const deleteLocation = async (id: string) => {
    setLoading(true);
    setError(null);
    try {
      const response = await fetch(`/api/locations?id=${id}`, {
        method: "DELETE",
      });
      const data = await response.json();
      if (response.ok) {
        setLocations((prevLocations) =>
          prevLocations.filter((location) => location._id !== id)
        );
      } else {
        setError(data.error);
      }
    } catch (err) {
      setError(
        err instanceof Error ? err.message : "Failed to delete location"
      );
    } finally {
      setLoading(false);
    }
  };

  return { locations, loading, error, createLocation, deleteLocation };
}
