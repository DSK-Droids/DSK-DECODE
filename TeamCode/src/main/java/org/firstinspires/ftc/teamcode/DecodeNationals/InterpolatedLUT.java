package org.firstinspires.ftc.teamcode.DecodeNationals;

import java.util.Map;
import java.util.TreeMap;

/**
 * Interpolated Lookup Table (LUT) for shooter parameters.
 * Stores distance -> value pairs and interpolates between them for smooth transitions.
 *
 * Usage:
 * 1. Add data points from testing: lut.addDataPoint(distance, value)
 * 2. Get interpolated value: lut.get(currentDistance)
 */
public class InterpolatedLUT {

    // TreeMap automatically sorts by key (distance), making interpolation easier
    private final TreeMap<Double, Double> dataPoints = new TreeMap<>();

    /**
     * Create an empty LUT
     */
    public InterpolatedLUT() {}

    /**
     * Create a LUT with initial data points
     * @param points Array of {distance, value} pairs
     */
    public InterpolatedLUT(double[][] points) {
        for (double[] point : points) {
            if (point.length >= 2) {
                addDataPoint(point[0], point[1]);
            }
        }
    }

    /**
     * Add a data point to the LUT
     * @param distance The distance (input)
     * @param value The value at that distance (output)
     */
    public void addDataPoint(double distance, double value) {
        dataPoints.put(distance, value);
    }

    /**
     * Remove a data point from the LUT
     * @param distance The distance to remove
     */
    public void removeDataPoint(double distance) {
        dataPoints.remove(distance);
    }

    /**
     * Clear all data points
     */
    public void clear() {
        dataPoints.clear();
    }

    /**
     * Get the interpolated value for a given distance.
     * - If distance is below all data points, returns the lowest value
     * - If distance is above all data points, returns the highest value
     * - Otherwise, linearly interpolates between the two nearest points
     *
     * @param distance The distance to look up
     * @return The interpolated value
     */
    public double get(double distance) {
        if (dataPoints.isEmpty()) {
            return 0.0;
        }

        // If only one data point, return it
        if (dataPoints.size() == 1) {
            return dataPoints.firstEntry().getValue();
        }

        // Get the entries just below and above the target distance
        Map.Entry<Double, Double> lower = dataPoints.floorEntry(distance);
        Map.Entry<Double, Double> upper = dataPoints.ceilingEntry(distance);

        // Handle edge cases
        if (lower == null) {
            // Distance is below all data points - return lowest
            return upper.getValue();
        }
        if (upper == null) {
            // Distance is above all data points - return highest
            return lower.getValue();
        }

        // If we hit an exact match
        if (lower.getKey().equals(upper.getKey())) {
            return lower.getValue();
        }

        // Linear interpolation
        double lowerDist = lower.getKey();
        double upperDist = upper.getKey();
        double lowerVal = lower.getValue();
        double upperVal = upper.getValue();

        double t = (distance - lowerDist) / (upperDist - lowerDist);
        return lowerVal + t * (upperVal - lowerVal);
    }

    /**
     * Get the number of data points in the LUT
     */
    public int size() {
        return dataPoints.size();
    }

    /**
     * Check if the LUT has any data points
     */
    public boolean isEmpty() {
        return dataPoints.isEmpty();
    }

    /**
     * Get the minimum distance in the LUT
     */
    public double getMinDistance() {
        return dataPoints.isEmpty() ? 0.0 : dataPoints.firstKey();
    }

    /**
     * Get the maximum distance in the LUT
     */
    public double getMaxDistance() {
        return dataPoints.isEmpty() ? 0.0 : dataPoints.lastKey();
    }

    /**
     * Get all data points as a TreeMap (for debugging/display)
     */
    public TreeMap<Double, Double> getDataPoints() {
        return new TreeMap<>(dataPoints);
    }
}

