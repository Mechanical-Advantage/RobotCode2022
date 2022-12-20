// Copyright (c) 2022 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.TreeMap;

/** Stores a history of timestamped Pose2d objects. */
public class PoseHistory {
  // Inner class handling interpolation of double values
  private static class InterpolatingTimestamp implements Comparable<InterpolatingTimestamp> {
    public double value;

    public InterpolatingTimestamp(double value) {
      this.value = value;
    }

    @SuppressWarnings("unused")
    public InterpolatingTimestamp interpolate(InterpolatingTimestamp other, double x) {
      double dydx = other.value - value;
      double searchY = dydx * x + value;
      return new InterpolatingTimestamp(searchY);
    }

    public double inverseInterpolate(InterpolatingTimestamp upper, InterpolatingTimestamp query) {
      double upper_to_lower = upper.value - value;
      if (upper_to_lower <= 0) {
        return 0;
      }
      double query_to_lower = query.value - value;
      if (query_to_lower <= 0) {
        return 0;
      }
      return query_to_lower / upper_to_lower;
    }

    @Override
    public int compareTo(InterpolatingTimestamp other) {
      return Double.compare(value, other.value);
    }

    @Override
    public boolean equals(Object o) {
      if (this == o) return true;
      if (o == null || getClass() != o.getClass()) return false;
      InterpolatingTimestamp that = (InterpolatingTimestamp) o;
      return Double.compare(that.value, value) == 0;
    }

    @Override
    public int hashCode() {
      return Objects.hash(value);
    }
  }

  public static class TimestampedPose2d {
    private final InterpolatingTimestamp timestamp;
    private final Pose2d pose;

    private TimestampedPose2d(InterpolatingTimestamp timestamp, Pose2d pose) {
      this.timestamp = timestamp;
      this.pose = pose;
    }

    /**
     * @return The timestamp that the pose was recorded, in seconds
     */
    public double getTimestamp() {
      return timestamp.value;
    }

    /**
     * @return The pose associated with the timestamp
     */
    public Pose2d getPose() {
      return pose;
    }
  }

  private final int capacity;
  private final TreeMap<InterpolatingTimestamp, Pose2d> map = new TreeMap<>();

  /**
   * Creates a new PoseHistory with the given capacity. When the history is at capacity, the oldest
   * poses are removed as new ones are inserted.
   *
   * @param capacity The capacity of the history
   */
  public PoseHistory(int capacity) {
    this.capacity = capacity;
  }

  /**
   * Creates a new PoseHistory with infinite capacity. This is usually not a good idea in practice,
   * because it can lead to high memory usage potentially without the user knowing
   */
  public PoseHistory() {
    this(0);
  }

  /** Resets the pose history, deleting all entries. */
  public void reset() {
    map.clear();
  }

  /**
   * Inserts a new timestamped pose into the history.
   *
   * @param timestamp The timestamp, in seconds
   * @param pose The pose
   */
  public void insert(double timestamp, Pose2d pose) {
    while (capacity > 0 && map.size() >= capacity) {
      // Remove elements since the tree is oversize
      map.remove(map.firstKey());
    }

    map.put(new InterpolatingTimestamp(timestamp), pose);
  }

  /**
   * Gets the latest timestamp and pose from the history.
   *
   * @return An object containing the timestamp and the pose
   */
  public Optional<TimestampedPose2d> getLatest() {
    Map.Entry<InterpolatingTimestamp, Pose2d> entry = map.lastEntry();
    if (entry == null) {
      return Optional.empty();
    }
    return Optional.of(new TimestampedPose2d(entry.getKey(), entry.getValue()));
  }

  /**
   * Retrieves a pose at the given timestamp. If no pose is available at the requested timestamp,
   * interpolation is performed between the two timestamps nearest to the one requested.
   *
   * @param timestamp The timestamp to obtain a pose at
   * @return An Optional object which potentially contains the located pose, or is empty if no pose
   *     could be computed.
   */
  public Optional<Pose2d> get(double timestamp) {
    InterpolatingTimestamp key = new InterpolatingTimestamp(timestamp);
    Pose2d retrieved = map.get(key);

    if (retrieved != null)
      return Optional.of(retrieved); // We have a pose at the exact timestamp, return it

    InterpolatingTimestamp topBound = map.ceilingKey(key);
    InterpolatingTimestamp bottomBound = map.floorKey(key);

    // If attempting interpolation at ends of tree, return the nearest data point
    if (topBound == null && bottomBound == null) {
      return Optional.empty();
    } else if (topBound == null) {
      return Optional.of(map.get(bottomBound));
    } else if (bottomBound == null) {
      return Optional.of(map.get(topBound));
    }

    // Get surrounding values for interpolation
    Pose2d topElem = map.get(topBound);
    Pose2d bottomElem = map.get(bottomBound);
    return Optional.of(
        GeomUtil.interpolate(bottomElem, topElem, bottomBound.inverseInterpolate(topBound, key)));
  }
}
