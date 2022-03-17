// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

/** Runs a linear interpolation between a set of points. */
public class LinearInterpolation {
  private static final Comparator<Point> sortByX = (Point p1,
      Point p2) -> Double.valueOf(p1.x).compareTo(Double.valueOf(p2.x));

  private final List<Point> points;

  public LinearInterpolation(List<Point> points) {
    this.points = points.stream().sorted(sortByX).collect(Collectors.toList());
  }

  /** Returns the predicted value of y at the provided x coordinate, interpolating if necessary. */
  public double predict(double x) {
    if (points.size() == 0) {
      return 0.0;
    }
    if (points.size() == 1) {
      return points.get(0).y;
    }
    if (x <= points.get(0).x) {
      return points.get(0).y;
    }
    if (x >= points.get(points.size() - 1).x) {
      return points.get(points.size() - 1).y;
    }
    for (int i = 0; i < points.size(); i++) {
      if (x < points.get(i).x) {
        return points.get(i).interpolate(points.get(i - 1), x);
      }
    }
    return 0.0;
  }

  public static class Point {
    public final double x;
    public final double y;

    public Point(double x, double y) {
      this.x = x;
      this.y = y;
    }

    private double interpolate(Point otherPoint, double newX) {
      double dydx = (otherPoint.y - y) / (otherPoint.x - x);
      return (dydx * (newX - x)) + y;
    }
  }
}
