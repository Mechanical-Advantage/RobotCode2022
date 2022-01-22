package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

public class CircleFitter {
  private CircleFitter() {}

  public static Translation2d fit(double radius, List<Translation2d> points,
      double precision) {

    // Find starting point
    double xSum = 0.0;
    double ySum = 0.0;
    for (Translation2d point : points) {
      xSum += point.getX();
      ySum += point.getY();
    }
    Translation2d center =
        new Translation2d(xSum / points.size() + radius, ySum / points.size());

    // Iterate to find optimal center
    double shiftDist = radius / 2.0;
    double minResidual = calcResidual(radius, points, center);
    while (true) {
      List<Translation2d> translations = List.of(
          new Translation2d(shiftDist, 0.0), new Translation2d(-shiftDist, 0.0),
          new Translation2d(0.0, shiftDist),
          new Translation2d(0.0, -shiftDist));
      Translation2d bestPoint = center;
      boolean centerIsBest = true;

      // Check all adjacent positions
      for (Translation2d translation : translations) {
        double residual =
            calcResidual(radius, points, center.plus(translation));
        if (residual < minResidual) {
          bestPoint = center.plus(translation);
          minResidual = residual;
          centerIsBest = false;
          break;
        }
      }

      // Decrease shift, exit, or continue
      if (centerIsBest) {
        shiftDist /= 2.0;
        if (shiftDist < precision) {
          return center;
        }
      } else {
        center = bestPoint;
      }
    }
  }

  private static double calcResidual(double radius, List<Translation2d> points,
      Translation2d center) {
    double residual = 0.0;
    for (Translation2d point : points) {
      double diff = point.getDistance(center) - radius;
      residual += diff * diff;
    }
    return residual;
  }
}
