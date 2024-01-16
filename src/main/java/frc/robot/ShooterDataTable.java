package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.BarycentricInterpolation;

public class ShooterDataTable {
  private BarycentricInterpolation interpolatorAngle;
  private BarycentricInterpolation interpolatorSpeedL;
  private BarycentricInterpolation interpolatorSpeedR;
  private BarycentricInterpolation interpolatorOffset;

  public ShooterDataTable(Translation2d[] points, ShooterSpec[] specs) {
    double[] x = new double[points.length];
    double[] y = new double[points.length];
    double[] angle = new double[points.length];
    double[] speedL = new double[points.length];
    double[] speedR = new double[points.length];
    double[] offset = new double[points.length];
    for (int i = 0; i < points.length; i++) {
      x[i] = points[i].getX();
      y[i] = points[i].getY();
      angle[i] = specs[i].angle();
      speedL[i] = specs[i].speedL();
      speedR[i] = specs[i].speedR();
      offset[i] = specs[i].offset();
    }
    try {
      interpolatorAngle = new BarycentricInterpolation(x, y, angle);
      interpolatorSpeedL = new BarycentricInterpolation(x, y, speedL);
      interpolatorSpeedR = new BarycentricInterpolation(x, y, speedR);
      interpolatorOffset = new BarycentricInterpolation(x, y, offset);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  // testing
  public static void main(String... args) {
    Translation2d[] points =
        new Translation2d[] {
          new Translation2d(0, 0),
          new Translation2d(0, 1),
          new Translation2d(1, 0),
          new Translation2d(1, 1)
        };
    ShooterSpec[] specs =
        new ShooterSpec[] {
          new ShooterSpec(0, 0, 0, 0),
          new ShooterSpec(1, 1, 1, 1),
          new ShooterSpec(2, 2, 2, 2),
          new ShooterSpec(2, 2, 2, 2)
        };
    ShooterDataTable table = new ShooterDataTable(points, specs);
    System.out.println(table.get(new Translation2d(0.5, 0.5)));
  }

  public ShooterSpec get(Translation2d toTarget) {
    return new ShooterSpec(
        interpolatorAngle.interpolate(toTarget.getX(), toTarget.getY()),
        interpolatorSpeedL.interpolate(toTarget.getX(), toTarget.getY()),
        interpolatorSpeedR.interpolate(toTarget.getX(), toTarget.getY()),
        interpolatorOffset.interpolate(toTarget.getX(), toTarget.getY()));
  }
}
