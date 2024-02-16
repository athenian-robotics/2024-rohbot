package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.BarycentricInterpolation;
import io.github.jdiemke.triangulation.NotEnoughPointsException;

public class ShooterDataTable {
  private BarycentricInterpolation interpolatorAngle;
  private BarycentricInterpolation interpolatorSpeedL;
  private BarycentricInterpolation interpolatorSpeedR;
  private BarycentricInterpolation interpolatorOffset;

  public ShooterDataTable(Translation2d[] points, ShooterSpec[] specs)
      throws NotEnoughPointsException {
    double[] x = new double[points.length];
    double[] y = new double[points.length];
    double[] angle = new double[points.length];
    double[] speedL = new double[points.length];
    double[] speedR = new double[points.length];
    double[] offset = new double[points.length];
    for (int i = 0; i < points.length; i++) {
      x[i] = points[i].getX();
      y[i] = points[i].getY();
      angle[i] = specs[i].angle().in(Degrees);
      speedL[i] = specs[i].speedL().in(RPM);
      speedR[i] = specs[i].speedR().in(RPM);
      offset[i] = specs[i].offset().in(Degrees);
    }
    try {
      interpolatorAngle = new BarycentricInterpolation(x, y, angle);
      interpolatorSpeedL = new BarycentricInterpolation(x, y, speedL);
      interpolatorSpeedR = new BarycentricInterpolation(x, y, speedR);
      interpolatorOffset = new BarycentricInterpolation(x, y, offset);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  // testing
  public static void main(String... args) throws NotEnoughPointsException {
    Translation2d[] points =
        new Translation2d[] {
          new Translation2d(0, 0),
          new Translation2d(0, 1),
          new Translation2d(1, 0),
          new Translation2d(1, 1)
        };
    ShooterSpec[] specs =
        new ShooterSpec[] {
          new ShooterSpec(Degrees.of(0), RPM.of(0), RPM.of(0), Degrees.of(0)),
          new ShooterSpec(Degrees.of(1), RPM.of(1), RPM.of(1), Degrees.of(1)),
          new ShooterSpec(Degrees.of(2), RPM.of(2), RPM.of(1), Degrees.of(2)),
          new ShooterSpec(Degrees.of(2), RPM.of(2), RPM.of(2), Degrees.of(2))
        };
    ShooterDataTable table = new ShooterDataTable(points, specs);
    System.out.println(table.get(new Translation2d(0.5, 0.5)));
  }

  public ShooterSpec get(Translation2d toTarget) {
    return new ShooterSpec(
        Degrees.of(interpolatorAngle.interpolate(toTarget.getX(), toTarget.getY())),
        RPM.of(interpolatorSpeedL.interpolate(toTarget.getX(), toTarget.getY())),
        RPM.of(interpolatorSpeedR.interpolate(toTarget.getX(), toTarget.getY())),
        Degrees.of(interpolatorOffset.interpolate(toTarget.getX(), toTarget.getY())));
  }
}
