package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import frc.robot.lib.BarycentricInterpolation;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class ShooterDataTable {
  private final BarycentricInterpolation interpolatorAngle;
  private final BarycentricInterpolation interpolatorSpeedL;
  private final BarycentricInterpolation interpolatorSpeedR;
  private final BarycentricInterpolation interpolatorOffset;

  public ShooterDataTable(Translation2d[] points, ShooterSpec[] specs, Boolean verboseLogging) {
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
      interpolatorAngle = new BarycentricInterpolation(x, y, angle, verboseLogging);
      interpolatorSpeedL = new BarycentricInterpolation(x, y, speedL, verboseLogging);
      interpolatorSpeedR = new BarycentricInterpolation(x, y, speedR, verboseLogging);
      interpolatorOffset = new BarycentricInterpolation(x, y, offset, verboseLogging);
    } catch (Exception e) {
      throw new RuntimeException(e);
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
          new ShooterSpec(Degrees.of(0), RPM.of(0), RPM.of(0), Degrees.of(0)),
          new ShooterSpec(Degrees.of(1), RPM.of(1), RPM.of(1), Degrees.of(1)),
          new ShooterSpec(Degrees.of(2), RPM.of(2), RPM.of(1), Degrees.of(2)),
          new ShooterSpec(Degrees.of(2), RPM.of(2), RPM.of(2), Degrees.of(2))
        };
    ShooterDataTable table = new ShooterDataTable(points, specs, false);
    System.out.println(table.get(new Translation2d(0.5, 0.5)).get());
  }

  public Optional<ShooterSpec> get(Translation2d toTarget) {
    return Stream.of(
            interpolatorAngle.interpolate(toTarget.getX(), toTarget.getY()).map(Degrees::of),
            interpolatorSpeedL.interpolate(toTarget.getX(), toTarget.getY()).map(RPM::of),
            interpolatorSpeedR.interpolate(toTarget.getX(), toTarget.getY()).map(RPM::of),
            interpolatorOffset.interpolate(toTarget.getX(), toTarget.getY()).map(Degrees::of))
        .collect(
            Collectors.collectingAndThen(
                Collectors.toList(),
                list ->
                    list.stream().allMatch(Optional::isPresent)
                        ? Optional.of(
                            new ShooterSpec(
                                (Measure<Angle>) list.get(0).get(),
                                (Measure<Velocity<Angle>>) list.get(1).get(),
                                (Measure<Velocity<Angle>>) list.get(2).get(),
                                (Measure<Angle>) list.get(3).get()))
                        : Optional.empty()));
  }
}
