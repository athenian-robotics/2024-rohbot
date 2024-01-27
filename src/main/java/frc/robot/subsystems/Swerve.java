package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
  private final double maximumSpeed = Units.feetToMeters(20);
  // speed in m/s
  private SwerveDrive swerveDrive;

  public Swerve(File swerveJsonDirectory) {
    // Angle conversion factor = 360 / (GEAR RATIO * ENCODER RESOLUTION)
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, 4096);
    // with real values
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO *
    // ENCODER RESOLUTION).
    double driveConversionFactor =
        SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(0), 6.75, 1);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive =
          new SwerveParser(swerveJsonDirectory)
              .createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    swerveDrive.setHeadingCorrection(false);
    return run(
        () -> {
          double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth control
          double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth control
          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  xInput,
                  yInput,
                  headingX.getAsDouble(),
                  headingY.getAsDouble(),
                  swerveDrive.getYaw().getRadians(),
                  swerveDrive.getMaximumVelocity()));
        });
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  // TODO: Implement a command to face speaker when shooting
}
