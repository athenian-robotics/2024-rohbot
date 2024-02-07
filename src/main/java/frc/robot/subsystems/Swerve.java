package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    AutoBuilder.configureHolonomic(
        () -> swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition(),
        pose -> swerveDrive.resetOdometry(pose),
        () -> swerveDrive.getRobotVelocity(),
        swerveDrive::drive,
        new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // TODO: Max module speed, in m/s
            0.4, // TODO: Drive base radius in meters. Distance from robot center to furthest
            // module.
            new ReplanningConfig() // TODO: Default path replanning config. See the API for the
            // options here
            ),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angSped) {
    swerveDrive.setHeadingCorrection(false);
    return run(
        () -> {
          double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth control
          double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth control
          swerveDrive.drive(
              new Translation2d(xInput * 20, yInput * 20),
              angSped.getAsDouble() * 20 / 1.07,
              true,
              false);
        });
  }

  public Command resetHeading() {
    return new InstantCommand(() -> swerveDrive.zeroGyro(), this);
  }
  // TODO: Implement a command to face speaker when shooting
}
