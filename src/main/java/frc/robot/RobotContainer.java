package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.inputs.PoseEstimator;
import frc.robot.lib.controllers.Thrustmaster;
import frc.robot.subsystems.Swerve;
import io.github.jdiemke.triangulation.NotEnoughPointsException;
import java.io.File;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class RobotContainer {
  private final Measure<Velocity<Distance>> maximumSpeed = Units.FeetPerSecond.of(20);

  private final Swerve drivebase;
  private final PoseEstimator poseEstimator;

  private final double LEFT_X_DEADBAND = 0.001;
  private final double LEFT_Y_DEADBAND = 0.001;
  private static final Thrustmaster leftThrustmaster = new Thrustmaster(0);
  private static final Thrustmaster rightThrustmaster = new Thrustmaster(1);
  private final SendableChooser<Command> autoChooser;
  private final SwerveDrive swerveDrive;

  static {
    SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
  }

  public RobotContainer() {
    // Angle conversion factor = 360 / (GEAR RATIO * ENCODER RESOLUTION)
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, 4096);
    // with real values
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO *
    // ENCODER RESOLUTION).
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(0, 6.75, 1);
    try {
      swerveDrive =
          new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
              .createSwerveDrive(
                  maximumSpeed.in(Units.MetersPerSecond),
                  angleConversionFactor,
                  driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    try {
      poseEstimator =
          new PoseEstimator(
              new PhotonCamera("cam"),
              swerveDrive::addVisionMeasurement,
              swerveDrive.swerveDrivePoseEstimator::getEstimatedPosition,
              swerveDrive::getModulePositions,
              swerveDrive.swerveDrivePoseEstimator::update);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    try {
      drivebase =
          new Swerve(
              swerveDrive,
              new ShooterDataTable(new Translation2d[] {}, new ShooterSpec[] {}),
              poseEstimator);
    } catch (NotEnoughPointsException e) {
      throw new RuntimeException(e); // GG go next
    }

    Command driveFieldOrientedDirectAngle =
        drivebase.driveCommand(
            () -> -leftThrustmaster.getY(),
            () -> -leftThrustmaster.getX(),
            () -> -rightThrustmaster.getX());
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
  }

  private void configureBindings() {
    rightThrustmaster.getButton(Thrustmaster.Button.TRIGGER).onTrue(drivebase.resetHeading());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
