package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.inputs.NoteDetector;
import frc.robot.inputs.PoseEstimator;
import frc.robot.lib.controllers.Thrustmaster;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import java.io.File;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

public class RobotContainer {

  private static final Thrustmaster leftThrustmaster = new Thrustmaster(0);
  private static final Thrustmaster rightThrustmaster = new Thrustmaster(1);

  static {
    SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
  }

  private final Swerve drivebase;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Angle conversion factor = 360 / (GEAR RATIO * ENCODER RESOLUTION)
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, 4096);
    // with real values
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO *
    // ENCODER RESOLUTION).
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(0, 6.75, 1);
    SwerveDrive swerveDrive;
    try {
      Measure<Velocity<Distance>> maximumSpeed = Units.FeetPerSecond.of(20);
      swerveDrive =
          new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
              .createSwerveDrive(
                  maximumSpeed.in(Units.MetersPerSecond),
                  angleConversionFactor,
                  driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // TODO: Ensure to get the actual points
    ShooterDataTable shooterDataTable;
    shooterDataTable = new ShooterDataTable(null, null);

    PoseEstimator poseEstimator;
    Intake intake;
    Shooter shooter;
    Indexer indexer;
    NoteDetector noteDetector;
    try {
      poseEstimator =
          new PoseEstimator(
              new PhotonCamera("cam"),
              swerveDrive::addVisionMeasurement,
              swerveDrive.swerveDrivePoseEstimator::getEstimatedPosition,
              swerveDrive::getModulePositions,
              swerveDrive.swerveDrivePoseEstimator::update);
      // TODO: Remember to replace with the actual camera name
      PhotonCamera photonCamera = new PhotonCamera("photonvision");
      noteDetector = new NoteDetector(photonCamera, poseEstimator);
      intake = new Intake();
      shooter = new Shooter(shooterDataTable, poseEstimator);
      indexer = new Indexer(shooterDataTable, poseEstimator);

    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    drivebase =
        new Swerve(
            swerveDrive,
            new ShooterDataTable(new Translation2d[] {}, new ShooterSpec[] {}),
            poseEstimator);

    Superstructure superstructure =
        new Superstructure(intake, indexer, shooter, drivebase, noteDetector, poseEstimator);

    Command driveFieldOrientedDirectAngle =
        drivebase.driveCommand(
            () -> -leftThrustmaster.getY(),
            () -> -leftThrustmaster.getX(),
            () -> -rightThrustmaster.getX());
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    configureBindings();

    autoChooser.addOption("Top with amp", superstructure.fromTopWithAmp());
    autoChooser.addOption("Top with no amp", superstructure.fromTopWithoutAmp());
    autoChooser.addOption("Bottom with no amp", superstructure.fromBottomWithoutAmp());
    autoChooser.addOption("Middle with no amp", superstructure.fromStartingMiddleWithoutAmp());
  }

  private void configureBindings() {
    rightThrustmaster.getButton(Thrustmaster.Button.TRIGGER).onTrue(drivebase.resetHeading());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
