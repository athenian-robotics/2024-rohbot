package frc.robot;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.inputs.NoteDetector;
import frc.robot.inputs.poseEstimator.PoseEstimator;
import frc.robot.lib.controllers.Thrustmaster;

import java.io.IOException;

import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.indexer.IndexerIOPhysical;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.powerBudget.PowerBudgetPhysical;
import frc.robot.subsystems.shooter.ShooterIOPhysical;
import org.photonvision.PhotonCamera;

public class RobotContainer {

  private static final Thrustmaster leftThrustmaster = new Thrustmaster(0);
  private static final Thrustmaster rightThrustmaster = new Thrustmaster(1);
  public static boolean INTAKE_SENSOR_TRIGGERED;


    private final Swerve drivebase;
  private final Superstructure superstructure;
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    //     TODO: Ensure to get the actual points
    ShooterDataTable shooterDataTable;
    Translation2d[] dummyPoints =
        new Translation2d[] {new Translation2d(), new Translation2d(), new Translation2d()};
    ShooterSpec[] dummySpecs =
        new ShooterSpec[] {
          new ShooterSpec(
              Units.Degrees.of(0),
              Units.DegreesPerSecond.of(0),
              Units.DegreesPerSecond.of(0),
              Units.Degrees.of(0)),
          new ShooterSpec(
              Units.Degrees.of(0),
              Units.DegreesPerSecond.of(0),
              Units.DegreesPerSecond.of(0),
              Units.Degrees.of(0)),
          new ShooterSpec(
              Units.Degrees.of(0),
              Units.DegreesPerSecond.of(0),
              Units.DegreesPerSecond.of(0),
              Units.Degrees.of(0))
        };
    shooterDataTable = new ShooterDataTable(dummyPoints, dummySpecs, false);

    PoseEstimator poseEstimator;
    IntakeIOSparkMax intake;
    ShooterIOPhysical shooter;
    IndexerIOPhysical indexer;
    NoteDetector noteDetector;
    try {
      poseEstimator =
          new PoseEstimator(
              new PhotonCamera("cam"));
      // TODO: Remember to replace with the actual camera name
      PhotonCamera photonCamera = new PhotonCamera("photonvision");
      noteDetector = new NoteDetector(photonCamera, poseEstimator);
      intake = new IntakeIOSparkMax();
      TimeOfFlight sensor = new TimeOfFlight(15);
      sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 0.02);

      PowerBudgetPhysical power = new PowerBudgetPhysical();
      shooter = new ShooterIOPhysical(shooterDataTable, poseEstimator, sensor, power);
      indexer = new IndexerIOPhysical(shooterDataTable, poseEstimator, sensor, power);

    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    drivebase = new Swerve(shooterDataTable, poseEstimator); //TODO: add shit

    superstructure =
        new Superstructure(
            intake, indexer, shooter, drivebase, noteDetector, poseEstimator, shooterDataTable);

    Command driveFieldOrientedDirectAngle =
        drivebase.driveCommand(
            () -> -leftThrustmaster.getY(),
            () -> -leftThrustmaster.getX(),
            () -> -rightThrustmaster.getX());
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    configureBindings();

    //    autoChooser.addOption("Top with amp", superstructure.fromTopWithAmp());
    //    autoChooser.addOption("Top with no amp", superstructure.fromTopWithoutAmp());
    //    autoChooser.addOption("Bottom with no amp", superstructure.fromBottomWithoutAmp());
    //    autoChooser.addOption("Middle with no amp",
    // superstructure.fromStartingMiddleWithoutAmp());
  }

  private void configureBindings() {
    rightThrustmaster.getButton(Thrustmaster.Button.TRIGGER).onTrue(drivebase.resetHeading());
    leftThrustmaster.getButton(Thrustmaster.Button.TRIGGER).onTrue(superstructure.shoot());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
