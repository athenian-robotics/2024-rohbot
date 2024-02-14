package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Translation2d;
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

public class RobotContainer {
  private final Swerve drivebase =
      new Swerve(
          new File(Filesystem.getDeployDirectory(), "swerve"),
          new ShooterDataTable(new Translation2d[] {}, new ShooterSpec[] {}));

  private static final Thrustmaster leftThrustmaster = new Thrustmaster(0);
  private static final Thrustmaster rightThrustmaster = new Thrustmaster(1);
  private final PhotonCamera photonCamera =
      new PhotonCamera("photonvision"); // Remember to replace with the actual
  // camera name

  private final ShooterDataTable shooterDataTable =
      new ShooterDataTable(null, null); // Ensure to get the actual points
  // and specs from Rohan
  private final Intake intake = new Intake();
  private final Shooter shooter;
  private final Indexer indexer;
  private PoseEstimator poseEstimator;
  private final NoteDetector noteDetector;
  private final Superstructure superstructure;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    try {
      poseEstimator = new PoseEstimator(photonCamera, drivebase.getSwerveDrive());
    } catch (IOException e) {
      e.printStackTrace();
    }
    noteDetector = new NoteDetector(photonCamera, poseEstimator);
    shooter = new Shooter(shooterDataTable, poseEstimator);
    indexer = new Indexer(shooterDataTable, poseEstimator);
    superstructure =
        new Superstructure(intake, indexer, shooter, drivebase, noteDetector, poseEstimator);

    Command driveFieldOrientedDirectAngle =
        drivebase.driveCommand(
            () -> leftThrustmaster.getY(),
            () -> leftThrustmaster.getX(),
            () -> rightThrustmaster.getX());
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    configureBindings();

    autoChooser.addOption("Top with amp", superstructure.fromTopWithAmp());
    autoChooser.addOption("Top with no amp", superstructure.fromTopWithoutAmp());
    autoChooser.addOption("Bottom with no amp", superstructure.fromBottomWithoutAmp());
    autoChooser.addOption("Middle with no amp", superstructure.fromStartingMiddleWithoutAmp());
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
