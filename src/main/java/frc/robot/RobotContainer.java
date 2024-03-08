package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.controllers.Thrustmaster;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOPhysical;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.powerBudget.PowerBudget;
import frc.robot.subsystems.powerBudget.PowerBudgetPhysical;
import frc.robot.subsystems.powerBudget.PowerBudgetSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOPhysical;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionIOPhoton;
import frc.robot.vision.VisionIOSim;

public class RobotContainer {

  private static final Thrustmaster leftThrustmaster = new Thrustmaster(0);
  private static final Thrustmaster rightThrustmaster = new Thrustmaster(1);

  private static final Thrustmaster opThrustmaster = new Thrustmaster(2);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private Intake intake = null;
  private Shooter shooter = null;
  private Indexer indexer = null;
  private final Drive drivebase;
  private final Superstructure superstructure;

  public RobotContainer() {
    switch (Constants.currentMode) {
      default -> { //     TODO: Ensure to get the actual points
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
        drivebase =
            new Drive(
                new GyroIOPigeon2(false),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3),
                shooterDataTable,
                new Vision(new VisionIOPhoton())); // TODO: add shit

        // TODO: Remember to replace with the actual camera name
        // noteDetector = new NoteDetector(photonCamera, poseEstimator);
        intake = new Intake(new IntakeIOSparkMax());

        PowerBudgetPhysical power = new PowerBudgetPhysical();
        shooter = new Shooter(new ShooterIOPhysical(shooterDataTable, drivebase, power));
        indexer = new Indexer(new IndexerIOPhysical(shooterDataTable, drivebase, power));

        superstructure = new Superstructure(intake, indexer, shooter, drivebase, shooterDataTable);
        drivebase.setDefaultCommand(
            drivebase.joystickDrive(
                rightThrustmaster::getY,
                () -> -rightThrustmaster.getX(),
                () -> leftThrustmaster.getX()));
        configureBindings();

        //    autoChooser.addOption("Top with amp", superstructure.fromTopWithAmp());
        //    autoChooser.addOption("Top with no amp", superstructure.fromTopWithoutAmp());
        //    autoChooser.addOption("Bottom with no amp", superstructure.fromBottomWithoutAmp());
        //    autoChooser.addOption("Middle with no amp",
        // superstructure.fromStartingMiddleWithoutAmp());}
      }
      case SIM -> {
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

        drivebase =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                shooterDataTable,
                new Vision(new VisionIOSim()));

        PowerBudget powerBudget = new PowerBudget(new PowerBudgetSim());
        Intake intake = new Intake(new IntakeSim(powerBudget));
        Indexer indexer = new Indexer(new IndexerIOSim(shooterDataTable, drivebase, powerBudget));
        Shooter shooter = new Shooter(new ShooterIOSim(shooterDataTable, drivebase, powerBudget));
        superstructure = new Superstructure(intake, indexer, shooter, drivebase, shooterDataTable);
      }
    }
  }

  private void configureBindings() {
    rightThrustmaster
        .getButton(Thrustmaster.Button.BOTTOM)
        .onTrue(
            runOnce(
                () ->
                    drivebase.setPose(
                        new Pose2d(drivebase.getPose().getTranslation(), new Rotation2d()))));
    leftThrustmaster
            .getButton(Thrustmaster.Button.TRIGGER)
            .onTrue(superstructure.amp());


//    leftThrustmaster.getButton(Thrustmaster.Button.LEFT).onTrue(shooter.sysId());
//    rightThrustmaster.getButton(Thrustmaster.Button.RIGHT).onTrue(indexer.sysId());
//    rightThrustmaster.getButton(Thrustmaster.Button.LEFT).onTrue(indexer.zero());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
