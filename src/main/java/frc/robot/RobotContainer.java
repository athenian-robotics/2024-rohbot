package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.lib.controllers.Thrustmaster;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOPhysical;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOFalcons;
import frc.robot.subsystems.intake.IntakeSim;
import frc.robot.subsystems.powerBudget.PowerBudget;
import frc.robot.subsystems.powerBudget.PowerBudgetPhysical;
import frc.robot.subsystems.powerBudget.PowerBudgetSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOPhysicalPID;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureIOPhysical;
import frc.robot.subsystems.superstructure.SuperstructureIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;

public class RobotContainer {

  private static final Thrustmaster leftThrustmaster = new Thrustmaster(0);
  private static final Thrustmaster rightThrustmaster = new Thrustmaster(1);

  private static final Thrustmaster opThrustmaster = new Thrustmaster(2);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private Drive drivebase;
  private Superstructure superstructure;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> { //     TODO: Ensure to get the actual points
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
                new Vision(new VisionIOPhoton()));

        // TODO: Remember to replace with the actual camera name
        //         noteDetector = new NoteDetector(photonCamera, poseEstimator);
        Intake intake = new Intake(new IntakeIOFalcons());

        PowerBudgetPhysical power = new PowerBudgetPhysical();
        Shooter shooter = new Shooter(new ShooterIOPhysicalPID(shooterDataTable, drivebase, power));
        Indexer indexer = new Indexer(new IndexerIOPhysical(shooterDataTable, drivebase, power));

        superstructure =
            new Superstructure(
                new SuperstructureIOPhysical(
                    intake, indexer, shooter, drivebase, shooterDataTable));
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
        superstructure =
            new Superstructure(
                new SuperstructureIOSim(intake, indexer, shooter, drivebase, shooterDataTable));
      }
      case REPLAY -> { // this doesnt work and I have no idea how it is supposed to work
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
        superstructure =
            new Superstructure(
                new SuperstructureIOPhysical(
                    intake, indexer, shooter, drivebase, shooterDataTable));
      }
    }
    configureBindings();
  }

  private void configureBindings() {
    // TODO: consider adding rizzed auto chooser to go between testing and match
    rightThrustmaster
        .getButton(Thrustmaster.Button.BOTTOM)
        .onTrue(
            runOnce(
                () ->
                    drivebase.setPose(
                        new Pose2d(drivebase.getPose().getTranslation(), new Rotation2d()))));

    // chat, idea thinks im dumb...
    //noinspection SuspiciousNameCombination
    drivebase.setDefaultCommand(
        drivebase.joystickDrive(
            leftThrustmaster::getY, leftThrustmaster::getX, rightThrustmaster::getX));

    rightThrustmaster.getButton(Thrustmaster.Button.TRIGGER).onTrue(superstructure.shoot());

    // TODO: these jawns should not be binded in a match
    // bro presses on button and unrecoverably bricks the robot :skull:
    rightThrustmaster.getButton(Thrustmaster.Button.LEFT).onTrue(superstructure.test());

    leftThrustmaster
        .getButton(Thrustmaster.Button.TRIGGER)
        .onTrue(drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    leftThrustmaster
        .getButton(Thrustmaster.Button.BOTTOM)
        .onTrue(drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    leftThrustmaster
        .getButton(Thrustmaster.Button.LEFT)
        .onTrue(drivebase.sysIdDynamic(SysIdRoutine.Direction.kForward));
    leftThrustmaster
        .getButton(Thrustmaster.Button.RIGHT)
        .onTrue(drivebase.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
