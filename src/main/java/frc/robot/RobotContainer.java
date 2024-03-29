package frc.robot;

import static com.pathplanner.lib.auto.AutoBuilder.*;
import static com.pathplanner.lib.path.PathPlannerPath.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private static final Thrustmaster leftThrustmaster = new Thrustmaster(0);
  private static final Thrustmaster rightThrustmaster = new Thrustmaster(1);

  private static final Thrustmaster opThrustmaster = new Thrustmaster(2);
  private Indexer indexer;
  private Shooter shooter;
  private Drive drivebase;
  private Superstructure superstructure;
  private final LoggedDashboardChooser<Command> autoChooser;
  private Command taxiPath = new InstantCommand();

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
        shooter = new Shooter(new ShooterIOPhysicalPID(shooterDataTable, drivebase, power));
        indexer = new Indexer(new IndexerIOPhysical(shooterDataTable, drivebase, power));

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
        indexer = new Indexer(new IndexerIOSim(shooterDataTable, drivebase, powerBudget));
        shooter = new Shooter(new ShooterIOSim(shooterDataTable, drivebase, powerBudget));
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
        indexer = new Indexer(new IndexerIOSim(shooterDataTable, drivebase, powerBudget));
        shooter = new Shooter(new ShooterIOSim(shooterDataTable, drivebase, powerBudget));
        superstructure =
            new Superstructure(
                new SuperstructureIOPhysical(
                    intake, indexer, shooter, drivebase, shooterDataTable));
      }
    }

    NamedCommands.registerCommand(
        "shoot", superstructure.shoot().alongWith(superstructure.waitUntilEmpty()));
    autoChooser = new LoggedDashboardChooser<>("auto chooser", buildAutoChooser());
    autoChooser.addOption("4note", fourNote(true));
    autoChooser.addOption("3note", threeNote(true));
    autoChooser.addOption("middle to taxi", runOnce(() -> {}));
    autoChooser.addOption("5note", fiveNote());
    autoChooser.addOption(
        "1note",
        sequence(
            waitSeconds(3),
            shootAndWait(),
            waitSeconds(5),
            resetToStartingPose(fromChoreoTrajectory("placeholder taxi")),
            followPath(fromChoreoTrajectory("placeholder taxi"))));
    configureBindings();
  }

  // 18.5 s we need to take 1 s to shoot
  private Command fiveNote() {
    return sequence(
        fourNote(false), // 12.3 s
        followPath(fromChoreoTrajectory("middle to 4th note to middle")), // 4.2
        shootAndWait()); // 2 s
  }

  // 8.7 s
  private Command threeNote(boolean isFirstAuto) {
    var firstPath = fromChoreoTrajectory("middle to 2nd note to middle");
    return sequence(
        resetToStartingPose(firstPath).onlyIf(() -> isFirstAuto),
        shootAndWait(), // 2 s
        followPath(firstPath), // 1.5 s
        shootAndWait(), // 2 s
        followPath(fromChoreoTrajectory("middle to 1st note to middle")), // 1.2 s
        shootAndWait()); // 2 s
  }

  // wait 3 s shoot wait 9 taxi

  private Command shootAndWait() {
    return superstructure
        .shootFixed()
        .andThen(race(superstructure.waitUntilEmpty(), waitSeconds(2)));
  }

  private Command resetToStartingPose(PathPlannerPath firstPath) {
    return runOnce(
        () ->
            drivebase.setPose(
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                    ? firstPath.getPreviewStartingHolonomicPose()
                    : firstPath
                        .flipPath()
                        .getPreviewStartingHolonomicPose()
                        .plus(new Transform2d(0, 0, new Rotation2d(Math.PI)))));
  }

  private Command fourNote(boolean taxi) {
    var firstPath = fromChoreoTrajectory("middle to 3d note to middle");
    return sequence(
        resetToStartingPose(firstPath),
        shootAndWait(),
        followPath(firstPath),
        threeNote(false)); // 2 s + 1.6 s + 8.7 s = 12.3 s
  }

  private Command middleToTaxi() {
    return followPath(fromChoreoTrajectory("middle to taxi"));
  }

  private void configureBindings() {
    leftThrustmaster
        .getButton(Thrustmaster.Button.BOTTOM)
        .onTrue(
            runOnce(
                () ->
                    drivebase.setPose(
                        new Pose2d(drivebase.getPose().getTranslation(), new Rotation2d()))));

    /*
     *    ░▒▓███████▓▒░ ░▒▓███████▓▒░ ░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓████████▓▒░
     *    ░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░
     *    ░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░ ░▒▓█▓▒▒▓█▓▒░ ░▒▓█▓▒░
     *    ░▒▓█▓▒░░▒▓█▓▒░░▒▓███████▓▒░ ░▒▓█▓▒░ ░▒▓█▓▒▒▓█▓▒░ ░▒▓██████▓▒░
     *    ░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░  ░▒▓█▓▓█▓▒░  ░▒▓█▓▒░
     *    ░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░  ░▒▓█▓▓█▓▒░  ░▒▓█▓▒░
     *    ░▒▓███████▓▒░ ░▒▓█▓▒░░▒▓█▓▒░░▒▓█▓▒░   ░▒▓██▓▒░   ░▒▓████████▓▒░
     */
    // chat, intellij thinks im dumb...
    //noinspection SuspiciousNameCombination
    drivebase.setDefaultCommand(
        drivebase.joystickDrive(
            () -> -leftThrustmaster.getY(),
            () -> leftThrustmaster.getX(),
            rightThrustmaster::getX));

    //    rightThrustmaster.getButton(Thrustmaster.Button.RIGHT).onTrue(superstructure.test());
    //    rightThrustmaster.getButton(Thrustmaster.Button.LEFT).onTrue(superstructure.idle());
    leftThrustmaster
        .getButton(Thrustmaster.Button.LEFT_INSIDE_BOTTOM)
        .onTrue(superstructure.sysId().andThen(indexer.sysId()));

    /*
     *     ________   ___  ___   ________   ________   _________   ___   ________    ________
     *    |\   ____\ |\  \|\  \ |\   __  \ |\   __  \ |\___   ___\|\  \ |\   ___  \ |\   ____\
     *    \ \  \___|_\ \  \\\  \\ \  \|\  \\ \  \|\  \\|___ \  \_|\ \  \\ \  \\ \  \\ \  \___|
     *     \ \_____  \\ \   __  \\ \  \\\  \\ \  \\\  \    \ \  \  \ \  \\ \  \\ \  \\ \  \  ___
     *      \|____|\  \\ \  \ \  \\ \  \\\  \\ \  \\\  \    \ \  \  \ \  \\ \  \\ \  \\ \  \|\  \
     *        ____\_\  \\ \__\ \__\\ \_______\\ \_______\    \ \__\  \ \__\\ \__\\ \__\\ \_______\
     *       |\_________\\|__|\|__| \|_______| \|_______|     \|__|   \|__| \|__| \|__| \|_______|
     *       \|_________|
     */
    rightThrustmaster.getButton(Thrustmaster.Button.TRIGGER).onTrue(superstructure.shootFixed());
    rightThrustmaster.getButton(Thrustmaster.Button.BOTTOM).onTrue(superstructure.cancelShot());
    leftThrustmaster
        .getButton(Thrustmaster.Button.TRIGGER)
        .onTrue(superstructure.shootAcrossField());

    // operator
    opThrustmaster.getButton(Thrustmaster.Button.LEFT).onTrue(superstructure.reverseIntake());
    opThrustmaster.getButton(Thrustmaster.Button.RIGHT).onTrue(superstructure.idle());
    indexer.fudge(() -> -opThrustmaster.getRawAxis(3));
  }

  public Command getAutonomousCommand() {
    return sequence(autoChooser.get(), taxiPath);
  }
}
