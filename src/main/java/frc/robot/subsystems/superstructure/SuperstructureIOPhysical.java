package frc.robot.subsystems.superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.subsystems.superstructure.SuperstructureIO.State.RangeState.IN_RANGE;
import static frc.robot.subsystems.superstructure.SuperstructureIO.State.RangeState.OUTSIDE_RANGE;
import static frc.robot.subsystems.superstructure.SuperstructureIO.State.SubsystemState.*;

import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ShooterDataTable;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class SuperstructureIOPhysical implements SuperstructureIO {
  private static final LoggedDashboardNumber SHOOTER_SENSOR_THRESHOLD =
      new LoggedDashboardNumber("shooter sensor threshold", 75);
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Drive swerve;
  private final Drive poseEstimator;
  private final ShooterDataTable dataTable;
  private final TimeOfFlight shooterSensor = new TimeOfFlight(12);
  private final LoggedDashboardBoolean intakeOn = new LoggedDashboardBoolean("intake on", false);
  private final LoggedDashboardBoolean shoot = new LoggedDashboardBoolean("shoot", false);
  private State state = new State(NO_NOTE, OUTSIDE_RANGE);
  private boolean latch; // icky shared state... i hate java
  private Command command;
  private State bufferedState;

  public SuperstructureIOPhysical(
      Intake intake, Indexer indexer, Shooter shooter, Drive swerve, ShooterDataTable dataTable) {
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    this.swerve = swerve;
    this.poseEstimator = swerve;
    this.dataTable = dataTable;

    command =
        runOnce(() -> indexer.setState(IndexerIO.State.AMP_INIT))
            .alongWith(runOnce(shooter::amp))
            .andThen(waitUntil(() -> indexer.ready() && shooter.ready()))
            .andThen(
                runOnce(() -> indexer.setState(IndexerIO.State.AMP_PULSE))
                    .alongWith(runOnce(shooter::shoot)));
    latch = state.subsystemState() == TESTING || state.subsystemState() == SYSID;
    if (state.subsystemState() == TESTING) {
      new Trigger(intakeOn::get).onTrue(runOnce(intake::on));
      new Trigger(intakeOn::get).onFalse(runOnce(intake::off));
    }
  }

  @Override
  public void iterateStateMachine() {
    state =
        state.changeRangeState(
            dataTable.get(poseEstimator.translationToSpeaker()).isPresent()
                ? IN_RANGE
                : OUTSIDE_RANGE);

    if (!latch) { // cuh like we dont wanna change the jawn if we are chilling in testing or sysid
      if (shooterEmpty()) state = state.changeSubsystemState(NO_NOTE);
      if (!shooterEmpty() && state.subsystemState() == NO_NOTE)
        state = state.changeSubsystemState(HAS_NOTE);
    }

    if (state.equals(bufferedState))
      return; // preserve looptime by not running the subsystemState machine if the subsystemState
    // hasn't changed

    switch (state.subsystemState()) {
      case NO_NOTE -> {
        intake.on();
        shooter.intake();
        indexer.setState(IndexerIO.State.IDLE);
        shooter.spinUp();
      }
      case HAS_NOTE -> {
        intake.off();
        switch (state.rangeState()) {
            //          case IN_RANGE -> indexer.setState(IndexerIO.State.ADJUSTING);
            //          case OUTSIDE_RANGE -> indexer.setState(IndexerIO.State.IDLE);
          default -> indexer.setState(IndexerIO.State.FLAT);
        }

        shooter.spinUp();
      }
      case SHOOTING -> {
        switch (state.rangeState()) {
          case IN_RANGE -> {
            shooter.spinUp();
            indexer.setState(IndexerIO.State.ADJUSTING);
            swerve.faceSpeaker().schedule();
          }
          case OUTSIDE_RANGE -> {
            // log?, should never happen
          }
        }
      }
      case TESTING -> {
        shooter.test();
        indexer.setState(IndexerIO.State.TESTING);

        new Trigger(intakeOn::get).onTrue(runOnce(intake::on));
        new Trigger(intakeOn::get).onFalse(runOnce(intake::off));
      }
      case SYSID -> {
        swerve.setDisable(
            true); // disable high frequency (high overhead) swerve odometry to reduce looptime
        shooter.sysIdState();
        indexer.setState(IndexerIO.State.SYSID);
      }
      case SHOOT_FIXED -> {
        shooter.shootFixed();
        indexer.setState(IndexerIO.State.SHOOTFIXED);
        new Trigger(
                () ->
                    shooter.ready()
                        && indexer.ready()
                        && bufferedState.subsystemState() == SHOOT_FIXED)
            .onTrue(runOnce(shooter::shoot));
      }
      case AMP -> {
        if (!command.isScheduled()) command.schedule();
      }
    }

    bufferedState = state;
  }

  @Override
  public Command shoot() {
    return runOnce(
            () -> {
              latch = false;
              state = state.changeSubsystemState(SHOOTING);
            })
        .onlyIf(() -> !shooterEmpty())
        .onlyIf(() -> state.rangeState() == IN_RANGE);
  }

  @Override
  public Command test() {
    return runOnce(
        () -> {
          latch = true;
          state = state.changeSubsystemState(TESTING);
        });
  }

  @Override
  public Command idle() {
    return runOnce(
        () -> {
          latch = false;
          state =
              shooterEmpty()
                  ? state.changeSubsystemState(NO_NOTE)
                  : state.changeSubsystemState(HAS_NOTE);
        });
  }

  @Override
  public Command cancelShot() {
    return runOnce(
        () -> {
          latch = false;
          state =
              shooterEmpty()
                  ? state.changeSubsystemState(NO_NOTE)
                  : state.changeSubsystemState(HAS_NOTE);
        });
  }

  private boolean shooterEmpty() {
    return shooterSensor.getRange() > SHOOTER_SENSOR_THRESHOLD.get();
  }

  @Override
  public void updateInputs(SuperstructureInputs inputs) {
    inputs.sensorRange = shooterSensor.getRange();
    inputs.shooterEmpty = shooterEmpty();
    inputs.state = state;
    inputs.bufferedState = bufferedState.toString();
    inputs.stateString = state.toString();
  }

  @Override
  public Command waitUntilEmpty() {
    return waitUntil(this::shooterEmpty);
  }

  // private Command onTheFlyRobotToNote() {
  // Translation2d robotPosition = poseEstimator.getPose().getTranslation();
  // Translation2d notePosition = noteDetector.getClosestNoteTranslation().get();

  // ArrayList<Translation2d> bezierPoints = new ArrayList<Translation2d>();
  // bezierPoints.add(robotPosition);
  // bezierPoints.add(notePosition);

  // double dotProduct = robotPosition.getX() * notePosition.getX() +
  // robotPosition.getY() * notePosition.getY();
  // Rotation2d rotationNeeded = new Rotation2d(dotProduct /
  // (robotPosition.getNorm() * notePosition.getNorm()));

  // PathPlannerPath pathToNote =
  // new PathPlannerPath(
  // bezierPoints,
  // new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // TODO: Tune
  // new GoalEndState(
  // 0.5,
  // rotationNeeded)); // NOTE: rotationNeeded might not work here

  // return AutoBuilder.followPath(pathToNote);
  // }

  public Command amp() {
    return runOnce(
        () -> {
          latch = false;
          state = state.changeSubsystemState(AMP);
        });
  }

  @Override
  public Command sysId() {
    return runOnce(
        () -> {
          latch = true;
          state = state.changeSubsystemState(SYSID);
        });
  }

  @Override
  public Command shootFixed() {
    return runOnce(
            () -> {
              latch = false;
              state = state.changeSubsystemState(SHOOT_FIXED);
            })
        .onlyIf(() -> !shooterEmpty());
  }

  @Override
  public boolean noNote() {
    return state.subsystemState() == NO_NOTE;
  }
}
