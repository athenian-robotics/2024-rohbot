package frc.robot.subsystems.superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.subsystems.superstructure.SuperstructureIO.State.RangeState.IN_RANGE;
import static frc.robot.subsystems.superstructure.SuperstructureIO.State.RangeState.OUTSIDE_RANGE;
import static frc.robot.subsystems.superstructure.SuperstructureIO.State.SubsystemState.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterDataTable;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class SuperstructureIOSim implements SuperstructureIO {
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private final Drive poseEstimator;
  private final ShooterDataTable dataTable;
  private State state;
  private LoggedDashboardBoolean intakeOn = new LoggedDashboardBoolean("intake on", false);
  private LoggedDashboardBoolean shoot = new LoggedDashboardBoolean("shoot", false);
  private LoggedDashboardNumber sensorRange = new LoggedDashboardNumber("sensor range", 0);
  private LoggedDashboardNumber SHOOTER_SENSOR_THRESHOLD =
      new LoggedDashboardNumber("shooter sensor threshold", 250); // TODO: Tune

  public SuperstructureIOSim(
      Intake intake, Indexer indexer, Shooter shooter, Drive swerve, ShooterDataTable dataTable) {
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
    this.poseEstimator = swerve;
    this.dataTable = dataTable;
  }

  @Override
  public void iterateStateMachine() {
    State bufferedState = state;

    state =
        state.changeRangeState(
            dataTable.get(poseEstimator.translationToSpeaker()).isPresent()
                ? IN_RANGE
                : OUTSIDE_RANGE);

    if (shooterEmpty()) state = state.changeSubsystemState(NO_NOTE);
    if (!shooterEmpty() && state.state() == NO_NOTE) state = state.changeSubsystemState(HAS_NOTE);

    if (state == bufferedState)
      return; // preserve looptime by not running the state machine if the state hasn't changed

    switch (state.state()) {
      case NO_NOTE -> {
        intake.on();
        shooter.intake();
        indexer.setState(IndexerIO.State.IDLE);
        shooter.spinUp();
      }
      case HAS_NOTE -> {
        intake.off();
        switch (state.rangeState()) {
          case IN_RANGE -> indexer.setState(IndexerIO.State.ADJUSTING);
          case OUTSIDE_RANGE -> indexer.setState(IndexerIO.State.IDLE);
        }

        shooter.spinUp();
      }
      case SHOOTING -> {
        switch (state.rangeState()) {
          case IN_RANGE -> {
            shooter.spinUp();
            indexer.setState(IndexerIO.State.ADJUSTING);
            poseEstimator.faceSpeaker().schedule();
            if (shooter.ready() && indexer.ready() && poseEstimator.ready()) shooter.shoot();
          }
          case OUTSIDE_RANGE -> {
            // log?, should never happen
          }
        }
      }
      case TESTING -> {
        shooter.test();
        indexer.setState(IndexerIO.State.TESTING);

        if (intakeOn.get()) {
          intake.on();
        } else intake.off();

        if (shoot.get()) shooter.shoot();
        else shooter.test();
      }
      case SYSID -> {
        poseEstimator.setDisable(
            true); // disable high frequency (high overhead) swerve odometry to reduce looptime
        shooter.sysId();
        indexer.sysId();
      }
      case AMP -> {
        indexer.setState(IndexerIO.State.AMP);
        shooter.amp();
      }
    }
  }

  private boolean shooterEmpty() {
    return sensorRange.get() > SHOOTER_SENSOR_THRESHOLD.get();
  }

  @Override
  public Command shoot() {
    return runOnce(() -> state = state.changeSubsystemState(SHOOTING))
        .onlyIf(() -> !shooterEmpty())
        .onlyIf(() -> state.rangeState() == IN_RANGE);
  }

  @Override
  public Command test() {
    return runOnce(() -> state = state.changeSubsystemState(TESTING));
  }

  @Override
  public Command cancelShot() {
    return runOnce(
        shooterEmpty()
            ? () -> state = state.changeSubsystemState(NO_NOTE)
            : () -> state = state.changeSubsystemState(HAS_NOTE));
  }

  @Override
  public void updateInputs(SuperstructureIO.SuperstructureInputs inputs) {
    inputs.state = state;
    inputs.sensorRange = sensorRange.get();
    inputs.shooterEmpty = shooterEmpty();
  }
}
