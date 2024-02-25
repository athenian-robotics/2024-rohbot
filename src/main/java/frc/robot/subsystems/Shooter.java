package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ShooterDataTable;
import frc.robot.ShooterSpec;
import frc.robot.inputs.PoseEstimator;
import frc.robot.lib.SimpleVelocitySystem;
import lombok.Getter;
import monologue.Annotations.Log;

public class Shooter extends SubsystemBase {
  private static final int LEFT_DRIVE_ID = 11;
  private static final int RIGHT_DRIVE_ID = 12;
  private static final Measure<Voltage> kS = Volts.of(.01);
  private static final Measure<Per<Voltage, Velocity<Angle>>> kV = VoltsPerRadianPerSecond.of(.087);
  private static final Measure<Per<Voltage, Velocity<Velocity<Angle>>>> kA =
      VoltsPerRadianPerSecondSquared.of(0.06);
  private static final double MAX_ERROR = 1;
  private static final double MAX_CONTROL_EFFORT = 8;
  private static final double MODEL_DEVIATION = 1;
  private static final double ENCODER_DEVIATION =
      1 / 42.0; // 1 tick of built-in neo encoder with reduction
  private static final Measure<Time> LOOP_TIME = Second.of(0.02);

  private final PoseEstimator poseEstimator;
  private final CANSparkMax driveL =
      new CANSparkMax(LEFT_DRIVE_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax driveR =
      new CANSparkMax(RIGHT_DRIVE_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final SimpleVelocitySystem sysL;
  private final SimpleVelocitySystem sysR;
  private final ShooterDataTable table;
  private final TimeOfFlight sensor;
  private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Velocity<Angle>> velocity = mutable(Rotations.per(Second).of(0));
  private final SysIdRoutine routineL;
  private final SysIdRoutine routineR;
  @Getter @Log.NT private State state = State.LOOKING_FOR_NOTE;

  public Shooter(ShooterDataTable table, PoseEstimator poseEstimator, TimeOfFlight sensor) {

    this.table = table;
    this.sensor = sensor;

    sysL =
        new SimpleVelocitySystem(
            kS.in(Volts),
            kV.in(VoltsPerRadianPerSecond),
            kA.in(VoltsPerRadianPerSecondSquared),
            MAX_ERROR,
            MAX_CONTROL_EFFORT,
            MODEL_DEVIATION,
            ENCODER_DEVIATION,
            LOOP_TIME.in(Seconds));
    sysR =
        new SimpleVelocitySystem(
            kS.in(Volts),
            kV.in(VoltsPerRadianPerSecond),
            kA.in(VoltsPerRadianPerSecondSquared),
            MAX_ERROR,
            MAX_CONTROL_EFFORT,
            MODEL_DEVIATION,
            ENCODER_DEVIATION,
            LOOP_TIME.in(Seconds));
    routineL =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Seconds).of(1), Volts.of(12), Seconds.of(10)),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> driveL.setVoltage(volts.in(Volts)),
                this::logL,
                this,
                "left-flywheel-motor"));
    routineR =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Seconds).of(1), Volts.of(12), Seconds.of(10)),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> driveR.setVoltage(volts.in(Volts)),
                this::logR,
                this,
                "right flywheel motor"));
    this.poseEstimator = poseEstimator;
  }

  private void logR(SysIdRoutineLog log) {
    log.motor("right-flywheel-motor")
        .voltage(appliedVoltage.mut_replace(driveL.getBusVoltage() * driveL.get(), Volts))
        .angularVelocity(velocity.mut_replace(driveL.getBusVoltage(), Rotations.per(Second)));
  }

  private void logL(SysIdRoutineLog log) {
    log.motor("left-flywheel-motor")
        .voltage(appliedVoltage.mut_replace(driveR.getBusVoltage() * driveR.get(), Volts))
        .angularVelocity(velocity.mut_replace(driveR.getBusVoltage(), Rotations.per(Second)));
  }

  @Log.NT
  private Measure<Velocity<Angle>> getWheelSpeedL() {
    return Units.RotationsPerSecond.of(driveL.getEncoder().getVelocity());
  }

  @Log.NT
  private Measure<Velocity<Angle>> getWheelSpeedR() {
    return Units.RotationsPerSecond.of(driveR.getEncoder().getVelocity());
  }

  private boolean atSetpoint() {
    return Math.abs(sysL.getError()) < MAX_ERROR && Math.abs(sysR.getError()) < MAX_ERROR;
  }

  public Command waitUntilReady() {
    return waitUntil(() -> state == State.READY);
  }

  public Command testing() {
    return runOnce(() -> state = State.TESTING);
  }

  public Command sysIdQuasistaticL(SysIdRoutine.Direction direction) {
    state = State.SYSID;
    return routineL.quasistatic(direction);
  }

  public Command sysIdDynamicL(SysIdRoutine.Direction direction) {
    state = State.SYSID;
    return routineL.dynamic(direction);
  }

  public Command sysIdQuasistaticR(SysIdRoutine.Direction direction) {
    state = State.SYSID;
    return routineR.quasistatic(direction);
  }

  public Command sysIdDynamicR(SysIdRoutine.Direction direction) {
    state = State.SYSID;
    return routineR.dynamic(direction);
  }

  public Command requestShot() {
    return runOnce(() -> state = State.APPROACHING).onlyIf(() -> state == State.NOTE_FOUND);
  }

  @Override
  public void periodic() {
    sysL.update(getWheelSpeedL().in(RadiansPerSecond)); // Returns RPS
    sysR.update(getWheelSpeedR().in(RadiansPerSecond));
    switch (state) {
      case LOOKING_FOR_NOTE:
        if (sensor.getRange() <= 60) { // TODO: make it a constant
          state = State.NOTE_FOUND;
          break;
        }

        sysL.set(RotationsPerSecond.of(5).in(RadiansPerSecond));
        sysR.set(RotationsPerSecond.of(5).in(RadiansPerSecond));
        break;

      case NOTE_FOUND:
        sysL.set(0);
        sysR.set(0);
        break;

      case TESTING:
        // log values
        break;
      case READY:
        if (!atSetpoint()) {
          state = State.APPROACHING;
        }
        if (sensor.getRange() >= 290) { // TODO: make it a constant
          state = State.LOOKING_FOR_NOTE;
        }
        break;
      case APPROACHING:
        // send limelight data to data table, send result to system
        ShooterSpec spec = table.get(poseEstimator.translationToSpeaker());
        sysL.set(spec.speedL().in(RotationsPerSecond));
        sysR.set(spec.speedR().in(RotationsPerSecond));
        if (sysR.getError() <= 30 && sysL.getError() <= 30) {
          state = State.READY;
        }
        break;
      case SYSID:
        break;
    }
    if (atSetpoint()) {
      state = State.READY;
    }
    driveL.set(sysL.getOutput());
    driveR.set(sysR.getOutput());
  }

  private enum State {
    READY, // at setpoint and within tolerance
    APPROACHING, // approaching setpoint
    TESTING, // for collecting shooter data table values
    SYSID,
    LOOKING_FOR_NOTE,
    NOTE_FOUND, // for system identification
  }
}
