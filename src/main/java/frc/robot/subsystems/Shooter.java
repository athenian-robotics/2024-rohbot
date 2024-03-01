package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ShooterDataTable;
import frc.robot.ShooterSpec;
import frc.robot.inputs.PoseEstimator;
import frc.robot.lib.SimpleVelocitySystem;
import frc.robot.lib.TunableNumber;
import java.util.Optional;
import lombok.Getter;
import monologue.Annotations.Log;

public class Shooter extends SubsystemBase {
  private static final int LEFT_DRIVE_ID = 11;
  private static final int RIGHT_DRIVE_ID = 12;
  private static final int ACTIVATION_ID = -1; // TODO: fill
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
  private static final double EMPTY_THRESHOLD = 290;
  private static final int CURRENT_LIMIT = 20;
  private static final double TOTAL_CURRENT_LIMIT = CURRENT_LIMIT * 3;
  private static final double ACTIVATION_SPEED = 0.1; // TODO: Tune

  private final PoseEstimator poseEstimator;
  private final CANSparkMax driveL =
      new CANSparkMax(LEFT_DRIVE_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax driveR =
      new CANSparkMax(RIGHT_DRIVE_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax activation =
      new CANSparkMax(ACTIVATION_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final SimpleVelocitySystem sysL;
  private final SimpleVelocitySystem sysR;
  private final ShooterDataTable table;
  private final TimeOfFlight sensor;
  private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Velocity<Angle>> velocity = mutable(Rotations.per(Second).of(0));
  private final SysIdRoutine routineL;
  private final SysIdRoutine routineR;
  @Getter @Log.NT private State state = State.APPROACHING;
  private final TunableNumber numL = new TunableNumber("left shooter power deg/s", 0);
  private final TunableNumber numR = new TunableNumber("right shooter power deg/s", 0);
  private final TunableNumber numActivation =
      new TunableNumber("activation motor power percent 0 to 1", 0);
  private final PowerBudget power;

  public Shooter(
      ShooterDataTable table, PoseEstimator poseEstimator, TimeOfFlight sensor, PowerBudget power) {

    this.table = table;
    this.sensor = sensor;
    this.power = power;

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

    driveL.setSmartCurrentLimit(CURRENT_LIMIT);
    driveR.setSmartCurrentLimit(CURRENT_LIMIT);
    activation.setSmartCurrentLimit(CURRENT_LIMIT);
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

  public boolean ready() {
    return Math.abs(sysL.getError()) < MAX_ERROR && Math.abs(sysR.getError()) < MAX_ERROR;
  }

  public Command activate() {
    return new InstantCommand(() -> state = State.ACTIVATED, this);
  }

  public Command deactivate() {
    return new InstantCommand(() -> state = State.APPROACHING, this);
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

  @Override
  public void periodic() {
    switch (state) {
      case APPROACHING -> {
        if (power.hasCurrent(
            driveL.getOutputCurrent() + driveR.getOutputCurrent(), TOTAL_CURRENT_LIMIT * 3 / 2)) {
          sysL.update(getWheelSpeedL().in(RadiansPerSecond)); // Returns RPS
          sysR.update(getWheelSpeedR().in(RadiansPerSecond));

          Optional<ShooterSpec> spec = table.get(poseEstimator.translationToSpeaker());

          sysL.set(
              spec.map(ShooterSpec::speedL).orElse(DegreesPerSecond.of(0)).in(RadiansPerSecond));
          sysR.set(
              spec.map(ShooterSpec::speedR).orElse(DegreesPerSecond.of(0)).in(RadiansPerSecond));

          driveL.set(sysL.getOutput());
          driveR.set(sysR.getOutput());
          activation.set(0);
        }
      }
      case SYSID -> {
        activation.set(0);
        // chill
      }
      case TESTING -> {
        sysL.update(getWheelSpeedL().in(RadiansPerSecond)); // Returns RPS
        sysR.update(getWheelSpeedR().in(RadiansPerSecond));

        sysL.set(DegreesPerSecond.of(numL.get()).in(RadiansPerSecond));
        sysR.set(DegreesPerSecond.of(numR.get()).in(RadiansPerSecond));

        activation.set(numActivation.get());
        driveL.set(sysL.getOutput());
        driveR.set(sysR.getOutput());
      }
      case ACTIVATED -> {
        if (power.hasCurrent(
            driveL.getOutputCurrent() + driveR.getOutputCurrent() + activation.getOutputCurrent(),
            TOTAL_CURRENT_LIMIT)) {
          sysL.update(getWheelSpeedL().in(RadiansPerSecond)); // Returns RPS
          sysR.update(getWheelSpeedR().in(RadiansPerSecond));

          Optional<ShooterSpec> spec = table.get(poseEstimator.translationToSpeaker());

          sysL.set(
              spec.map(ShooterSpec::speedL).orElse(DegreesPerSecond.of(0)).in(RadiansPerSecond));
          sysR.set(
              spec.map(ShooterSpec::speedR).orElse(DegreesPerSecond.of(0)).in(RadiansPerSecond));

          driveL.set(sysL.getOutput());
          driveR.set(sysR.getOutput());
          activation.set(ACTIVATION_SPEED);
        }
      }
    }
  }

  public boolean empty() {
    return sensor.getRange() >= EMPTY_THRESHOLD;
  }

  private enum State {
    APPROACHING,
    SYSID,
    TESTING,
    ACTIVATED
  }
}
