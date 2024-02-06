package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ShooterDataTable;
import frc.robot.ShooterSpec;
import frc.robot.lib.SimpleVelocitySystem;
import lombok.Getter;
import monologue.Annotations.Log;

public class Shooter extends SubsystemBase {
  // TODO: fill in values
  private static final int LEFT_DRIVE_ID = 0;
  private static final int RIGHT_DRIVE_ID = 0;
  private static final int LEAD_TRIGGER_ID = 0;
  private static final int FOLLOW_TRIGGER_ID = 0;
  private static final double kS = 0;
  private static final double kV = 0;
  private static final double kA = 0;
  private static final double MAX_ERROR = 0;
  private static final double MAX_CONTROL_EFFORT = 0.0;
  private static final double MODEL_DEVIATION = 0;
  private static final double ENCODER_DEVIATION = 0;
  private static final double LOOPTIME = 0.02;

  private final TalonFX driveL =
          new TalonFX(LEFT_DRIVE_ID, "rio"); //TODO: Make sure the canbus is right.
  private final TalonFX driveR =
          new TalonFX(RIGHT_DRIVE_ID, "rio");
  private final CANSparkMax leadTrigger = new CANSparkMax(LEAD_TRIGGER_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax followTrigger = new CANSparkMax(FOLLOW_TRIGGER_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final SimpleVelocitySystem sysL;
  private final SimpleVelocitySystem sysR;
  private final ShooterDataTable table;
  private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Velocity<Angle>> velocity = mutable(Rotations.per(Minute).of(0));
  @Getter @Log.NT private State state = State.IDLE;
  private final SysIdRoutine routineL;
  private final SysIdRoutine routineR;

  private Translation2d translationToSpeaker;

  public Shooter(ShooterDataTable table) {

    followTrigger.follow(leadTrigger);

    this.table = table;

    sysL =
            new SimpleVelocitySystem(
                    kS,
                    kV,
                    kA,
                    MAX_ERROR,
                    MAX_CONTROL_EFFORT,
                    MODEL_DEVIATION,
                    ENCODER_DEVIATION,
                    LOOPTIME);
    sysR =
            new SimpleVelocitySystem(
                    kS,
                    kV,
                    kA,
                    MAX_ERROR,
                    MAX_CONTROL_EFFORT,
                    MODEL_DEVIATION,
                    ENCODER_DEVIATION,
                    LOOPTIME);
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
  }

  private void logR(SysIdRoutineLog log) {
    log.motor("right-flywheel-motor")
            .voltage(appliedVoltage.mut_replace(driveL.getSupplyVoltage().getValue() * driveL.get(), Volts))
            .angularVelocity(velocity.mut_replace(driveL.getVelocity().getValue(), Rotations.per(Second)));
  }

  private void logL(SysIdRoutineLog log) {
    log.motor("left-flywheel-motor")
            .voltage(appliedVoltage.mut_replace(driveR.getSupplyVoltage().getValue() * driveR.get(), Volts))
            .angularVelocity(velocity.mut_replace(driveR.getVelocity().getValue(), Rotations.per(Second)));
  }

  @Log.NT
  private double getWheelSpeedL() {
    return driveL.getVelocity().getValue()*60; //Returns velocity in RPM.
  }

  @Log.NT
  private double getWheelSpeedR() {
    return driveR.getVelocity().getValue()*60; //Returns velocity in RPM.
  }

  private boolean atSetpoint() {
    return Math.abs(sysL.getError()) < MAX_ERROR && Math.abs(sysR.getError()) < MAX_ERROR;
  }

  public Command requestShot(Translation2d translationToSpeaker) {
    return new InstantCommand(
            () -> {
              state = State.APPROACHING;
              this.translationToSpeaker = translationToSpeaker;
            },
            this);
  }

  public Command idle() {
    return new InstantCommand(() -> state = State.IDLE, this);
  }

  public Command waitUntilReady() {
    return new WaitUntilCommand(() -> state == State.READY);
  }

  public Command testing() {
    return new InstantCommand(() -> state = State.TESTING);
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

  public Command fire() {
    return new InstantCommand(() -> state = State.FIRING, this);
  }

  @Override
  public void periodic() {
    sysL.update(getWheelSpeedL()); // Returns RPM
    sysR.update(getWheelSpeedR());
    switch (state) {
      case IDLE:
        sysL.set(0.0);
        sysR.set(0.0);
        break;
      case TESTING:
        // log values
        break;
      case APPROACHING:
        // send limelight data to data table, send result to system
        ShooterSpec spec = table.get(translationToSpeaker);
        sysL.set(spec.speedL());
        sysR.set(spec.speedR());
        break;
      case FIRING:
        leadTrigger.set(0.5); // TODO: Tune
        break;
      case SYSID:
        break;
    }
    if (atSetpoint()) {
      state = State.READY;
    }
  }

  private enum State {
    IDLE, // default state
    READY, // at setpoint and within tolerance
    APPROACHING, // approaching setpoint
    TESTING, // for collecting shooter data table values
    SYSID, // for system identification
    FIRING,
  }
}
