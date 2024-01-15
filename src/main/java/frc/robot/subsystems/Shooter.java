package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.geometry.Translation2d;
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
  private static final int LEFT_ID = 0;
  private static final int RIGHT_ID = 0;
  private static final double kS = 0;
  private static final double kV = 0;
  private static final double kA = 0;
  private static final double MAX_ERROR = 0;
  private static final double MAX_CONTROL_EFFORT = 0.0;
  private static final double MODEL_DEVIATION = 0;
  private static final double ENCODER_DEVIATION = 0;
  private static final double LOOPTIME = 0.02;

  private final CANSparkMax motorL =
      new CANSparkMax(LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorR =
      new CANSparkMax(RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder encoderL;
  private final RelativeEncoder encoderR;
  private final SimpleVelocitySystem sysL;
  private final SimpleVelocitySystem sysR;
  private final ShooterDataTable table;
  @Getter @Log.NT private State state = State.IDLE;
  private SysIdRoutine routineL;
  private SysIdRoutine routineR;

  public Shooter(ShooterDataTable table) {
    encoderL = motorL.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    encoderR = motorR.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

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
    //        routineL = new SysIdRoutine(
    //                new SysIdRoutine.Config(),
    //                new SysIdRoutine.Mechanism((Measure<Voltage> volts) ->
    // motorL.setVoltage(volts.in(Volts)), encoderL, this, "left flywheel motor")
    //        )
  }

  private void motorLLog(SysIdRoutineLog log) {}

  @Log
  private double getWheelSpeedL() {
    return encoderL.getVelocity();
  }

  @Log
  private double getWheelSpeedR() {
    return encoderR.getVelocity();
  }

  private boolean atSetpoint() {
    return Math.abs(sysL.getError()) < MAX_ERROR && Math.abs(sysR.getError()) < MAX_ERROR;
  }

  public Command requestShot() {
    return new InstantCommand(() -> state = State.APPROACHING, this);
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

  public Command sysId() {
    return new InstantCommand(() -> state = State.SYSID);
  }

  public Command sysIdQuasistaticL(SysIdRoutine.Direction direction) {
    return routineL.quasistatic(direction);
  }

  public Command sysIdDynamicL(SysIdRoutine.Direction direction) {
    return routineL.dynamic(direction);
  }

  public Command sysIdQuasistaticR(SysIdRoutine.Direction direction) {
    return routineR.quasistatic(direction);
  }

  public Command sysIdDynamicR(SysIdRoutine.Direction direction) {
    return routineR.dynamic(direction);
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
        Translation2d toSpeaker = new Translation2d(); // TODO: get poseestimator data
        ShooterSpec spec = table.get(toSpeaker);
        sysL.set(spec.speedL());
        sysR.set(spec.speedR());
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
    SYSID
  }
}
