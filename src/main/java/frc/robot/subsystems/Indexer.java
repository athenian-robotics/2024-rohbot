package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.ShooterDataTable;
import frc.robot.inputs.PoseEstimator;
import lombok.Getter;

public class Indexer extends SubsystemBase {
  private static final int INDEXER_MOTOR_ID = 0; // TODO: ADD MOTOR PORTS ACCURATELY
  private static final int LEAD_ANGLE_MOTOR_ID = 0; // TODO: port
  private static final int FOLLOW_ANGLE_MOTOR_ID = 0; // TODO: number
  private static final double kV = 0; // TODO: Sysid
  private static final double kA = 0; // TODO: sysid
  private static final double kS = 0;
  private static final double kG = 0;
  private static final Measure<Angle> ANGLE_STANDARD_DEVIATION = null;
  private static final Measure<Angle> ANGLE_ERROR_TOLERANCE = null;
  private static final Measure<Velocity<Angle>> ANGLE_SPEED_STANDARD_DEVIATION = null;
  private static final Measure<Velocity<Angle>> ANGLE_SPEED_ERROR_TOLERANCE = null;
  private static final Measure<Angle> TICKS_TO_ANGLE =
      Degrees.of(0); // TODO: adjust for this year's robot
  private static final Measure<Voltage> MAX_ANGLE_MOTOR_VOLTAGE = Units.Volts.of(12);
  private static final Measure<Distance> NOTE_LOADED_THRESHOLD = Units.Inches.of(0); // TODO: Tune
  private static final Measure<Distance> SHOT_FIRED_THRESHOLD = Units.Inches.of(0); // TODO: Tune
  private static final Measure<Time> ROBOT_TIME_STEP = Units.Milli(Units.Milliseconds).of(20);
  private final PoseEstimator poseEstimator;
  private final CANSparkMax indexMotor;
  private final CANSparkMax leadAngleMotor;
  private final TimeOfFlight sensor;
  private final LinearSystemLoop<N2, N1, N1> loop;
  private final ShooterDataTable table;
  @Getter private State state;

  // private final Rev2mDistanceSensor sensor;

  public Indexer(ShooterDataTable table, final PoseEstimator poseEstimator) {
    indexMotor = new CANSparkMax(INDEXER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    leadAngleMotor = new CANSparkMax(LEAD_ANGLE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax followAngleMotor =
        new CANSparkMax(FOLLOW_ANGLE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    followAngleMotor.follow(leadAngleMotor);
    this.table = table;
    sensor = new TimeOfFlight(0); // TODO: Fill with the right sensor id

    LinearSystem<N2, N1, N1> sys = LinearSystemId.identifyPositionSystem(kV, kA);

    KalmanFilter<N2, N1, N1> filter =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            sys,
            VecBuilder.fill(
                ANGLE_STANDARD_DEVIATION.in(Units.Radians),
                ANGLE_SPEED_STANDARD_DEVIATION.in(Units.RadiansPerSecond)),
            VecBuilder.fill(TICKS_TO_ANGLE.in(Units.Radians)),
            ROBOT_TIME_STEP.in(Units.Seconds));

    LinearQuadraticRegulator<N2, N1, N1> controller =
        new LinearQuadraticRegulator<>(
            sys,
            VecBuilder.fill(
                ANGLE_ERROR_TOLERANCE.in(Units.Radians),
                ANGLE_SPEED_ERROR_TOLERANCE.in(Units.RadiansPerSecond)),
            VecBuilder.fill(MAX_ANGLE_MOTOR_VOLTAGE.in(Units.Volts)),
            ROBOT_TIME_STEP.in(Units.Seconds));

    loop =
        new LinearSystemLoop<>(
            sys,
            controller,
            filter,
            MAX_ANGLE_MOTOR_VOLTAGE.in(Units.Volts),
            ROBOT_TIME_STEP.in(Units.Seconds));

    this.poseEstimator = poseEstimator;
  }

  public boolean isLoading() {
    return this.getState() == State.LOADING;
  }

  public boolean isLoaded() {
    return this.getState() == State.LOADED;
  }

  public boolean isEmpty() {
    return this.getState() == State.EMPTY;
  }

  public boolean isInactive() {
    return this.getState() == State.EMPTY
        && !(this.getState() == State.LOADING)
        && !(this.getState() == State.LOADED);
  }

  public Command startLoading() {
    return runOnce(() -> state = State.LOADING);
  }

  public Command waitUntilReady() {
    return waitUntil(() -> state == State.READY);
  }

  public Command fire() {
    return runOnce(() -> state = State.FIRING);
  }

  @Override
  public void periodic() {
    switch (state) {
      case EMPTY, READY -> indexMotor.set(0);
      case LOADING -> {
        indexMotor.set(1); // TODO: Tune
        if (Units.Inches.of(sensor.getRange()).baseUnitMagnitude()
            < NOTE_LOADED_THRESHOLD.in(Units.Inches)) {
          state = State.LOADED;
        }
      }
      case LOADED -> {
        indexMotor.set(0);
        loop.setNextR(table.get(poseEstimator.translationToSpeaker()).angle().in(Units.Radians));
        if (loop.getError(1) < ANGLE_ERROR_TOLERANCE.in(Units.Radians)
            && loop.getError(2) < ANGLE_SPEED_ERROR_TOLERANCE.in(Units.RadiansPerSecond)) {
          state = State.READY;
        }
      }
      case FIRING -> {
        indexMotor.set(1); // TODO: Tune
        if (Units.Inches.of(sensor.getRange()).baseUnitMagnitude()
            < SHOT_FIRED_THRESHOLD.in(Units.Inches)) {
          state = State.EMPTY;
        }
      }
    }
    loop.correct(
        VecBuilder.fill(
            leadAngleMotor.getEncoder().getPosition() * TICKS_TO_ANGLE.in(Units.Radians)));
    loop.predict(ROBOT_TIME_STEP.in(Units.Seconds));
    leadAngleMotor.set(
        loop.getU(0) + kS * Math.signum(loop.getNextR(1) + kG * Math.cos(loop.getNextR(0))));
  }

  private enum State {
    FIRING,
    LOADING,
    LOADED,
    READY,
    EMPTY
  }
}
