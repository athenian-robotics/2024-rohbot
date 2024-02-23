package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;
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
import frc.robot.ShooterDataTable;
import frc.robot.inputs.PoseEstimator;
import lombok.Getter;

public class Indexer extends SubsystemBase {
  private static final int LEAD_ANGLE_MOTOR_ID = 11; // TODO: port
  private static final int FOLLOW_ANGLE_MOTOR_ID = 12; // TODO: number
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
  private static final Measure<Distance> SHOT_FIRED_THRESHOLD = Units.Inches.of(0); // TODO: Tune
  private static final Measure<Time> ROBOT_TIME_STEP = Units.Milli(Units.Milliseconds).of(20);
  private static final Measure<Angle> IDLE_ANGLE = Units.Radians.of(0);
  private final PoseEstimator poseEstimator;
  private final CANSparkMax leadAngleMotor;
  private final TimeOfFlight sensor;
  private final LinearSystemLoop<N2, N1, N1> loop;
  private final ShooterDataTable table;
  @Getter private State state;

  // private final Rev2mDistanceSensor sensor;

  public Indexer(ShooterDataTable table, final PoseEstimator poseEstimator) {
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

  public boolean isApproaching() {
    return this.getState() == State.APPROACHING;
  }

  public boolean isInactive() {
    return this.getState() == State.IDLE;
  }

  public Command waitUntilReady() {
    return waitUntil(() -> state == State.READY);
  }

  public Command fire() {
    return runOnce(() -> state = State.APPROACHING);
  }

  @Override
  public void periodic() {
    switch (state) {
      case APPROACHING -> {
        loop.setNextR(table.get(poseEstimator.translationToSpeaker()).angle().in(Units.Radians));
        if (loop.getError(1) < ANGLE_ERROR_TOLERANCE.in(Units.Radians)
            && loop.getError(2) < ANGLE_SPEED_ERROR_TOLERANCE.in(Units.RadiansPerSecond)) {
          state = State.READY;
        }
      }
      case READY -> {
        loop.setNextR(table.get(poseEstimator.translationToSpeaker()).angle().in(Units.Radians));
        if (sensor.getRange() <= SHOT_FIRED_THRESHOLD.in(Millimeters)) {
          state = State.IDLE;
        }
      }
      case IDLE -> {
        loop.setNextR(IDLE_ANGLE.in(Units.Radians));
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
    APPROACHING,
    READY,
    IDLE
  }
}
