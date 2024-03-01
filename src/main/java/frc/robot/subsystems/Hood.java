package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static monologue.Annotations.Log;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterDataTable;
import frc.robot.ShooterSpec;
import frc.robot.inputs.PoseEstimator;
import frc.robot.lib.TunableNumber;
import lombok.Getter;
import monologue.Annotations;
import monologue.Logged;

public class Hood extends SubsystemBase implements Logged {
  private static final int LEAD_ANGLE_MOTOR_ID = 14;
  private static final int FOLLOW_ANGLE_MOTOR_ID = 13;
  private static final double kV = 5.26; // TODO: Sysid
  private static final double kA = 0.01; // TODO: sysid
  private static final double kS = 0;
  private static final double kG = 0.09;
  private static final Measure<Angle> ANGLE_STANDARD_DEVIATION = Degrees.of(10);
  private static final Measure<Angle> ANGLE_ERROR_TOLERANCE = Degrees.of(1.4);
  private static final Measure<Velocity<Angle>> ANGLE_SPEED_STANDARD_DEVIATION =
      Degrees.of(10).per(Seconds);
  private static final Measure<Velocity<Angle>> ANGLE_SPEED_ERROR_TOLERANCE =
      Degrees.of(1.3).per(Seconds);
  private static final Measure<Angle> TICKS_TO_ANGLE = Rotations.of(1.0 / 11340.0);
  private static final Measure<Voltage> MAX_ANGLE_MOTOR_VOLTAGE = Units.Volts.of(12);
  private static final Measure<Time> ROBOT_TIME_STEP = Units.Milli(Units.Milliseconds).of(20);
  private static final Measure<Angle> FLAT_ANGLE = Degrees.of(10);
  private static final int CURRENT_LIMIT = 15;
  private static final double TOTAL_CURRENT_LIMIT = CURRENT_LIMIT * 2;
  private static final Measure<Angle> IDLE_ANGLE = Degrees.of(45);

  private final PoseEstimator poseEstimator;
  private final CANSparkMax leadAngleMotor;
  private final TimeOfFlight sensor;
  private final LinearSystemLoop<N2, N1, N1> loop;
  private final ShooterDataTable table;
  private final CANSparkMax followAngleMotor;
  @Log.NT @Getter private State state = State.FLAT;
  private final PowerBudget power;
  private final TunableNumber angle = new TunableNumber("hood angle", 0);

  // private final Rev2mDistanceSensor sensor;

  public Hood(
      ShooterDataTable table,
      final PoseEstimator poseEstimator,
      TimeOfFlight sensor,
      PowerBudget power) {
    this.sensor = sensor;
    this.power = power;
    leadAngleMotor = new CANSparkMax(LEAD_ANGLE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    leadAngleMotor.restoreFactoryDefaults();
    leadAngleMotor.setInverted(true);
    leadAngleMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    followAngleMotor =
        new CANSparkMax(FOLLOW_ANGLE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    followAngleMotor.restoreFactoryDefaults();
    followAngleMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    followAngleMotor.follow(leadAngleMotor, true);
    this.table = table;

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

    leadAngleMotor.getEncoder().setPosition(0.0);
    followAngleMotor.getEncoder().setPosition(0.0);
    leadAngleMotor
        .getEncoder()
        .setPositionConversionFactor(42.0); // Causes encoder to output in ticks, not rotations
    followAngleMotor.getEncoder().setPositionConversionFactor(42.0);
  }

  @Log.NT
  public double getAngle() {
    return leadAngleMotor.getEncoder().getPosition() * TICKS_TO_ANGLE.in(Degrees);
  }

  @Log.NT
  public double getVoltage() {
    return loop.getU(0);
  }

  @Override
  public void periodic() {
    switch (state) {
      case FLAT -> loop.setNextR(FLAT_ANGLE.in(Radians), 0);
      case ADJUSTING -> loop.setNextR(
          table
              .get(poseEstimator.translationToSpeaker())
              .map(ShooterSpec::angle)
              .orElse(Degrees.of(0))
              .in(Units.Radians),
          0);
      case TESTING -> loop.setNextR(Degrees.of(angle.get()).in(Radians), 0);
      case IDLE -> loop.setNextR(IDLE_ANGLE.in(Radians), 0);
    }

    if (power.hasCurrent(
        leadAngleMotor.getOutputCurrent() + followAngleMotor.getOutputCurrent(),
        TOTAL_CURRENT_LIMIT)) {
      loop.correct(
          VecBuilder.fill(
              leadAngleMotor.getEncoder().getPosition() * TICKS_TO_ANGLE.in(Units.Radians)));
      loop.predict(ROBOT_TIME_STEP.in(Units.Seconds));
      leadAngleMotor.setVoltage(
          loop.getU(0) + kS * Math.signum(loop.getNextR(1) + kG * Math.cos(loop.getNextR(0))));
    }

    SmartDashboard.putNumber("hood sensor", sensor.getRange());
    SmartDashboard.putNumber("sys volts hood", getVoltage());
    SmartDashboard.putNumber("hood angle", getAngle());
    SmartDashboard.putNumber("hood error", loop.getError(0));
  }

  public void flat() {
    state = State.FLAT;
  }

  public void adjusting() {
    state = State.ADJUSTING;
  }

  public boolean ready() {
    return loop.getError(0) < ANGLE_ERROR_TOLERANCE.in(Units.Radians)
        && loop.getError(1) < ANGLE_SPEED_ERROR_TOLERANCE.in(Units.RadiansPerSecond);
  }

  public void idle() {
    state = State.IDLE;
  }

  private enum State {
    FLAT,
    ADJUSTING,
    IDLE,
    TESTING
  }
}
