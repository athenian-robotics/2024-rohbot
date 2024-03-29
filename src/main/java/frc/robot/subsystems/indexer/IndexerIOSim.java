package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterDataTable;
import frc.robot.ShooterSpec;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.powerBudget.PowerBudget;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

// TODO: make equal to indexeriophys
public class IndexerIOSim extends SubsystemBase implements IndexerIO {
  private static final double kV = 13.3;
  private static final double kA = 0.9; // 1.3738;
  private static final double kS = 0.6971;
  private static final double kG = 0.6326;
  private static final Measure<Angle> ANGLE_STANDARD_DEVIATION = Rotations.of(10000);
  private static final Measure<Velocity<Angle>> ANGLE_SPEED_STANDARD_DEVIATION =
      RotationsPerSecond.of(1000);

  private static final Measure<Angle> ANGLE_ERROR_TOLERANCE = Degrees.of(5);
  private static final Measure<Velocity<Angle>> ANGLE_SPEED_ERROR_TOLERANCE =
      DegreesPerSecond.of(2000);

  private static final Measure<Angle> REDUCTION_RATIO = Rotations.of(1 / ((54.0 / 22) * 60));
  private static final Measure<Voltage> MAX_ANGLE_MOTOR_VOLTAGE =
      Volts.of(8); // TODO: consider if this is ass
  private static final Measure<Time> ROBOT_TIME_STEP = Seconds.of(0.02);
  private static final Measure<Angle> FLAT_ANGLE = Degrees.of(0);
  private static final int CURRENT_LIMIT = 30;
  private static final double TOTAL_CURRENT_LIMIT = CURRENT_LIMIT * 2;
  private static final Measure<Angle> IDLE_ANGLE = Degrees.of(20);
  private static final Measure<Angle> AMP_ANGLE = Degrees.of(80); // snmTODO: tune
  private static final Measure<Angle> PULSE_ANGLE = Degrees.of(11);
  private static final Measure<Angle> SHOOT_FIXED_ANGLE = Degrees.of(55);
  private static final Measure<Angle> ANGLE_ALLOTED_ERROR = Degrees.of(3);
  private final Drive poseEstimator;
  private final LinearSystemLoop<N2, N1, N1> loop;
  private final ShooterDataTable table;
  private final PowerBudget power;
  private final LoggedDashboardNumber angle = new LoggedDashboardNumber("hood angle", 0);
  private final SlewRateLimiter limiter = new SlewRateLimiter(0.06); // rot / s
  @Getter private final double UPPER_LIMIT = 30; // 40
  @Getter private final double LOWER_LIMIT = 5;
  @Getter private State state = State.TESTING;
  private double nextR = 0;
  private static final Measure<Angle> TICKS_TO_ANGLE = Rotations.of(1.0 / 11340.0);
  private static final double GEAR_RATIO = 240;

  private final DCMotor motors;
  SingleJointedArmSim sim;

  public IndexerIOSim(ShooterDataTable table, final Drive poseEstimator, PowerBudget power) {
    this.power = power;
    this.table = table;

    motors = DCMotor.getNeoVortex(2);

    LinearSystem<N2, N1, N1> sys = LinearSystemId.identifyPositionSystem(kV, kA);

    sim =
        new SingleJointedArmSim(
            sys,
            motors,
            GEAR_RATIO,
            Inches.of(20).in(Meters),
            0,
            (double) 3 / 4 * Math.PI,
            true,
            0);

    KalmanFilter<N2, N1, N1> filter =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            sys,
            VecBuilder.fill(
                ANGLE_STANDARD_DEVIATION.in(Rotations),
                ANGLE_SPEED_STANDARD_DEVIATION.in(RotationsPerSecond)),
            VecBuilder.fill(REDUCTION_RATIO.in(Rotations)),
            ROBOT_TIME_STEP.in(Seconds));

    LinearQuadraticRegulator<N2, N1, N1> controller =
        new LinearQuadraticRegulator<>(
            sys,
            VecBuilder.fill(
                ANGLE_ERROR_TOLERANCE.in(Rotations),
                ANGLE_SPEED_ERROR_TOLERANCE.in(RotationsPerSecond)),
            VecBuilder.fill(MAX_ANGLE_MOTOR_VOLTAGE.in(Volts)),
            ROBOT_TIME_STEP.in(Seconds));

    controller.latencyCompensate(sys, 0.02, 0.02);

    loop =
        new LinearSystemLoop<>(
            sys,
            controller,
            filter,
            MAX_ANGLE_MOTOR_VOLTAGE.in(Volts),
            ROBOT_TIME_STEP.in(Seconds));

    this.poseEstimator = poseEstimator;

    limiter.reset(getAngle().in(Rotations));
  }

  public Measure<Angle> getAngle() {
    return Radians.of(sim.getAngleRads());
  }

  public double getVoltage() {
    return loop.getU(0);
  }

  @Override
  public void periodic() {
    switch (state) {
      case FLAT -> nextR = FLAT_ANGLE.in(Rotations);
      case ADJUSTING -> nextR =
          table
              .get(poseEstimator.translationToSpeaker())
              .map(ShooterSpec::angle)
              .orElse(IDLE_ANGLE)
              .in(Rotations);
      case TESTING -> nextR = Degrees.of(angle.get()).in(Rotations);
      case IDLE -> nextR = IDLE_ANGLE.in(Rotations);
      case AMP_INIT -> nextR = AMP_ANGLE.in(Rotations);
      case SHOOTFIXED -> nextR = SHOOT_FIXED_ANGLE.in(Rotations);
      case AMP_PULSE -> nextR = AMP_ANGLE.in(Rotations) + PULSE_ANGLE.in(Rotations);
    }

    if (!power.hasCurrent(sim.getCurrentDrawAmps(), TOTAL_CURRENT_LIMIT)
        || state == State.SYSID
        || DriverStation.isDisabled()
        || ready()) {
      loop.reset(
          VecBuilder.fill(
              getAngle().in(Rotations),
              RadiansPerSecond.of(sim.getVelocityRadPerSec()).in(RotationsPerSecond)));
      return;
    }

    loop.setNextR(limiter.calculate(nextR), 0);
    loop.correct(VecBuilder.fill(getAngle().in(Rotations)));
    loop.predict(ROBOT_TIME_STEP.in(Seconds));
    sim.setInputVoltage(
        loop.getU(0) + kS * Math.signum(loop.getNextR(1)) + kG * Math.cos(loop.getNextR(0)));

    power.report(sim.getCurrentDrawAmps());
  }

  @Override
  public void setState(State state) {
    this.state = state;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.angle = getAngle().baseUnitMagnitude();
    inputs.appliedVoltage = getVoltage();
    inputs.state = state;
  }

  @Override
  public boolean ready() {
    return loop.getError(0) < ANGLE_ERROR_TOLERANCE.in(Units.Radians)
        && loop.getError(1) < ANGLE_SPEED_ERROR_TOLERANCE.in(Units.RadiansPerSecond);
  }
}
