package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ShooterDataTable;
import frc.robot.ShooterSpec;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.powerBudget.PowerBudgetPhysical;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class IndexerIOPhysical extends SubsystemBase implements IndexerIO {
  private static final int LEAD_ANGLE_MOTOR_ID = 10;
  private static final int FOLLOW_ANGLE_MOTOR_ID = 11;
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
  private final CANSparkFlex leadMotor;
  private final LinearSystemLoop<N2, N1, N1> loop;
  private final ShooterDataTable table;
  private final CANSparkFlex followMotor;
  private final PowerBudgetPhysical power;
  private final LoggedDashboardNumber angle = new LoggedDashboardNumber("hood angle", 0);
  private final SysIdRoutine routine;
  private final SlewRateLimiter limiter = new SlewRateLimiter(0.06); // rot / s
  @Getter private final double UPPER_LIMIT = 30; // 40
  @Getter private final double LOWER_LIMIT = 5;
  @Getter private State state = State.TESTING;
  private double nextR = 0;

  public IndexerIOPhysical(
      ShooterDataTable table, final Drive poseEstimator, PowerBudgetPhysical power) {
    this.power = power;
    leadMotor = new CANSparkFlex(LEAD_ANGLE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    leadMotor.restoreFactoryDefaults();
    leadMotor.setInverted(false);
    leadMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    leadMotor.enableVoltageCompensation(12.8);

    followMotor = new CANSparkFlex(FOLLOW_ANGLE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    followMotor.restoreFactoryDefaults();
    followMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    followMotor.follow(leadMotor, true);
    followMotor.enableVoltageCompensation(12.8);

    this.table = table;

    LinearSystem<N2, N1, N1> sys = LinearSystemId.identifyPositionSystem(kV, kA);

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

    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Seconds).of(8), Volts.of(6), Seconds.of(2)),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {
                  Logger.recordOutput("volts hood sysid", volts.in(Volts));
                  leadMotor.setVoltage(volts.in(Volts));
                },
                this::log,
                this,
                "hood-motor"));

    this.poseEstimator = poseEstimator;

    leadMotor.getEncoder().setPosition(0.0);
    followMotor.getEncoder().setPosition(0.0);

    leadMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    followMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

    Timer.delay(.1);

    leadMotor.burnFlash();
    followMotor.burnFlash();

    limiter.reset(getAngle().in(Rotations));
  }

  private void log(SysIdRoutineLog sysIdRoutineLog) {
    sysIdRoutineLog
        .motor("hood-motor")
        .voltage(Volts.of(leadMotor.getAppliedOutput() * leadMotor.getBusVoltage()))
        .angularPosition(getAngle())
        .angularVelocity(getVelocity());
  }

  public Measure<Angle> getAngle() {
    return Rotations.of(leadMotor.getEncoder().getPosition() * REDUCTION_RATIO.in(Rotations));
  }

  public double getVoltage() {
    return leadMotor.getBusVoltage() * leadMotor.getAppliedOutput();
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

    if (!power.hasCurrent(
            leadMotor.getOutputCurrent() + followMotor.getOutputCurrent(), TOTAL_CURRENT_LIMIT)
        || state == State.SYSID
        || DriverStation.isDisabled()
        || ready()) {
      loop.reset(VecBuilder.fill(getAngle().in(Rotations), getVelocity().in(RotationsPerSecond)));
      return;
    }

    loop.setNextR(limiter.calculate(nextR), 0);
    loop.correct(VecBuilder.fill(getAngle().in(Rotations)));
    loop.predict(ROBOT_TIME_STEP.in(Seconds));
    leadMotor.setVoltage(
        loop.getU(0) + kS * Math.signum(loop.getNextR(1)) + kG * Math.cos(loop.getNextR(0)));
  }

  @Override
  public void setState(State state) {
    this.state = state;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.amps = leadMotor.getOutputCurrent() + followMotor.getOutputCurrent();
    inputs.angle = getAngle().in(Degrees);
    inputs.velocity = getVelocity().in(DegreesPerSecond);
    inputs.appliedVoltage = getVoltage();
    inputs.state = state;
    inputs.inBounds = upperBound() && lowerBound();
    inputs.upBound = upperBound();
    inputs.lowBound = lowerBound();
    inputs.limiterValue = limiter.lastValue();
    inputs.ready = ready();
    inputs.error = Rotations.of(loop.getError(0)).in(Degrees);
    inputs.errorVelo = RotationsPerSecond.of(loop.getError(1)).in(DegreesPerSecond);
    inputs.hasCurrent =
        power.hasCurrent(
            leadMotor.getOutputCurrent() + followMotor.getOutputCurrent(), TOTAL_CURRENT_LIMIT);
  }

  private Measure<Velocity<Angle>> getVelocity() {
    return Rotations.per(Minutes)
        .of(leadMotor.getEncoder().getVelocity() * REDUCTION_RATIO.in(Rotations));
  }

  @Override
  public Command zero() {
    return runOnce(() -> leadMotor.getEncoder().setPosition(0));
  }

  @Override
  public SequentialCommandGroup sysId() {
    state = State.SYSID;
    return routine
        .quasistatic(SysIdRoutine.Direction.kForward)
        .onlyWhile(this::upperBound)
        .andThen(new WaitCommand(.5))
        .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse).onlyWhile(this::lowerBound))
        .andThen(new WaitCommand(.5))
        .andThen(routine.dynamic(SysIdRoutine.Direction.kForward).onlyWhile(this::upperBound))
        .andThen(new WaitCommand(.5))
        .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse).onlyWhile(this::lowerBound));
  }

  private boolean upperBound() {
    return leadMotor.getEncoder().getPosition() < UPPER_LIMIT;
  }

  private boolean lowerBound() {
    return leadMotor.getEncoder().getPosition() > LOWER_LIMIT;
  }

  @Override
  public boolean ready() {
    return Math.abs(getAngle().minus(Rotations.of(nextR)).in(Rotations))
            < ANGLE_ALLOTED_ERROR.in(Rotations)
        && nextR == SHOOT_FIXED_ANGLE.in(Rotations);
  }
}
