package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
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
  private static final double kV = 12.335;
  private static final double kA = 0.91085;
  private static final double kS = 0; // 1.0203
  private static final double kG = 0; // 1.71;
  private static final Measure<Angle> ANGLE_STANDARD_DEVIATION = Rotations.of(100);
  private static final Measure<Angle> ANGLE_ERROR_TOLERANCE = Degrees.of(2);
  private static final Measure<Velocity<Angle>> ANGLE_SPEED_STANDARD_DEVIATION =
      RotationsPerSecond.of(100);
  private static final Measure<Velocity<Angle>> ANGLE_SPEED_ERROR_TOLERANCE =
      RotationsPerSecond.of(2);
  private static final Measure<Angle> ROT_TO_ANGLE = Rotations.of(1 / ((54.0 / 22) * 60));
  private static final Measure<Voltage> MAX_ANGLE_MOTOR_VOLTAGE = Units.Volts.of(12);
  private static final Measure<Time> ROBOT_TIME_STEP = Seconds.of(0.02);
  private static final Measure<Angle> FLAT_ANGLE = Degrees.of(10);
  private static final int CURRENT_LIMIT = 30;
  private static final double TOTAL_CURRENT_LIMIT = CURRENT_LIMIT * 2;
  private static final Measure<Angle> IDLE_ANGLE = Degrees.of(45);
  private static final Measure<Angle> AMP_ANGLE = Degrees.of(95); // TODO: tunes;

  private final Drive poseEstimator;
  private final CANSparkFlex leadAngleMotor;
  private final LinearSystemLoop<N2, N1, N1> loop;
  private final ShooterDataTable table;
  private final CANSparkFlex followAngleMotor;
  private final PowerBudgetPhysical power;
  private final LoggedDashboardNumber angle = new LoggedDashboardNumber("hood angle", 0);
  private final SysIdRoutine routine;
  @Getter private State state = State.TESTING;

  @Getter private double UPPER_LIMIT = 10; // 40
  @Getter private double LOWER_LIMIT = 5;

  public IndexerIOPhysical(
      ShooterDataTable table, final Drive poseEstimator, PowerBudgetPhysical power) {
    this.power = power;
    leadAngleMotor = new CANSparkFlex(LEAD_ANGLE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    leadAngleMotor.restoreFactoryDefaults();
    leadAngleMotor.setInverted(false);
    leadAngleMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    followAngleMotor =
        new CANSparkFlex(FOLLOW_ANGLE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
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
                ANGLE_STANDARD_DEVIATION.in(Rotations),
                ANGLE_SPEED_STANDARD_DEVIATION.in(RotationsPerSecond)),
            VecBuilder.fill(ROT_TO_ANGLE.in(Rotations)),
            ROBOT_TIME_STEP.in(Units.Seconds));

    LinearQuadraticRegulator<N2, N1, N1> controller =
        new LinearQuadraticRegulator<>(
            sys,
            VecBuilder.fill(
                ANGLE_ERROR_TOLERANCE.in(Rotations),
                ANGLE_SPEED_ERROR_TOLERANCE.in(RotationsPerSecond)),
            VecBuilder.fill(MAX_ANGLE_MOTOR_VOLTAGE.in(Units.Volts)),
            ROBOT_TIME_STEP.in(Units.Seconds));

    loop =
        new LinearSystemLoop<>(
            sys,
            controller,
            filter,
            MAX_ANGLE_MOTOR_VOLTAGE.in(Units.Volts),
            ROBOT_TIME_STEP.in(Units.Seconds));

    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Seconds).of(8), Volts.of(6), Seconds.of(2)),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {
                  Logger.recordOutput("volts hood sysid", volts.in(Volts));
                  leadAngleMotor.setVoltage(volts.in(Volts));
                },
                this::log,
                this,
                "hood-motor"));

    this.poseEstimator = poseEstimator;

    leadAngleMotor.getEncoder().setPosition(0.0);
    followAngleMotor.getEncoder().setPosition(0.0);
    leadAngleMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    followAngleMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    leadAngleMotor.burnFlash();
    followAngleMotor.burnFlash();
  }

  private void log(SysIdRoutineLog sysIdRoutineLog) {
    sysIdRoutineLog
        .motor("hood-motor")
        .voltage(Volts.of(leadAngleMotor.getAppliedOutput() * leadAngleMotor.getBusVoltage()))
        .angularPosition(getAngle())
        .angularVelocity(getVelocity());
  }

  public Measure<Angle> getAngle() {
    return Degrees.of(leadAngleMotor.getEncoder().getPosition() * ROT_TO_ANGLE.in(Degrees));
  }

  public double getVoltage() {
    return leadAngleMotor.getBusVoltage() * leadAngleMotor.getAppliedOutput();
  }

  public void periodic() {
    switch (state) {
      case FLAT -> loop.setNextR(FLAT_ANGLE.in(Rotations), 0);
      case ADJUSTING -> loop.setNextR(
          table
              .get(poseEstimator.translationToSpeaker())
              .map(ShooterSpec::angle)
              .orElse(IDLE_ANGLE)
              .in(Rotations),
          0);
      case TESTING -> loop.setNextR(Degrees.of(angle.get()).in(Rotations), 0);
      case IDLE -> loop.setNextR(IDLE_ANGLE.in(Rotations), 0);
      case SYSID -> {}
      case AMP -> loop.setNextR(AMP_ANGLE.in(Rotations), 0);
    }

    loop.correct(VecBuilder.fill(getAngle().in(Rotations)));

    if (power.hasCurrent(
            leadAngleMotor.getOutputCurrent() + followAngleMotor.getOutputCurrent(),
            TOTAL_CURRENT_LIMIT)
        && state != State.SYSID) {
      loop.predict(ROBOT_TIME_STEP.in(Units.Seconds));
      //      leadAngleMotor.setVoltage(
      //          loop.getU(0) + kS * Math.signum(loop.getNextR(1)) + kG *
      // Math.cos(loop.getNextR(0)));
    }
  }

  @Override
  public void setState(State state) {
    this.state = state;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.angle = getAngle().in(Degrees);
    inputs.velocity = getVelocity().in(DegreesPerSecond);
    inputs.appliedVoltage = getVoltage();
    inputs.state = state;
    inputs.inBounds = upperBound() && lowerBound();
    inputs.upBound = upperBound();
    inputs.lowBound = lowerBound();
    inputs.error = Rotations.of(loop.getError(0)).in(Degrees);
    inputs.errorVelo = RotationsPerSecond.of(loop.getError(1)).in(DegreesPerSecond);
  }

  private Measure<Velocity<Angle>> getVelocity() {
    return Rotations.per(Minutes)
        .of(leadAngleMotor.getEncoder().getVelocity() * ROT_TO_ANGLE.in(Rotations));
  }

  @Override
  public Command zero() {
    return runOnce(() -> leadAngleMotor.getEncoder().setPosition(0));
  }

  @Override
  public SequentialCommandGroup sysId() {
    state = State.SYSID;
    return routine
        .quasistatic(SysIdRoutine.Direction.kForward)
        .onlyWhile((this::upperBound))
        .andThen(new WaitCommand(.5))
        .andThen(routine.quasistatic(SysIdRoutine.Direction.kReverse).onlyWhile(this::lowerBound))
        .andThen(new WaitCommand(.5))
        .andThen(routine.dynamic(SysIdRoutine.Direction.kForward).onlyWhile(this::upperBound))
        .andThen(new WaitCommand(.5))
        .andThen(routine.dynamic(SysIdRoutine.Direction.kReverse).onlyWhile(this::lowerBound));
  }

  private boolean upperBound() {
    return leadAngleMotor.getEncoder().getPosition() < UPPER_LIMIT;
  }

  private boolean lowerBound() {
    return leadAngleMotor.getEncoder().getPosition() > LOWER_LIMIT;
  }

  public boolean ready() {
    return loop.getError(0) < ANGLE_ERROR_TOLERANCE.in(Rotations)
        && loop.getError(1) < ANGLE_SPEED_ERROR_TOLERANCE.in(RotationsPerSecond);
  }
}
