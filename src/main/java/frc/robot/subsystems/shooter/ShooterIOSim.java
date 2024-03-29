package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterIO.State.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import frc.robot.subsystems.powerBudget.PowerBudget;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ShooterIOSim extends SubsystemBase implements ShooterIO {
  private static final int LEFT_DRIVE_ID = 7;
  private static final int RIGHT_DRIVE_ID = 8;
  private static final int ACTIVATION_ID = 14;
  private static final int ACTIVATION_ID2 = 15;
  private static final PIDController pidL = new PIDController(1, 0, 0); // TODO: tune perchance
  private static final PIDController pidR = new PIDController(0.01, 0, 0); // TODO: tune perchance
  private static final SimpleMotorFeedforward feedforwardL =
      new SimpleMotorFeedforward(0.37178, 0.12017, 0.020435);
  private static final SimpleMotorFeedforward feedforwardR =
      new SimpleMotorFeedforward(0.38374, 0.12137, 0.0092311);
  private static final int CURRENT_LIMIT = 10;
  private static final double TOTAL_CURRENT_LIMIT = CURRENT_LIMIT * 2;
  private static final double ACTIVATION_SPEED = 1; // TODO: Tune
  private static final Measure<Velocity<Angle>> AMP_SPEEDL = DegreesPerSecond.of(80); // TODO: Tune
  private static final Measure<Velocity<Angle>> AMP_SPEEDR = DegreesPerSecond.of(105);
  private static final Measure<Velocity<Angle>> FIXED_SHOT_SPEEDR = RotationsPerSecond.of(75);
  private static final Measure<Velocity<Angle>> FIXED_SHOTE_SPEEDL = RotationsPerSecond.of(65);
  private static final Measure<Velocity<Angle>> ACROSS_FIELD_SPEEDR = RotationsPerSecond.of(75);

  private static final Measure<Velocity<Angle>> ACROSS_FIELD_SPEEDL = RotationsPerSecond.of(65);

  private final Drive poseEstimator;
  private final TalonFX driveL = new TalonFX(LEFT_DRIVE_ID, "can");
  private final TalonFX driveR = new TalonFX(RIGHT_DRIVE_ID, "can");
  private boolean activation = false;
  private final ShooterDataTable table;
  private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
  private final SysIdRoutine routineL;
  private final SysIdRoutine routineR;
  private final LoggedDashboardNumber numL =
      new LoggedDashboardNumber("left shooter power deg s", 0);
  private final LoggedDashboardNumber numR =
      new LoggedDashboardNumber("right shooter power deg s", 0);
  private final LoggedDashboardBoolean activationOn =
      new LoggedDashboardBoolean("activation motor toggle", false);
  private final PowerBudget power;
  @Getter private State state = TESTING;
  private final double MAX_ERROR = 4; // todo: tune

  private final LoggedDashboardNumber lP = new LoggedDashboardNumber("lP", 0.1);
  private final LoggedDashboardNumber rP = new LoggedDashboardNumber("rP", 1);

  public ShooterIOSim(ShooterDataTable table, Drive poseEstimator, PowerBudget power) {

    this.table = table;
    this.power = power;

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

    driveL.getConfigurator().apply(new TalonFXConfiguration());
    driveR.getConfigurator().apply(new TalonFXConfiguration());

    driveL.setInverted(true);

    driveL
        .getConfigurator()
        .apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(CURRENT_LIMIT));
    driveR
        .getConfigurator()
        .apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(CURRENT_LIMIT));

    //    pidL.setTolerance(MAX_ERROR - 1, 50);
    //    pidR.setTolerance(MAX_ERROR - 1, 50);
    this.poseEstimator = poseEstimator;
  }

  private void logR(SysIdRoutineLog log) {
    log.motor("right-flywheel-motor")
        .voltage(appliedVoltage.mut_replace(driveR.getMotorVoltage().getValue(), Volts))
        .angularVelocity(getWheelSpeedR())
        .angularPosition(Rotations.of(driveR.getPosition().getValue()));
  }

  private void logL(SysIdRoutineLog log) {
    log.motor("left-flywheel-motor")
        .voltage(appliedVoltage.mut_replace(driveL.getMotorVoltage().getValue(), Volts))
        .angularVelocity(getWheelSpeedL())
        .angularPosition(Rotations.of(driveL.getPosition().getValue()));
  }

  private Measure<Velocity<Angle>> getWheelSpeedL() {
    return RotationsPerSecond.of(driveL.getVelocity().getValue());
  }

  private Measure<Velocity<Angle>> getWheelSpeedR() {
    return RotationsPerSecond.of(driveR.getVelocity().getValue());
  }

  @Override
  public boolean ready() {
    return Math.abs(pidL.getPositionError()) < MAX_ERROR
        && Math.abs(pidR.getPositionError()) < MAX_ERROR
        && (state == SHOOTFIXED);
  }

  public Command sysIdQuasistaticL(SysIdRoutine.Direction direction) {
    state = SYSID;
    return routineL.quasistatic(direction);
  }

  public Command sysIdDynamicL(SysIdRoutine.Direction direction) {
    state = SYSID;
    return routineL.dynamic(direction);
  }

  public Command sysIdQuasistaticR(SysIdRoutine.Direction direction) {
    state = SYSID;
    return routineR.quasistatic(direction);
  }

  public Command sysIdDynamicR(SysIdRoutine.Direction direction) {
    state = SYSID;
    return routineR.dynamic(direction);
  }

  private double calculateLVoltage(Measure<Velocity<Angle>> setpoint) {
    return pidL.calculate(getWheelSpeedL().in(RotationsPerSecond), setpoint.in(RotationsPerSecond))
        + feedforwardL.calculate(getWheelSpeedL().in(RotationsPerSecond));
  }

  private double calculateRVoltage(Measure<Velocity<Angle>> setpoint) {
    return pidR.calculate(getWheelSpeedR().in(RotationsPerSecond), setpoint.in(RotationsPerSecond))
        + feedforwardR.calculate(setpoint.in(RotationsPerSecond));
  }

  @Override
  public void periodic() {
    switch (state) {
      case SPINUP -> {
        if (power.hasCurrent(
            driveL.getSupplyCurrent().getValue() + driveR.getSupplyCurrent().getValue(),
            TOTAL_CURRENT_LIMIT)) {

          Optional<ShooterSpec> spec = table.get(poseEstimator.translationToSpeaker());

          driveL.setVoltage(
              calculateLVoltage(spec.map(ShooterSpec::speedL).orElse(FIXED_SHOTE_SPEEDL)));

          driveR.setVoltage(
              calculateRVoltage(spec.map(ShooterSpec::speedR).orElse(FIXED_SHOT_SPEEDR)));

          activation = false;
        }
      }

      case SYSID -> activation = false;

      case TESTING -> {
        activation = activationOn.get();

        driveL.setVoltage(calculateLVoltage(RotationsPerSecond.of(numL.get())));

        driveR.setVoltage(calculateRVoltage(RotationsPerSecond.of(numR.get())));

        pidL.setPID(lP.get(), 0, 0);
        pidR.setPID(rP.get(), 0, 0);
      }

      case SHOOT -> {
        activation = true;
      }
      case INTAKE -> {}
      case SHOOTFIXED -> {
        driveL.setVoltage(calculateLVoltage(FIXED_SHOT_SPEEDR));

        driveR.setVoltage(calculateRVoltage(FIXED_SHOTE_SPEEDL));

        activation = false;
      }
      case SHOOT_ACROSS_FIELD -> {
        driveL.setVoltage(calculateLVoltage(ACROSS_FIELD_SPEEDR));

        driveR.setVoltage(calculateRVoltage(ACROSS_FIELD_SPEEDL));

        activation = false;
      }
      case AMP -> {
        activation = false;

        driveL.setVoltage(calculateLVoltage(AMP_SPEEDL));

        driveR.setVoltage(calculateRVoltage(AMP_SPEEDR));
      }
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.amps = driveL.getSupplyCurrent().getValue() + driveR.getSupplyCurrent().getValue();
    inputs.state = state;
    inputs.appliedVoltageL = driveL.getMotorVoltage().getValue();
    inputs.appliedVoltageR = driveR.getMotorVoltage().getValue();
    inputs.velocityL = getWheelSpeedL().in(RotationsPerSecond);
    inputs.velocityR = getWheelSpeedR().in(RotationsPerSecond);
    inputs.errorL = RotationsPerSecond.of(pidL.getPositionError()).in(RotationsPerSecond);
    inputs.errorR = RotationsPerSecond.of(pidR.getPositionError()).in(RotationsPerSecond);
    inputs.setPointL = RotationsPerSecond.of(pidL.getSetpoint()).in(RotationsPerSecond);
    inputs.setPointR = RotationsPerSecond.of(pidR.getSetpoint()).in(RotationsPerSecond);
    inputs.ready = ready();
    inputs.activated = activation;
  }

  @Override
  public void spinUp() {
    state = SPINUP;
  }

  @Override
  public SequentialCommandGroup sysId() {
    return sysIdQuasistaticL(SysIdRoutine.Direction.kForward)
        .andThen(new WaitCommand(0.5))
        .andThen(sysIdQuasistaticL(SysIdRoutine.Direction.kReverse))
        .andThen(new WaitCommand(0.5))
        .andThen(sysIdDynamicL(SysIdRoutine.Direction.kForward))
        .andThen(new WaitCommand(0.5))
        .andThen(sysIdDynamicL(SysIdRoutine.Direction.kReverse))
        .andThen(new WaitCommand(0.5))
        .andThen(sysIdQuasistaticR(SysIdRoutine.Direction.kForward))
        .andThen(new WaitCommand(0.5))
        .andThen(sysIdQuasistaticR(SysIdRoutine.Direction.kReverse))
        .andThen(new WaitCommand(0.5))
        .andThen(sysIdDynamicR(SysIdRoutine.Direction.kForward))
        .andThen(new WaitCommand(0.5))
        .andThen(sysIdDynamicR(SysIdRoutine.Direction.kReverse));
  }

  @Override
  public void test() {
    state = TESTING;
  }

  @Override
  public void shoot() {
    state = SHOOT;
  }

  @Override
  public void amp() {
    state = AMP;
  }

  @Override
  public void intake() {
    state = INTAKE;
  }

  @Override
  public void sysIdState() {
    state = SYSID;
  }

  @Override
  public void shootFixed() {
    state = SHOOTFIXED;
  }

  @Override
  public void shootAcrossField() {
    state = SHOOT_ACROSS_FIELD;
  }
}
