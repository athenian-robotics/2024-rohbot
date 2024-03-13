package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterIO.State.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
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
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ShooterIOPhysicalPID extends SubsystemBase implements ShooterIO {
  private static final int LEFT_DRIVE_ID = 7;
  private static final int RIGHT_DRIVE_ID = 8;
  private static final int ACTIVATION_ID = 14;
  private static final int ACTIVATION_ID2 = 15;
  private static final PIDController pidL = new PIDController(6.2, 0, 0); // TODO: tune perchance
  private static final PIDController pidR = new PIDController(0, 0, 0); // TODO: tune perchance
  private static final int CURRENT_LIMIT = 10;
  private static final double TOTAL_CURRENT_LIMIT = CURRENT_LIMIT * 2;
  private static final double ACTIVATION_SPEED = 1; // TODO: Tune
  private static final Measure<Velocity<Angle>> AMP_SPEED = DegreesPerSecond.of(0); // TODO: Tune

  private final Drive poseEstimator;
  private final TalonFX driveL = new TalonFX(LEFT_DRIVE_ID, "can");
  private final TalonFX driveR = new TalonFX(RIGHT_DRIVE_ID, "can");
  private final CANSparkMax activation;
  private final CANSparkMax activation2;
  private final ShooterDataTable table;
  private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Velocity<Angle>> velocity = mutable(Rotations.per(Second).of(0));
  private final SysIdRoutine routineL;
  private final SysIdRoutine routineR;
  private final LoggedDashboardNumber numL =
      new LoggedDashboardNumber("left shooter power deg s", 0);
  private final LoggedDashboardNumber numR =
      new LoggedDashboardNumber("right shooter power deg s", 0);
  private final LoggedDashboardBoolean activationOn =
      new LoggedDashboardBoolean("activation motor toggle", false);
  private final PowerBudgetPhysical power;
  @Getter private State state = TESTING;
  private final double MAX_ERROR = 0.1; // todo: tune

  private final LoggedDashboardNumber lP = new LoggedDashboardNumber("lP", 0);
  private final LoggedDashboardNumber lD = new LoggedDashboardNumber("lD", 0);

  private final LoggedDashboardNumber rP = new LoggedDashboardNumber("rP", 0);
  private final LoggedDashboardNumber rD = new LoggedDashboardNumber("rD", 0);
  private final LoggedDashboardNumber lkS = new LoggedDashboardNumber("lKs", 0);
  private final LoggedDashboardNumber rkS = new LoggedDashboardNumber("rKs", 0);

  public ShooterIOPhysicalPID(
      ShooterDataTable table, Drive poseEstimator, PowerBudgetPhysical power) {

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

    driveL
        .getConfigurator()
        .apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(CURRENT_LIMIT));
    driveL
        .getConfigurator()
        .apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(CURRENT_LIMIT));

    activation = new CANSparkMax(ACTIVATION_ID, CANSparkLowLevel.MotorType.kBrushless);
    activation.restoreFactoryDefaults();
    activation.setInverted(false);

    activation2 = new CANSparkMax(ACTIVATION_ID2, CANSparkLowLevel.MotorType.kBrushless);
    activation2.restoreFactoryDefaults();
    activation2.setInverted(false);
    activation2.follow(activation, true);

    activation.burnFlash();
    activation2.burnFlash();

    pidL.setTolerance(0.5, 1);
    pidR.setTolerance(0.5, 1);
    this.poseEstimator = poseEstimator;
  }

  private void logR(SysIdRoutineLog log) {
    log.motor("right-flywheel-motor")
        .voltage(appliedVoltage.mut_replace(driveR.getMotorVoltage().getValue(), Volts))
        .angularVelocity(
            velocity.mut_replace(driveR.getVelocity().getValue(), Rotations.per(Second)))
        .angularPosition(Rotations.of(driveR.getPosition().getValue()));
  }

  private void logL(SysIdRoutineLog log) {
    log.motor("left-flywheel-motor")
        .voltage(appliedVoltage.mut_replace(driveL.getMotorVoltage().getValue(), Volts))
        .angularVelocity(
            velocity.mut_replace(driveL.getVelocity().getValue(), Rotations.per(Second)))
        .angularPosition(Rotations.of(driveL.getPosition().getValue()));
  }

  private Measure<Velocity<Angle>> getWheelSpeedL() {
    return RPM.of(driveL.getVelocity().getValue());
  }

  private Measure<Velocity<Angle>> getWheelSpeedR() {
    return RPM.of(driveR.getVelocity().getValue());
  }

  @Override
  public boolean ready() {
    return Math.abs(pidL.getVelocityError()) < MAX_ERROR
        && Math.abs(pidR.getVelocityError()) < MAX_ERROR;
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

  @Override
  public void periodic() {
    switch (state) {
      case SPINUP -> {
        if (power.hasCurrent(
            driveL.getSupplyCurrent().getValue() + driveR.getSupplyCurrent().getValue(),
            TOTAL_CURRENT_LIMIT)) {

          Optional<ShooterSpec> spec = table.get(poseEstimator.translationToSpeaker());

          driveL.setVoltage(
              pidL.calculate(
                  getWheelSpeedL().in(RadiansPerSecond),
                  spec.map(ShooterSpec::speedL)
                      .orElse(DegreesPerSecond.of(0))
                      .in(RadiansPerSecond)));

          driveR.setVoltage(
              pidR.calculate(
                  getWheelSpeedR().in(RadiansPerSecond),
                  spec.map(ShooterSpec::speedR)
                      .orElse(DegreesPerSecond.of(0))
                      .in(RadiansPerSecond)));

          activation.set(0);
        }
      }

      case SYSID -> activation.set(0);

      case TESTING -> {
        if (activationOn.get()) activation.set(ACTIVATION_SPEED);
        else activation.set(0);

        driveL.setVoltage(
            pidL.calculate(
                    getWheelSpeedL().in(RadiansPerSecond),
                    DegreesPerSecond.of(numL.get()).in(RadiansPerSecond))
                + lkS.get() * Math.signum(pidL.getSetpoint()));

        driveR.setVoltage(
            pidR.calculate(
                    getWheelSpeedR().in(RadiansPerSecond),
                    DegreesPerSecond.of(numR.get()).in(RadiansPerSecond))
                + rkS.get() * Math.signum(pidR.getSetpoint()));

        pidL.setPID(lP.get(), 0, lD.get());
        pidR.setPID(rP.get(), 0, rD.get());
      }

      case SHOOT -> {
        if (!ready()) break;

        Optional<ShooterSpec> spec = table.get(poseEstimator.translationToSpeaker());

        driveL.setVoltage(
            spec.map(ShooterSpec::speedL).orElse(DegreesPerSecond.of(0)).in(RadiansPerSecond));
        driveR.setVoltage(
            spec.map(ShooterSpec::speedR).orElse(DegreesPerSecond.of(0)).in(RadiansPerSecond));

        activation.set(ACTIVATION_SPEED);
      }
      case INTAKE -> activation.set(ACTIVATION_SPEED);
      case AMP -> {
        activation.set(0);

        driveL.setVoltage(
            pidL.calculate(getWheelSpeedL().in(RadiansPerSecond), AMP_SPEED.in(RadiansPerSecond)));

        driveR.setVoltage(
            pidR.calculate(getWheelSpeedR().in(RadiansPerSecond), AMP_SPEED.in(RadiansPerSecond)));
      }
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.amps = driveL.getSupplyCurrent().getValue() + driveR.getSupplyCurrent().getValue();
    inputs.state = state;
    inputs.appliedVoltageL = driveL.getMotorVoltage().getValue();
    inputs.appliedVoltageR = driveR.getMotorVoltage().getValue();
    inputs.velocityL = getWheelSpeedL();
    inputs.velocityR = getWheelSpeedR();
    inputs.errorL = pidL.getVelocityError();
    inputs.errorR = pidR.getVelocityError();
    inputs.setPointL = pidL.getSetpoint();
    inputs.setPointR = pidR.getSetpoint();
    inputs.ready = ready();
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
}
