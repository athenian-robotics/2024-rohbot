package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterIO.State.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ShooterDataTable;
import frc.robot.ShooterSpec;
import frc.robot.lib.SimpleVelocitySystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.powerBudget.PowerBudgetPhysical;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class ShooterIOPhysicalLQR extends SubsystemBase implements ShooterIO {
  static final double AMP_SPEED = DegreesPerSecond.of(100).in(RadiansPerSecond); // TODO: Tune
  private static final int LEFT_DRIVE_ID = 7;
  private static final int RIGHT_DRIVE_ID = 8;
  private static final int ACTIVATION_ID = 14;
  private static final int ACTIVATION_ID2 = 15;
  private static final Measure<Voltage> kS = Volts.of(.01);
  private static final Measure<Per<Voltage, Velocity<Angle>>> kV = VoltsPerRadianPerSecond.of(7.48);
  private static final Measure<Per<Voltage, Velocity<Velocity<Angle>>>> kA =
      VoltsPerRadianPerSecondSquared.of(9.25);
  private static final double MAX_ERROR = 20;
  private static final double MAX_CONTROL_EFFORT = 8;
  private static final double MODEL_DEVIATION = 1;
  private static final double ENCODER_DEVIATION =
      (double) 1 / 2048 * 2 * 3.14; // 1 tick of built-in neo encoder with reduction
  private static final Measure<Time> LOOP_TIME = Second.of(0.02);
  private static final int CURRENT_LIMIT = 5;
  private static final double TOTAL_CURRENT_LIMIT = CURRENT_LIMIT * 3;
  private static final double ACTIVATION_SPEED = 1;
  private static final Measure<Velocity<Angle>> FIXED_SPEAKER_SPEED = RadiansPerSecond.of(500);

  private final Drive poseEstimator;
  private final TalonFX driveL = new TalonFX(LEFT_DRIVE_ID, "can");
  private final TalonFX driveR = new TalonFX(RIGHT_DRIVE_ID, "can");
  private final CANSparkMax activation;
  private final CANSparkMax activation2;
  private final SimpleVelocitySystem sysL;
  private final SimpleVelocitySystem sysR;
  private final ShooterDataTable table;
  private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Velocity<Angle>> velocity = mutable(Rotations.per(Second).of(0));
  private final SysIdRoutine routineL;
  private final SysIdRoutine routineR;
  private final LoggedDashboardNumber numL =
      new LoggedDashboardNumber("left shooter power deg s", 0);
  private final LoggedDashboardNumber numR =
      new LoggedDashboardNumber("right shooter power deg s", 0);
  private final LoggedDashboardNumber numActivation =
      new LoggedDashboardNumber("activation motor power percent 0 to 1", 0);
  private final PowerBudgetPhysical power;
  @Getter private State state = TESTING;

  public ShooterIOPhysicalLQR(
      ShooterDataTable table, Drive poseEstimator, PowerBudgetPhysical power) {

    this.table = table;
    this.power = power;

    sysL =
        new SimpleVelocitySystem(
            kS.in(Volts),
            kV.in(VoltsPerRadianPerSecond),
            kA.in(VoltsPerRadianPerSecondSquared),
            MAX_ERROR,
            MAX_CONTROL_EFFORT,
            MODEL_DEVIATION,
            ENCODER_DEVIATION,
            LOOP_TIME.in(Seconds));
    sysR =
        new SimpleVelocitySystem(
            kS.in(Volts),
            kV.in(VoltsPerRadianPerSecond),
            kA.in(VoltsPerRadianPerSecondSquared),
            MAX_ERROR,
            MAX_CONTROL_EFFORT,
            MODEL_DEVIATION,
            ENCODER_DEVIATION,
            LOOP_TIME.in(Seconds));
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

    driveL.setInverted(true);

    activation = new CANSparkMax(ACTIVATION_ID, CANSparkLowLevel.MotorType.kBrushless);
    activation.restoreFactoryDefaults();
    activation.setInverted(false);

    activation2 = new CANSparkMax(ACTIVATION_ID2, CANSparkLowLevel.MotorType.kBrushless);
    activation2.restoreFactoryDefaults();
    activation2.setInverted(false);
    activation2.follow(activation, true);

    activation.burnFlash();
    activation2.burnFlash();
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
    return Units.RotationsPerSecond.of(driveL.getVelocity().getValue());
  }

  private Measure<Velocity<Angle>> getWheelSpeedR() {
    return Units.RotationsPerSecond.of(driveR.getVelocity().getValue());
  }

  @Override
  public boolean ready() {
    return getWheelSpeedL().in(RadiansPerSecond) > 185;
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
            TOTAL_CURRENT_LIMIT * 3 / 2)) {
          sysL.update(getWheelSpeedL().in(RadiansPerSecond));
          sysR.update(getWheelSpeedR().in(RadiansPerSecond));

          Optional<ShooterSpec> spec = table.get(poseEstimator.translationToSpeaker());

          sysL.set(
              spec.map(ShooterSpec::speedL).orElse(DegreesPerSecond.of(0)).in(RadiansPerSecond));
          sysR.set(
              spec.map(ShooterSpec::speedR).orElse(DegreesPerSecond.of(0)).in(RadiansPerSecond));

          driveL.set(sysL.getOutput());
          driveR.set(sysR.getOutput());
          activation.set(0);
        }
      }
      case SYSID -> {
        activation.set(0);
        // chill            .andThen(new WaitCommand(0.5))
      }
      case TESTING -> {
        sysL.update(getWheelSpeedL().in(RadiansPerSecond)); // Returns RPS
        sysR.update(getWheelSpeedR().in(RadiansPerSecond));

        sysL.set(DegreesPerSecond.of(numL.get()).in(RadiansPerSecond));
        sysR.set(DegreesPerSecond.of(numR.get()).in(RadiansPerSecond));

        activation.setVoltage(numActivation.get());
        driveL.set(sysL.getOutput());
        driveR.set(sysR.getOutput());
      }
      case SHOOT -> {
        if (!ready()) break;
        sysL.update(getWheelSpeedL().in(RadiansPerSecond)); // Returns RPS
        sysR.update(getWheelSpeedR().in(RadiansPerSecond));

        driveL.set(sysL.getOutput());
        driveR.set(sysR.getOutput());

        activation.set(ACTIVATION_SPEED);
      }
      case AMP -> {
        activation.set(0);
        sysL.update(getWheelSpeedL().in(RadiansPerSecond)); // Returns RPS
        sysR.update(getWheelSpeedR().in(RadiansPerSecond));

        sysL.set(AMP_SPEED * 0.8);
        sysR.set(AMP_SPEED);

        driveL.set(sysR.getOutput());
        driveR.set(sysR.getOutput());
      }
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.amps = driveL.getSupplyCurrent().getValue() + driveR.getSupplyCurrent().getValue();
    inputs.state = state;
    inputs.appliedVoltageL = sysL.getOutput();
    inputs.appliedVoltageR = sysR.getOutput();
    inputs.velocityL = getWheelSpeedL();
    inputs.velocityR = getWheelSpeedR();
    inputs.ready = ready();
  }

  @Override
  public void spinUp() {
    state = SPINUP;
  }

  public void pulse() {
    runOnce(
            () -> {
              activation.set(ACTIVATION_SPEED);
            })
        .andThen(new WaitCommand(0.5))
        .andThen(
            () -> {
              activation.set(0);
            })
        .schedule();
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
    runOnce(() -> state = TESTING);
  }

  @Override
  public void shoot() {
    state = SHOOT;
  }

  @Override
  public void amp() {
    state = AMP;
  }
}
