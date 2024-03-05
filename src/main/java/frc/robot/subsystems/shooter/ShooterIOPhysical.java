package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ShooterDataTable;
import frc.robot.ShooterSpec;
import frc.robot.inputs.PoseEstimator;
import frc.robot.lib.SimpleVelocitySystem;
import frc.robot.subsystems.powerBudget.PowerBudgetPhysical;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.Optional;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterIO.State.*;

public class ShooterIOPhysical extends SubsystemBase implements ShooterIO {
    private static final int LEFT_DRIVE_ID = 11;
    private static final int RIGHT_DRIVE_ID = 12;
    private static final int ACTIVATION_ID = -1; // TODO: fill
    private static final Measure<Voltage> kS = Volts.of(.01);
    private static final Measure<Per<Voltage, Velocity<Angle>>> kV = VoltsPerRadianPerSecond.of(.087);
    private static final Measure<Per<Voltage, Velocity<Velocity<Angle>>>> kA =
            VoltsPerRadianPerSecondSquared.of(0.06);
    private static final double MAX_ERROR = 1;
    private static final double MAX_CONTROL_EFFORT = 8;
    private static final double MODEL_DEVIATION = 1;
    private static final double ENCODER_DEVIATION =
            1 / 42.0; // 1 tick of built-in neo encoder with reduction
    private static final Measure<Time> LOOP_TIME = Second.of(0.02);
    private static final int CURRENT_LIMIT = 20;
    private static final double TOTAL_CURRENT_LIMIT = CURRENT_LIMIT * 3;
    private static final double ACTIVATION_SPEED = 0.1; // TODO: Tune

    private final PoseEstimator poseEstimator;
    private final CANSparkMax driveL =
            new CANSparkMax(LEFT_DRIVE_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax driveR =
            new CANSparkMax(RIGHT_DRIVE_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax activation =
            new CANSparkMax(ACTIVATION_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final SimpleVelocitySystem sysL;
    private final SimpleVelocitySystem sysR;
    private final ShooterDataTable table;
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(Rotations.per(Second).of(0));
    private final SysIdRoutine routineL;
    private final SysIdRoutine routineR;
    private final LoggedDashboardNumber numL = new LoggedDashboardNumber("left shooter power deg/s", 0);
    private final LoggedDashboardNumber numR = new LoggedDashboardNumber("right shooter power deg/s", 0);
    private final LoggedDashboardNumber numActivation = new LoggedDashboardNumber("activation motor power percent 0 to 1", 0);
    private final PowerBudgetPhysical power;
    @Getter
    private State state = SPINUP;

    public ShooterIOPhysical(
            ShooterDataTable table, PoseEstimator poseEstimator, PowerBudgetPhysical power) {

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

        driveL.setSmartCurrentLimit(CURRENT_LIMIT);
        driveR.setSmartCurrentLimit(CURRENT_LIMIT);
        activation.setSmartCurrentLimit(CURRENT_LIMIT);
        this.poseEstimator = poseEstimator;
    }

    private void logR(SysIdRoutineLog log) {
        log.motor("right-flywheel-motor")
                .voltage(appliedVoltage.mut_replace(driveL.getBusVoltage() * driveL.get(), Volts))
                .angularVelocity(velocity.mut_replace(driveL.getBusVoltage(), Rotations.per(Second)));
    }

    private void logL(SysIdRoutineLog log) {
        log.motor("left-flywheel-motor")
                .voltage(appliedVoltage.mut_replace(driveR.getBusVoltage() * driveR.get(), Volts))
                .angularVelocity(velocity.mut_replace(driveR.getBusVoltage(), Rotations.per(Second)));
    }

    private Measure<Velocity<Angle>> getWheelSpeedL() {
        return Units.RotationsPerSecond.of(driveL.getEncoder().getVelocity());
    }

    private Measure<Velocity<Angle>> getWheelSpeedR() {
        return Units.RotationsPerSecond.of(driveR.getEncoder().getVelocity());
    }

    @Override
    public boolean ready() {
        return Math.abs(sysL.getError()) < MAX_ERROR && Math.abs(sysR.getError()) < MAX_ERROR;
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
                        driveL.getOutputCurrent() + driveR.getOutputCurrent(), TOTAL_CURRENT_LIMIT * 3 / 2)) {
                    sysL.update(getWheelSpeedL().in(RadiansPerSecond)); // Returns RPS
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
                // chill
            }
            case TESTING -> {
                sysL.update(getWheelSpeedL().in(RadiansPerSecond)); // Returns RPS
                sysR.update(getWheelSpeedR().in(RadiansPerSecond));

                sysL.set(DegreesPerSecond.of(numL.get()).in(RadiansPerSecond));
                sysR.set(DegreesPerSecond.of(numR.get()).in(RadiansPerSecond));

                activation.set(numActivation.get());
                driveL.set(sysL.getOutput());
                driveR.set(sysR.getOutput());
            }
            case SHOOT -> {
                    if (!ready()) break;
                    sysL.update(getWheelSpeedL().in(RadiansPerSecond)); // Returns RPS
                    sysR.update(getWheelSpeedR().in(RadiansPerSecond));

                    Optional<ShooterSpec> spec = table.get(poseEstimator.translationToSpeaker());

                    sysL.set(
                            spec.map(ShooterSpec::speedL).orElse(DegreesPerSecond.of(0)).in(RadiansPerSecond));
                    sysR.set(
                            spec.map(ShooterSpec::speedR).orElse(DegreesPerSecond.of(0)).in(RadiansPerSecond));

                    driveL.set(sysL.getOutput());
                    driveR.set(sysR.getOutput());
                    activation.set(ACTIVATION_SPEED);
            }
        }
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.amps = driveL.getOutputCurrent() + driveR.getOutputCurrent();
        inputs.state = state;
        inputs.appliedVoltageL = sysL.getOutput();
        inputs.appliedVoltageR = sysR.getOutput();
        inputs.velocityL = getWheelSpeedL();
        inputs.velocityR = getWheelSpeedR();
    }

    @Override
    public Command spinUp() {
        return runOnce(() -> state = SPINUP);
    }

    @Override
    public Command sysId() {
        return sysIdQuasistaticL(SysIdRoutine.Direction.kForward).andThen(sysIdQuasistaticL(SysIdRoutine.Direction.kReverse)).andThen(sysIdDynamicL(SysIdRoutine.Direction.kForward)).andThen(sysIdDynamicL(SysIdRoutine.Direction.kReverse))
                .andThen(sysIdQuasistaticR(SysIdRoutine.Direction.kForward)).andThen(sysIdQuasistaticR(SysIdRoutine.Direction.kReverse)).andThen(sysIdDynamicR(SysIdRoutine.Direction.kForward)).andThen(sysIdDynamicR(SysIdRoutine.Direction.kReverse));
    }

    @Override
    public Command test() {
        return runOnce(() -> state = TESTING);
    }

    @Override
    public Command shoot() {
        return runOnce(() -> state = SHOOT);
    }
}
