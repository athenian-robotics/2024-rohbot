package frc.robot.subsystems.shooter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterDataTable;
import frc.robot.ShooterSpec;
import frc.robot.inputs.poseEstimator.PoseEstimator;
import frc.robot.lib.SimpleVelocitySystem;
import frc.robot.subsystems.powerBudget.PowerBudgetPhysical;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.Optional;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterIO.State.*;

public class ShooterIOSim extends SubsystemBase implements ShooterIO {
    private static final Measure<Voltage> kS = Volts.of(.01);
    private static final Measure<Per<Voltage, Velocity<Angle>>> kV = VoltsPerRadianPerSecond.of(.087);
    private static final Measure<Per<Voltage, Velocity<Velocity<Angle>>>> kA =
            VoltsPerRadianPerSecondSquared.of(0.06);
    private static final double MAX_ERROR = 1;
    private static final double MAX_CONTROL_EFFORT = 8;
    private static final double MODEL_DEVIATION = 1;
    private static final Measure<Time> LOOP_TIME = Second.of(0.02);
    private static final int CURRENT_LIMIT = 20;
    private static final double TOTAL_CURRENT_LIMIT = CURRENT_LIMIT * 3;
    private static final double ACTIVATION_SPEED = 0.1; // TODO: Tune

    private final PoseEstimator poseEstimator;
    private final SimpleVelocitySystem sysL;
    private final SimpleVelocitySystem sysR;
    private final ShooterDataTable table;
    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(Rotations.per(Second).of(0));
    private final LoggedDashboardNumber numL = new LoggedDashboardNumber("left shooter power deg/s", 0);
    private final LoggedDashboardNumber numR = new LoggedDashboardNumber("right shooter power deg/s", 0);
    private final LoggedDashboardBoolean activation = new LoggedDashboardBoolean("shooter activation", false);
    private final PowerBudgetPhysical power;
    @Getter
    private State state = SPINUP;
    private FlywheelSim driveL;
    private FlywheelSim driveR;
    private boolean activated;

    public ShooterIOSim(
            ShooterDataTable table, PoseEstimator poseEstimator, PowerBudgetPhysical power) {

        this.table = table;
        this.power = power;
        driveL = new FlywheelSim(LinearSystemId.identifyVelocitySystem(kV.baseUnitMagnitude(), kA.baseUnitMagnitude()), DCMotor.getKrakenX60(1), 1);
        driveR = new FlywheelSim(LinearSystemId.identifyVelocitySystem(kV.baseUnitMagnitude(), kA.baseUnitMagnitude()), DCMotor.getKrakenX60(1), 1);

        sysL =
                new SimpleVelocitySystem(
                        kS.in(Volts),
                        kV.in(VoltsPerRadianPerSecond),
                        kA.in(VoltsPerRadianPerSecondSquared),
                        MAX_ERROR,
                        MAX_CONTROL_EFFORT,
                        MODEL_DEVIATION,
                        0,
                        LOOP_TIME.in(Seconds));
        sysR =
                new SimpleVelocitySystem(
                        kS.in(Volts),
                        kV.in(VoltsPerRadianPerSecond),
                        kA.in(VoltsPerRadianPerSecondSquared),
                        MAX_ERROR,
                        MAX_CONTROL_EFFORT,
                        MODEL_DEVIATION,
                0,
                        LOOP_TIME.in(Seconds));
        this.poseEstimator = poseEstimator;
    }


    private Measure<Velocity<Angle>> getWheelSpeedL() {
        return RPM.of(driveL.getAngularVelocityRPM());
    }

    private Measure<Velocity<Angle>> getWheelSpeedR() {
        return RPM.of(driveR.getAngularVelocityRPM());
    }

    @Override
    public boolean ready() {
        return Math.abs(sysL.getError()) < MAX_ERROR && Math.abs(sysR.getError()) < MAX_ERROR;
    }

    @Override
    public void periodic() {
        switch (state) {
            case SPINUP -> {
                if (power.hasCurrent(
                        driveL.getCurrentDrawAmps() + driveR.getCurrentDrawAmps(), TOTAL_CURRENT_LIMIT * 3 / 2)) {
                    sysL.update(getWheelSpeedL().in(RadiansPerSecond)); // Returns RPS
                    sysR.update(getWheelSpeedR().in(RadiansPerSecond));

                    Optional<ShooterSpec> spec = table.get(poseEstimator.translationToSpeaker());

                    sysL.set(
                            spec.map(ShooterSpec::speedL).orElse(DegreesPerSecond.of(0)).in(RadiansPerSecond));
                    sysR.set(
                            spec.map(ShooterSpec::speedR).orElse(DegreesPerSecond.of(0)).in(RadiansPerSecond));

                    driveL.setInputVoltage(sysL.getOutput());
                    driveR.setInputVoltage(sysR.getOutput());
                    activated = false;
                }
            }
            case SYSID -> {
                activated = false;
                // chill
            }
            case TESTING -> {
                sysL.update(getWheelSpeedL().in(RadiansPerSecond)); // Returns RPS
                sysR.update(getWheelSpeedR().in(RadiansPerSecond));

                sysL.set(DegreesPerSecond.of(numL.get()).in(RadiansPerSecond));
                sysR.set(DegreesPerSecond.of(numR.get()).in(RadiansPerSecond));

                activated = activation.get();
                driveL.setInputVoltage(sysL.getOutput());
                driveR.setInputVoltage(sysR.getOutput());
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

                driveL.setInputVoltage(sysL.getOutput());
                driveR.setInputVoltage(sysR.getOutput());
                activated = true;
            }
        }
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.amps = driveL.getCurrentDrawAmps() + driveR.getCurrentDrawAmps();
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
    public Command test() {
        return runOnce(() -> state = TESTING);
    }

    @Override
    public Command shoot() {
        return runOnce(() -> state = SHOOT);
    }
}
