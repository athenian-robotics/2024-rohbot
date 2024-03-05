package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public State state;
        public double amps;
        public double sensorDistance;
public double appliedVoltage;
    public Measure<Velocity<Angle>> velocityL;
    public Measure<Velocity<Angle>> velocityR;


    }
    enum State {
        APPROACHING,
        SYSID,
        TESTING,
        ACTIVATED
    }

public default void updateInputs(ShooterIOInputs inputs) {}

}
