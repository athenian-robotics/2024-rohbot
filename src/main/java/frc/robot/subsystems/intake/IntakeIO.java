package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double sensorDistance;
        public boolean hasNote;
        public boolean on;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void on() {}
    public default void off() {}
}
