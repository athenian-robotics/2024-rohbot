package frc.robot.subsystems.indexer;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO  {
    @AutoLog
    public static class IndexerIOInputs {
        public boolean hasNote;
        public double sensorDistance;
        public Measure<Angle> angle;
        public Measure<Velocity<Angle>> velocity;
        public double appliedVoltage;
        public State state;
    }
    enum State {
        FLAT,
        ADJUSTING,
        IDLE,
        TESTING
    }
    public default void updateInputs(IndexerIOInputs inputs) {}
    public default void setState(State state) {}
    public boolean ready();
}
