package frc.robot.subsystems.indexer;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  default void updateInputs(IndexerIOInputs inputs) {}

  default void setState(State state) {}

  boolean ready();

  void periodic();

  enum State {
    FLAT,
    ADJUSTING,
    IDLE,
    TESTING
  }

  @AutoLog
  class IndexerIOInputs {
    public Measure<Angle> angle;
    public Measure<Velocity<Angle>> velocity;
    public double appliedVoltage;
    public State state;
  }
}
