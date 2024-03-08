package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  default void updateInputs(IndexerIOInputs inputs) {}

  default void setState(State state) {}

  default Command sysId() {
    return new PrintCommand("your dumb");
  }

  boolean ready();

  void periodic();

  default Command zero() {
    return new PrintCommand("ur dumb");
  }

  enum State {
    FLAT,
    ADJUSTING,
    IDLE,
    SYSID,
    TESTING,
    AMP
  }

  @AutoLog
  class IndexerIOInputs {
    public double angle;
    public double velocity;
    public double appliedVoltage;
    public State state;
    public boolean inBounds;
  }
}
