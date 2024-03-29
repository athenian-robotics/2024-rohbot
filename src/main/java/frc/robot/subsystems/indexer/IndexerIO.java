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

  default boolean ready() {
    return true;
  }

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
    AMP_INIT,
    SHOOTFIXED,
    AMP_PULSE
  }

  @AutoLog
  class IndexerIOInputs {
    public double angle;
    public double velocity;
    public double appliedVoltage;
    public State state;
    public boolean inBounds;
    public boolean upBound;
    public boolean lowBound;
    public double error;
    public double errorVelo;
    public double amps;
    public boolean hasCurrent;
    public boolean ready;
    public double limiterValue;
  }
}
