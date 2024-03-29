package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  default void updateInputs(IntakeIOInputs inputs) {}

  default void on() {}

  default void off() {}

  void reverse();

  enum State {
    REVERSE,
    OFF,
    ON
  }

  @AutoLog
  class IntakeIOInputs {
    public double amps;
    public State state;
  }
}
