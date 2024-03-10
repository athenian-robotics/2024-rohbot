package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  default void updateInputs(IntakeIOInputs inputs) {}

  default void on() {}

  default void off() {}

  default void reverse() {}

  @AutoLog
  class IntakeIOInputs {
    public boolean on;
  }
}
