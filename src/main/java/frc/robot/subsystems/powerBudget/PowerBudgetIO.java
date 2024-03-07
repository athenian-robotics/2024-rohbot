package frc.robot.subsystems.powerBudget;

import org.littletonrobotics.junction.AutoLog;

public interface PowerBudgetIO {
  void updateInputs(PowerBudget inputs);

  boolean hasCurrent(double currentlyUsing, double wantToUse);

  default void report(final double currentDrawAmps, final int... channels) {}

  default void report(final double currentDrawAmps) {}

  @AutoLog
  class PowerBudget {
    public double currentlyUsing;
    public double batteryVoltage;
  }
}
