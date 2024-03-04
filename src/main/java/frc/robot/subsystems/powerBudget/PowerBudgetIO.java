package frc.robot.subsystems.powerBudget;

import org.littletonrobotics.junction.AutoLog;

public interface PowerBudgetIO {
    @AutoLog
    public static class PowerBudget {
        public double currentlyUsing;
        public double batteryVoltage;
    }

    public void updateInputs(PowerBudget inputs);
    abstract public boolean hasCurrent(double currentlyUsing, double wantToUse);
    public default void report(final double currentDrawAmps, final int... channels) {}
    public default void report(final double currentDrawAmps) {}
}
