package frc.robot.subsystems.powerBudget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerBudget extends SubsystemBase {
    private final PowerBudgetIO io;
    private final  PowerBudgetIO.PowerBudget inputs;

    public PowerBudget(PowerBudgetIO io) {
        this.io = io;
        inputs = new PowerBudgetIO.PowerBudget();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    // only for sim
    public void report(final double currentDrawAmps, final int... channels) {
        io.report(currentDrawAmps, channels);
    }

    // only for sim
    public void report(final double currentDrawAmps) {
        io.report(currentDrawAmps);
    }

    public boolean hasCurrent(double currentlyUsing, double wantToUse) {
        return io.hasCurrent(currentlyUsing, wantToUse);
    }
}
