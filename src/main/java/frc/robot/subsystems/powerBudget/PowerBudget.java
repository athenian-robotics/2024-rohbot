package frc.robot.subsystems.powerBudget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class PowerBudget extends SubsystemBase {
  private final PowerBudgetIO io;
  private final PowerBudgetAutoLogged inputs;

  public PowerBudget(PowerBudgetIO io) {
    this.io = io;
    inputs = new PowerBudgetAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("powerBudget", inputs);
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
