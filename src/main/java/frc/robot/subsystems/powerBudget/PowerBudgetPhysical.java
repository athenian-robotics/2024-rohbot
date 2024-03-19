package frc.robot.subsystems.powerBudget;

import edu.wpi.first.wpilibj.PowerDistribution;

public class PowerBudgetPhysical implements PowerBudgetIO {
  private static final double LIMIT = 100;
  private final PowerDistribution pdh = new PowerDistribution();

  public PowerBudgetPhysical() {}

  @Override
  public void updateInputs(PowerBudget inputs) {
    inputs.currentlyUsing = pdh.getTotalCurrent();
    inputs.batteryVoltage = pdh.getVoltage();
  }

  // currentlyUsing is the current being used by the mechanism, wantToUse is the max current/total
  // current the mechanism wants to use
  @Override
  public boolean hasCurrent(double currentlyUsing, double wantToUse) {
    return pdh.getTotalCurrent() - currentlyUsing < LIMIT - wantToUse;
  }
}
