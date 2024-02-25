package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;

public class PowerBudget {
  private static final double LIMIT = 100;
  private final PowerDistribution pdh = new PowerDistribution();

  public PowerBudget() {}

  public boolean hasCurrent(double currentlyUsing, double wantToUse) {
    return pdh.getTotalCurrent() - currentlyUsing < LIMIT - wantToUse;
  }
}
