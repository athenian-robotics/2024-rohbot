package frc.robot.subsystems.intake;

import frc.robot.subsystems.powerBudget.PowerBudget;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class IntakeSim implements IntakeIO {
  private final PowerBudget powerBudget;
  // TODO: Implement LoggedDashboardNumber
  private final LoggedDashboardBoolean hasNote = new LoggedDashboardBoolean("IntakeSim/hasNote");
  private boolean intakeOn = false;

  public IntakeSim(PowerBudget powerBudget) {
    this.powerBudget = powerBudget;
    // TODO: Implement powerbudget.report (DCMotorSim) and recalc or just sysid
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.on = intakeOn;
  }

  private boolean hasNote() {
    return hasNote.get();
  }

  @Override
  public void on() {
    intakeOn = true;
  }

  @Override
  public void off() {
    intakeOn = false;
  }
}
