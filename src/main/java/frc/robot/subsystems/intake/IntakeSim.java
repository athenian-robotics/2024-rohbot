package frc.robot.subsystems.intake;

import frc.robot.subsystems.powerBudget.PowerBudget;

public class IntakeSim implements IntakeIO {
  private final PowerBudget powerBudget;
  private boolean intakeOn = false;

  public IntakeSim(PowerBudget powerBudget) {
    this.powerBudget = powerBudget;
    // TODO: Implement powerbudget.report (DCMotorSim) and recalc or just sysid
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.on = intakeOn;
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
