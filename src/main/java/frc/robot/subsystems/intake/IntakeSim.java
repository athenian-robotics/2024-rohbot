package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeIO.State.*;

import frc.robot.subsystems.powerBudget.PowerBudget;

public class IntakeSim implements IntakeIO {
  private final PowerBudget powerBudget; // TODO: Implement
  private State state = OFF;

  public IntakeSim(PowerBudget powerBudget) {
    this.powerBudget = powerBudget;
    // TODO: Implement powerbudget.report (DCMotorSim) and recalc or just sysid
  }

  @Override
  public void on() {
    state = ON;
  }

  @Override
  public void off() {
    state = OFF;
  }

  @Override
  public void reverse() {
    state = REVERSE;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.state = state;
  }
}
