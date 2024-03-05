package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.powerBudget.PowerBudget;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class IntakeSim implements IntakeIO {
    private final PowerBudget powerBudget;
    private boolean intakeOn = false;
    // TODO: Implement LoggedDashboardNumber
    private final LoggedDashboardBoolean hasNote = new LoggedDashboardBoolean("IntakeSim/hasNote");
    public IntakeSim(PowerBudget powerBudget) {
        this.powerBudget = powerBudget;
        // TODO: Implement powerbudget.report (DCMotorSim) and recalc or just sysid
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.on = intakeOn;
        inputs.sensorDistance = 0; // lol
        inputs.hasNote = hasNote();
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
