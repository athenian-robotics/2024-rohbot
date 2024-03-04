package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

public class IntakeSim implements IntakeIO {
    private boolean intakeOn = false;
    private final LoggedDashboardBoolean hasNote = new LoggedDashboardBoolean("IntakeSim/hasNote");

    public IntakeSim() {

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
