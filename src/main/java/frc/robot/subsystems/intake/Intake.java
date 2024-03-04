package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeIO.IntakeIOInputs inputs;


    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public boolean hasNote() {
        return inputs.hasNote;
    }

    public void on() {
        io.on();
    }

    public void off() {
        io.off();
    }
}
