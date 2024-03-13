package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
    periodic();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("intake", inputs);
  }

  public void on() {
    io.on();
  }

  public void off() {
    io.off();
  }

  public void reverse() {
    io.reverse();
  }
}
