package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private final SuperstructureIO superstructureIO;
  private final SuperstructureInputsAutoLogged inputs = new SuperstructureInputsAutoLogged();

  public Superstructure(SuperstructureIO superstructureIO) {
    this.superstructureIO = superstructureIO;
  }

  @Override
  public void periodic() {
    superstructureIO.iterateStateMachine();
    superstructureIO.updateInputs(inputs);
    Logger.processInputs("superstructure", inputs);
  }

  public Command shoot() {
    return superstructureIO.shoot();
  }

  public Command test() {
    return superstructureIO.test();
  }

  public Command cancelShot() {
    return superstructureIO.cancelShot();
  }
}
