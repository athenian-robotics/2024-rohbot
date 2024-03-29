package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;

  public Shooter(ShooterIO io) {
    this.io = io;
    inputs = new ShooterIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("shooter", inputs);
  }

  public void spinUp() {
    io.spinUp();
  }

  public Command sysId() {
    return io.sysId();
  }

  public void test() {
    io.test();
  }

  public void shoot() {
    io.shoot();
  }

  public boolean ready() {
    return io.ready();
  }

  public void amp() {
    io.amp();
  }

  public void pulse() {
    io.pulse();
  }

  public void intake() {
    io.intake();
  }

  public void sysIdState() {
    io.sysIdState();
  }

  public void shootFixed() {
    io.shootFixed();
  }

  public void shootAcrossField() {
    io.shootAcrossField();
  }
}
