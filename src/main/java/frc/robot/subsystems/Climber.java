package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  public Climber() {
    // TODO: add motor ports
    int LEFT_MOTOR_ID = 15;
    leftMotor = new TalonFX(LEFT_MOTOR_ID);
    int RIGHT_MOTOR_ID = 16;
    rightMotor = new TalonFX(RIGHT_MOTOR_ID);
    leftMotor.setInverted(true); // TODO: Change based on change
    rightMotor.setInverted(true);
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  private void setTelescopeSpeed(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  public Command telescopeUp() {
    return startEnd(() -> setTelescopeSpeed(0.5), () -> setTelescopeSpeed(0));
  }

  public Command telescopeDown() {
    return startEnd(() -> setTelescopeSpeed(-0.5), () -> setTelescopeSpeed(0));
  }

  public Command leftUp() {
    return startEnd(() -> leftMotor.set(0.5), () -> leftMotor.set(0));
  }

  public Command leftDown() {
    return startEnd(() -> leftMotor.set(-0.5), () -> leftMotor.set(0));
  }

  public Command rightUp() {
    return startEnd(() -> rightMotor.set(0.5), () -> rightMotor.set(0));
  }

  public Command rightDown() {
    return startEnd(() -> rightMotor.set(-0.5), () -> rightMotor.set(0));
  }
}
