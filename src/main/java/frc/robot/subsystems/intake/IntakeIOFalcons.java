package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeIO.State.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeIOFalcons extends SubsystemBase implements IntakeIO {
  private static final int LEAD_MOTOR_ID = 9;
  private static final int FOLLOW_MOTOR_ID = 10;
  private static final int CURRENT_LIMIT = 5;
  private final TalonFX leadMotor;
  private final TalonFX followMotor;
  private State state = State.OFF;

  public IntakeIOFalcons() {
    leadMotor = new TalonFX(LEAD_MOTOR_ID, "can");
    leadMotor.setInverted(true);
    followMotor = new TalonFX(FOLLOW_MOTOR_ID, "can");

    leadMotor
        .getConfigurator()
        .apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(CURRENT_LIMIT));

    followMotor
        .getConfigurator()
        .apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(CURRENT_LIMIT));
    followMotor.setControl(new Follower(LEAD_MOTOR_ID, false));
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
  public void periodic() {
    switch (state) {
      case REVERSE -> leadMotor.set(-.8);
      case OFF -> leadMotor.set(0);
      case ON -> leadMotor.set(.8);
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.state = state;
    inputs.amps =
        leadMotor.getSupplyCurrent().getValue() + followMotor.getSupplyCurrent().getValue();
  }
}
