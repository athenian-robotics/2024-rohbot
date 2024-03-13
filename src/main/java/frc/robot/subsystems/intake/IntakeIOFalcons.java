package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeIOFalcons extends SubsystemBase implements IntakeIO {
  private static final int LEAD_MOTOR_ID = 9;
  private static final int FOLLOW_MOTOR_ID = 10;
  private static final double EMPTY_THRESHOLD = 680;
  private static final double HAS_NOTE_THRESHOLD = 375;
  private static final int CURRENT_LIMIT = 5;
  private final TalonFX leadMotor;
  private final TalonFX followMotor;
  private boolean intakeOn = false;

  private boolean isInverted = true;

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
    intakeOn = true;
    leadMotor.set(1);
  }

  @Override
  public void off() {
    intakeOn = false;
    leadMotor.set(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.on = intakeOn;
    inputs.amps =
        leadMotor.getSupplyCurrent().getValue() + followMotor.getSupplyCurrent().getValue();
  }
}
