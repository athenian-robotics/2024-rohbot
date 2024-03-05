package frc.robot.subsystems.intake;


import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeIOSparkMax extends SubsystemBase implements IntakeIO {
  private static final int LEAD_MOTOR_ID = 9;
  private static final int FOLLOW_MOTOR_ID = 10;
  private static final double EMPTY_THRESHOLD = 680;
  private static final double HAS_NOTE_THRESHOLD = 375;
  private static final int CURRENT_LIMIT = 10;
  private final CANSparkMax leadMotor;
  private final TimeOfFlight sensor;
  private boolean intakeOn = false;

  public IntakeIOSparkMax() {
    leadMotor = new CANSparkMax(LEAD_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax followMotor =
        new CANSparkMax(FOLLOW_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

    leadMotor.restoreFactoryDefaults();
    followMotor.restoreFactoryDefaults();

    followMotor.follow(leadMotor);
    leadMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    followMotor.setSmartCurrentLimit(CURRENT_LIMIT);

    sensor = new TimeOfFlight(16);
    sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 0.02);
  }

  public Double getDistance() {
    return sensor.getRange();
  }

  private boolean hasNote() {
    return sensor.getRange() <= HAS_NOTE_THRESHOLD;
  }

  @Override
  public void on() {
    intakeOn = true;
    leadMotor.set(-1);
  }

  @Override
  public void off() {
    intakeOn = false;
    leadMotor.set(0);
  }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.on = intakeOn;
        inputs.sensorDistance = sensor.getRange();
        inputs.hasNote = hasNote();
    }
  public boolean empty() {
    return sensor.getRange() >= EMPTY_THRESHOLD;
  }
}
