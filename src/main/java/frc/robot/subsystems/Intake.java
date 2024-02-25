package frc.robot.subsystems;

import static monologue.Annotations.Log;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {
  private static final int LEAD_MOTOR_ID = 9;
  private static final int FOLLOW_MOTOR_ID = 10;
  private static final double EMPTY_THRESHOLD = 680;
  private static final double HAS_NOTE_THRESHOLD = 375;
  private static final int CURRENT_LIMIT = 10;
  private final CANSparkMax leadMotor;
  private final TimeOfFlight sensor;

  public Intake() {
    leadMotor = new CANSparkMax(LEAD_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax followMotor =
        new CANSparkMax(FOLLOW_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    followMotor.follow(leadMotor);
    leadMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    followMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    sensor = new TimeOfFlight(16);
    sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 0.02);
  }

  @Log.NT
  public Double getDistance() {
    return sensor.getRange();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake sensor", sensor.getRange());
  }

  public boolean hasNote() {
    return sensor.getRange() <= HAS_NOTE_THRESHOLD;
  }

  public void on() {
    leadMotor.set(-2);
  }

  public void off() {
    leadMotor.set(0);
  }

  public boolean empty() {
    return sensor.getRange() >= EMPTY_THRESHOLD;
  }
}
