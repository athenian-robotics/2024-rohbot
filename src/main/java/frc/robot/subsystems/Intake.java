package frc.robot.subsystems;

import static monologue.Annotations.*;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {
  private static final int LEAD_MOTOR_ID = 9;
  private static final int FOLLOW_MOTOR_ID = 10;
  private static final Measure<Distance> NOTE_FOUND_THRESHOLD = Units.Inches.of(680); // TODO: Tune
  private static final Measure<Distance> NOTE_PASSED_THRESHOLD = Units.Inches.of(680); // TODO: Tune
  private final CANSparkMax leadMotor;
  private final TimeOfFlight sensor;

  @Log.NT @Getter private State state;

  public Intake() {
    leadMotor = new CANSparkMax(LEAD_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    CANSparkMax followMotor =
        new CANSparkMax(FOLLOW_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    followMotor.follow(leadMotor);
    sensor = new TimeOfFlight(16);
    sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 0.02);
    state = State.LOOKING_FOR_NOTE;
  }

  public boolean isNoteFound() {
    return this.getState() == State.NOTE_FOUND;
  }

  public boolean isNoNote() {
    return this.getState() == State.LOOKING_FOR_NOTE;
  }

  public boolean isNotePassed() {
    return this.getState() == State.NOTE_PASSED;
  }

  public Command startIntake() {
    return runOnce(() -> state = State.LOOKING_FOR_NOTE);
  }

  public Command stopIntake() {
    return runOnce(() -> state = State.NOTE_PASSED);
  }

  public Command toggleIntake() {
    return state == State.NOTE_PASSED ? startIntake() : stopIntake();
  }

  @Log.NT
  public Double getDistance() {
    return sensor.getRange();
  }

  @Override
  public void periodic() {
    switch (state) {
      case LOOKING_FOR_NOTE:
        leadMotor.set(-1); // TODO: Tune speed
        if (Units.Inches.of(sensor.getRange()).baseUnitMagnitude()
            < NOTE_FOUND_THRESHOLD.in(Units.Inches)) {
          state = State.NOTE_FOUND;
        }
        break;
      case NOTE_FOUND:
        leadMotor.set(-1); // TODO: Tune
        if (Units.Inches.of(sensor.getRange()).baseUnitMagnitude()
            > NOTE_PASSED_THRESHOLD.in(Units.Inches)) {
          state = State.NOTE_PASSED;
        }
        break;
      case NOTE_PASSED:
        leadMotor.set(0);
    }

    SmartDashboard.putNumber("intake sensor", sensor.getRange());
  }

  private enum State {
    LOOKING_FOR_NOTE,
    NOTE_FOUND,
    NOTE_PASSED
  }
}
