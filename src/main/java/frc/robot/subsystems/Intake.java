package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static final int MOTOR_ID = 0; // TODO: Fill out value
  private static final Measure<Distance> NOTE_FOUND_THRESHOLD = Units.Inches.of(0); // TODO: Tune
  private static final Measure<Distance> NOTE_PASSED_THRESHOLD = Units.Inches.of(0); // TODO: Tune
  private final CANSparkMax motor;
  private final Rev2mDistanceSensor sensor;
  private enum State {
    NO_NOTE,
    NOTE_FOUND,
    NOTE_PASSED
  }
  private State state;

  public Intake() {
    motor = new CANSparkMax(MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    sensor = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard);
    state = State.NO_NOTE;
  }

  public Command shotFired() {
    return new InstantCommand(() -> state = State.NO_NOTE, this);
  }

  @Override
  public void periodic() {
    switch (state) {
      case NO_NOTE:
        motor.set(1); // TODO: Tune speed
        if (sensor.getRange(Rev2mDistanceSensor.Unit.kInches) < NOTE_FOUND_THRESHOLD.in(Units.Inches)) {
          state = State.NOTE_FOUND;
        }
        break;
      case NOTE_FOUND:
        motor.set(1); // TODO: Tune
        if (sensor.getRange(Rev2mDistanceSensor.Unit.kInches) > NOTE_PASSED_THRESHOLD.in(Units.Inches)) {
          state = State.NOTE_PASSED;
        }
        break;
      case NOTE_PASSED:
        motor.set(0);
    }
  }
}
