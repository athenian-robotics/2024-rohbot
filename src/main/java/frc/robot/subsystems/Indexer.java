package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final int motorPort = 0; // TODO: ADD MOTOR PORTS ACCURATELY
  private final int sensorPort = 0;
  private final CANSparkMax motor;
  private final Rev2mDistanceSensor sensor;

  //  private final Rev2mDistanceSensor sensor;

  enum State {
    FIRING,
    LOADING,
    READY,
    EMPTY
  }

  public Indexer() {
    motor = new CANSparkMax(motorPort, CANSparkLowLevel.MotorType.kBrushless);
    sensor = new Rev2mDistanceSensor()
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
