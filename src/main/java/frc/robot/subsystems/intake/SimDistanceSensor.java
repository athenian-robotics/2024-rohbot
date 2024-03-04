package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import lombok.Getter;

public class SimDistanceSensor extends SubsystemBase {

    @Getter private boolean on = false;
    public SimDistanceSensor() {
    }

    @Override
    public void periodic() {
        on = RobotContainer.INTAKE_SENSOR_TRIGGERED;
    }
}
