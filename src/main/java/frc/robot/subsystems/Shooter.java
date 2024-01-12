package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class  Shooter extends SubsystemBase {
    private static final int LEFT_ID = 0; // TODO: fill in
    private static final int RIGHT_ID = 0; // TODO: fill

    private final CANSparkMax shooterL = new CANSparkMax(LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax shooterR = new CANSparkMax(RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private State state = State.IDLE;

    public Shooter() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        
    }

    private enum State {
        IDLE, // default state
        READY, // at setpoint and withing tolerance
        APPROACHING, // approaching setpoint
        TESTING // for collecting shooter data table values
    }

    @Override
    public void periodic() {

    }
}

