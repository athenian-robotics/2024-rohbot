package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final int LEFT_MOTOR_ID = 15; //TODO: add motor ports
    private final int RIGHT_MOTOR_ID = 16;
    private TalonFX leftMotor;
    private TalonFX rightMotor;

    public Climber(){
        leftMotor = new TalonFX(LEFT_MOTOR_ID);
        rightMotor = new TalonFX(RIGHT_MOTOR_ID);
        leftMotor.setInverted(true); //TODO: Change based on change
        rightMotor.setInverted(true);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    private void setTelescopeSpeed(double speed){
        leftMotor.set(speed);
        rightMotor.set(speed);
    }
    public Command telescopeUp(){
        return new StartEndCommand(()->{
            setTelescopeSpeed(0.5);
        },()->{
            setTelescopeSpeed(0);
        }, this);
    }
    public Command telescopeDown(){
        return new StartEndCommand(()->{
            setTelescopeSpeed(-0.5);
        },()->{
            setTelescopeSpeed(0);
        }, this);
    }

    public Command leftUp() {
        return new StartEndCommand(() -> leftMotor.set(0.5), () -> leftMotor.set(0), this);
    }
    public Command leftDown() {
        return new StartEndCommand(() -> leftMotor.set(-0.5), () -> leftMotor.set(0), this);
    }
    public Command rightUp() {
        return new StartEndCommand(() -> rightMotor.set(0.5), () -> rightMotor.set(0), this);
    }
    public Command rightDown() {
        return new StartEndCommand(() -> rightMotor.set(-0.5), () -> rightMotor.set(0), this);
    }

}