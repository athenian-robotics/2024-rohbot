package frc.robot.subsystems;


import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShooterDataTable;
import frc.robot.ShooterSpec;
import frc.robot.lib.SimpleVelocitySystem;
import org.ejml.dense.row.misc.ImplCommonOps_DDMA;

public class  Shooter extends SubsystemBase {
    private static final int LEFT_ID = 0; // TODO: fill in
    private static final int RIGHT_ID = 0; // TODO: fill
    private static final int kS = 0;
    private static final int kV = 0;
    private static final int kA = 0;
    private static final int MAX_ERROR = 0;
    private static final double MAX_CONTROL_EFFORT = 0.0;
    private static final int MODEL_DEVIATION = 0;
    private static final int ENCODER_DEVIATION = 0;
    private static final int LOOPTIME = 0;

    private final CANSparkMax shooterL = new CANSparkMax(LEFT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final CANSparkMax shooterR = new CANSparkMax(RIGHT_ID, CANSparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder shooterLEncoder;
    private final RelativeEncoder shooterREncoder;
    private State state = State.IDLE;
    private final SimpleVelocitySystem sysL;
    private final SimpleVelocitySystem sysR;
    private final ShooterDataTable dataTable;

    public Shooter() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        shooterLEncoder = shooterL.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        shooterREncoder = shooterR.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

        dataTable = new ShooterDataTable();

        sysL = new SimpleVelocitySystem(kS, kV, kA, MAX_ERROR, MAX_CONTROL_EFFORT, MODEL_DEVIATION, ENCODER_DEVIATION, LOOPTIME);
        sysR = new SimpleVelocitySystem(kS, kV, kA, MAX_ERROR, MAX_CONTROL_EFFORT, MODEL_DEVIATION, ENCODER_DEVIATION, LOOPTIME);
    }

    private enum State {
        IDLE, // default state
        READY, // at setpoint and within tolerance
        APPROACHING, // approaching setpoint
        TESTING // for collecting shooter data table values
    }

    private boolean atSetpoint() {
        return Math.abs(sysL.getError()) < MAX_ERROR && Math.abs(sysR.getError()) < MAX_ERROR;
    }

    @Override
    public void periodic() {
        sysL.update(shooterLEncoder.getVelocity()); // Returns RPM
        sysR.update(shooterREncoder.getVelocity());
        switch (state) {
            case IDLE:
                sysL.set(0.0);
                sysR.set(0.0);
                break;
            case TESTING:
                // log values
                break;
            case APPROACHING:
                // send limelight data to data table, send result to system
                double distance = 0.0;// TODO: get limelight data
                ShooterSpec spec = dataTable.getSpecs(distance);
                sysL.set(spec.getLeftSpeed());
                sysR.set(spec.getRightSpeed());
                break;
        }
        if (atSetpoint()) {
            state = State.READY;
        }
    }
}

