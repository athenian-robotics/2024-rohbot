package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public State state;
        public double amps;
public double appliedVoltageL;
public double appliedVoltageR;
    public Measure<Velocity<Angle>> velocityL;
    public Measure<Velocity<Angle>> velocityR;


    }
    enum State {
        SPINUP,
        SYSID,
        TESTING,
        SHOOT
    }

    boolean ready();

    void updateInputs(ShooterIOInputs inputs);
    Command spinUp();
    default Command sysId(){ return new PrintCommand("your dumb");}
    Command test();
    Command shoot();
}
