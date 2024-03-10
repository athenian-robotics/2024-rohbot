package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  boolean ready();

  void updateInputs(ShooterIOInputs inputs);

  void spinUp();

  default Command sysId() {
    return new PrintCommand("your dumb");
  }

  void test();

  void shoot();

  default void amp() {}

  void fixedSpeaker();

  default void pulse() {}

    default void intake() {

    }

    enum State {
    SPINUP,
    SYSID,
    TESTING,
    SHOOT,
    FIXED_SPEAKER,
    AMP
  }

  @AutoLog
  class ShooterIOInputs {
    public State state;
    public double amps;
    public double appliedVoltageL;
    public double appliedVoltageR;
    public Measure<Velocity<Angle>> velocityL;
    public Measure<Velocity<Angle>> velocityR;
    public boolean ready;
    public double error;
  }
}
