package frc.robot.subsystems.shooter;

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

  default void pulse() {}

  default void intake() {}

  void sysIdState();

  enum State {
    SPINUP,
    SYSID,
    TESTING,
    SHOOT,
    INTAKE,
    AMP
  }

  @AutoLog
  class ShooterIOInputs {
    public State state;
    public double amps;
    public double appliedVoltageL;
    public double appliedVoltageR;
    public double velocityL;
    public double velocityR;
    public boolean ready;
    public double errorL;
    public double errorR;
    public double setPointL;
    public double setPointR;
  }
}
