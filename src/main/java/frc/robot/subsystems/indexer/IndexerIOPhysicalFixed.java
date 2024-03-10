package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.powerBudget.PowerBudgetPhysical;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class IndexerIOPhysicalFixed extends SubsystemBase implements IndexerIO {
  private static final int LEAD_ANGLE_MOTOR_ID = 10;
  private static final int FOLLOW_ANGLE_MOTOR_ID = 11;
  private static final int CURRENT_LIMIT = 30;
  private static final Measure<Angle> ROT_TO_ANGLE = Rotations.of(1 / ((54.0 / 22) * 60));
  private final CANSparkFlex leadAngleMotor;
  private final CANSparkFlex followAngleMotor;
  private final PowerBudgetPhysical power;
  private final LoggedDashboardNumber angle = new LoggedDashboardNumber("hood angle", 0);
  @Getter private State state = State.TESTING;

  @Getter private double UPPER_LIMIT = 10; // 40
  @Getter private double LOWER_LIMIT = 5;

  public IndexerIOPhysicalFixed(final Drive poseEstimator, PowerBudgetPhysical power) {
    this.power = power;
    leadAngleMotor = new CANSparkFlex(LEAD_ANGLE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    leadAngleMotor.restoreFactoryDefaults();
    leadAngleMotor.setInverted(false);
    leadAngleMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    followAngleMotor =
        new CANSparkFlex(FOLLOW_ANGLE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    followAngleMotor.restoreFactoryDefaults();
    followAngleMotor.setSmartCurrentLimit(CURRENT_LIMIT);
    followAngleMotor.follow(leadAngleMotor, true);

    leadAngleMotor.getEncoder().setPosition(0.0);
    followAngleMotor.getEncoder().setPosition(0.0);
    leadAngleMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    followAngleMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    leadAngleMotor.burnFlash();
    followAngleMotor.burnFlash();
  }

  private void log(SysIdRoutineLog sysIdRoutineLog) {
    sysIdRoutineLog
        .motor("hood-motor")
        .voltage(Volts.of(leadAngleMotor.getAppliedOutput() * leadAngleMotor.getBusVoltage()))
        .angularPosition(getAngle())
        .angularVelocity(getVelocity());
  }

  public Measure<Angle> getAngle() {
    return Degrees.of(leadAngleMotor.getEncoder().getPosition() * ROT_TO_ANGLE.in(Degrees));
  }

  public double getVoltage() {
    return leadAngleMotor.getBusVoltage() * leadAngleMotor.getAppliedOutput();
  }

  public void periodic() {}

  @Override
  public void setState(State state) {
    this.state = state;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.angle = getAngle().in(Degrees);
    inputs.velocity = getVelocity().in(DegreesPerSecond);
    inputs.appliedVoltage = getVoltage();
    inputs.state = state;
    inputs.inBounds = upperBound() && lowerBound();
    inputs.upBound = upperBound();
    inputs.lowBound = lowerBound();
  }

  private Measure<Velocity<Angle>> getVelocity() {
    return Rotations.per(Minutes)
        .of(leadAngleMotor.getEncoder().getVelocity() * ROT_TO_ANGLE.in(Rotations));
  }

  @Override
  public Command zero() {
    return runOnce(() -> leadAngleMotor.getEncoder().setPosition(0));
  }

  private boolean upperBound() {
    return leadAngleMotor.getEncoder().getPosition() < UPPER_LIMIT;
  }

  private boolean lowerBound() {
    return leadAngleMotor.getEncoder().getPosition() > LOWER_LIMIT;
  }
}
