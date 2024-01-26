package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class Indexer extends SubsystemBase {
  private static final int INDEXER_MOTOR_ID = 0; // TODO: ADD MOTOR PORTS ACCURATELY
  private static final int ANGLE_MOTOR_ID = 0; // TODO: port
  private static final double kV = 0; // TODO: Sysid
  private static final double kA = 0; // TODO: sysid
  private static final double GEAR_RATIO = 1; // TODO: fill
  private static final double TICKS_TO_RAD =
      2 * Math.PI / 2048 / GEAR_RATIO; // TODO: adjust for this year's robot
  private static final Measure<Distance> NOTE_LOADED_THRESHOLD = Units.Inches.of(0); // TODO: Tune
  private final CANSparkMax indexMotor;
  private final CANSparkMax angleMotor;
  private final Rev2mDistanceSensor sensor;
  private final LinearSystemLoop loop;
  @Getter private State state;

  //  private final Rev2mDistanceSensor sensor;

  enum State {
    FIRING,
    LOADING,
    LOADED,
    READY,
    EMPTY
  }

  public Indexer() {
    indexMotor = new CANSparkMax(INDEXER_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    angleMotor = new CANSparkMax(ANGLE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);
    sensor =
        new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kOnboard); // TODO: Figure out right value

    LinearSystem<N2, N1, N1> sys = LinearSystemId.identifyPositionSystem(kV, kA);

    KalmanFilter<N2, N1, N1> filter =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            sys,
            VecBuilder.fill(0.02, 0.02),
            VecBuilder.fill(TICKS_TO_RAD),
            0.02);

    LinearQuadraticRegulator<N2, N1, N1> controller =
        new LinearQuadraticRegulator<>(
            sys, VecBuilder.fill(0.0015, 0.02), VecBuilder.fill(12), 0.02);

    loop = new LinearSystemLoop<>(sys, controller, filter, 12, 0.02);
  }

  @Override
  public void periodic() {
    switch (state) {
      case EMPTY -> {
        indexMotor.set(0);
      }
      case LOADING -> {
        indexMotor.set(1); // TODO: Tune
        if (sensor.getRange(Rev2mDistanceSensor.Unit.kInches)
            < NOTE_LOADED_THRESHOLD.in(Units.Inches)) {
          state = State.READY;
        }
      }
      case READY -> {
        indexMotor.set(0);
      }
      case LOADED -> {
        indexMotor.set(0);
      }
      case FIRING -> {
        indexMotor.set(1); // TODO: Tune
      }
    }
  }
}
