package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs;

  public Indexer(IndexerIO io) {
    this.io = io;
    inputs = new IndexerIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.periodic();
    Logger.processInputs("indexer", inputs);
  }

  public void setState(IndexerIO.State state) {
    io.setState(state);
  }

  public boolean ready() {
    return io.ready();
  }

  public Command sysId() {
    return io.sysId();
  }

  public Command zero() {
    return io.zero();
  }

  public void fudge(DoubleSupplier o) {
    io.fudge(o);
  }
}
