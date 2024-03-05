package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIO.IndexerIOInputs inputs;

    public Indexer(IndexerIO io) {
        this.io = io;
        inputs = new IndexerIO.IndexerIOInputs();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.periodic();
    }

    public void setState(IndexerIO.State state) {
        io.setState(state);
    }

    public boolean ready() {
        return io.ready();
    }
}
