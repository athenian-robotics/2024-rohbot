package frc.robot.subsystems.shooter;

public class Shooter {
    private final ShooterIO io;
    private final ShooterIO.ShooterIOInputs inputs;

    public Shooter(ShooterIO io) {
        this.io = io;
        inputs = new ShooterIO.ShooterIOInputs();
    }

    public void periodic() {
        io.updateInputs(inputs);
    }

    public void spinUp() {
        io.spinUp();
    }

    public void sysId() {
        io.sysId();
    }

    public void test() {
        io.test();
    }

    public void shoot() {
        io.shoot();
    }

    public boolean ready() {
        return io.ready();
    }
}
