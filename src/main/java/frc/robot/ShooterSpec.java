package frc.robot;

public class ShooterSpec {
    private final double shooterAngle;
    private final double leftSpeed;

    public double getShooterAngle() {
        return shooterAngle;
    }

    public double getLeftSpeed() {
        return leftSpeed;
    }

    public double getRightSpeed() {
        return rightSpeed;
    }

    private final double rightSpeed;

    public ShooterSpec(double shooterAngle, double leftSpeed, double rightSpeed) {
        this.shooterAngle = shooterAngle;
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
    }
}
