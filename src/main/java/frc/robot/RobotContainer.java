package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.controllers.Thrustmaster;
import frc.robot.subsystems.Swerve;
import java.io.File;

public class RobotContainer {
  private final Swerve drivebase = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final double LEFT_X_DEADBAND = 0.001;
  private final double LEFT_Y_DEADBAND = 0.001;
  private static final Thrustmaster leftThrustmaster = new Thrustmaster(0);
  private static final Thrustmaster rightThrustmaster = new Thrustmaster(1);

  public RobotContainer() {
    Command driveFieldOrientedDirectAngle =
        drivebase.driveCommand(
            () -> leftThrustmaster.getY(),
            () -> leftThrustmaster.getX(),
            () -> rightThrustmaster.getX());
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
