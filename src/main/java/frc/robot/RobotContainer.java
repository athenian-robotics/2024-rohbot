package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.controllers.Thrustmaster;
import frc.robot.subsystems.Swerve;
import java.io.File;

public class RobotContainer {
  private final Swerve drivebase = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));

  private final double LEFT_X_DEADBAND = 0.01;
  private final double LEFT_Y_DEADBAND = 0.01;
  private static final Thrustmaster leftThrustmaster = new Thrustmaster(0);
  private static final Thrustmaster rightThrustmaster = new Thrustmaster(1);
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // TODO: Fix this -- likely an issue with offset
    Command driveFieldOrientedDirectAngle =
        drivebase.driveCommand(
            () -> MathUtil.applyDeadband(leftThrustmaster.getY(), LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(leftThrustmaster.getX(), LEFT_X_DEADBAND),
            () -> rightThrustmaster.getX(),
            () -> rightThrustmaster.getY());
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
