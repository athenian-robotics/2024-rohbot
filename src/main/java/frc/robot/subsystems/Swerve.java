package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Swerve {
  private static final double MAX_SPEED = 0.0; // TODO: real value
  private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
  private SwerveDrive swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(MAX_SPEED);

  public Swerve() throws IOException {

  }


}
