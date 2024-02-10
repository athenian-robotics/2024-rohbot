package frc.robot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

public record ShooterSpec(Measure<Angle> angle, double speedL, double speedR, Measure<Angle> offset) {}
