package frc.robot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public record ShooterSpec(
    Measure<Angle> angle,
    Measure<Velocity<Angle>> speedL,
    Measure<Velocity<Angle>> speedR,
    Measure<Angle> offset) {}
