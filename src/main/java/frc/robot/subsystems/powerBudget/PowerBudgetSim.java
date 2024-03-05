package frc.robot.subsystems.powerBudget;

import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;

import java.util.HashMap;
import java.util.Map;

public class PowerBudgetSim extends SubsystemBase implements PowerBudgetIO {
    protected static final String logKey = "CurrentDrawSim";
    private static final int RESERVED_CHANNEL = -1;
    private static final double LIMIT = 100;

    private final LoggedPowerDistribution powerDistribution;
    private final HashMap<Integer, Double> currentDraws;
    private boolean isDataOld = false;

    public PowerBudgetSim() {
        super();

        this.powerDistribution = LoggedPowerDistribution.getInstance();
        this.currentDraws = new HashMap<>(Map.of(RESERVED_CHANNEL, 0d));
    }

    private double getAdjustedCurrentDrawAmps(double currentDrawAmps, int channel) {
        if (channel == RESERVED_CHANNEL) {
            throw new IllegalArgumentException("Attempted to report current to RESERVED_CHANNEL!");
        }

        final LoggedPowerDistribution.PowerDistributionInputs inputs = powerDistribution.getInputs();
        final double adjustedCurrentDrawAmps;
        if (channel >= 1 && channel <= inputs.channelCount && channel <= inputs.pdpChannelCurrents.length) {
            adjustedCurrentDrawAmps = currentDrawAmps + inputs.pdpChannelCurrents[channel - 1];
        } else {
            adjustedCurrentDrawAmps = currentDrawAmps;
        }

        return adjustedCurrentDrawAmps;
    }

    public void report(final double currentDrawAmps, final int... channels) {
        final int nChannels = channels.length;
        for (final int channel : channels) {
            final double averageCurrentDrawAmps = currentDrawAmps / nChannels;
            final double adjustedCurrentDrawAmps = getAdjustedCurrentDrawAmps(averageCurrentDrawAmps, channel);

            if (currentDraws.put(channel, adjustedCurrentDrawAmps) != null && !isDataOld) {
                throw new RuntimeException("Attempted to report duplicate channel within 1 loop period!");
            }
        }

        if (isDataOld && channels.length > 0) {
            isDataOld = false;
        }
    }

    public void report(final double currentDrawAmps) {
        if (isDataOld) {
            this.isDataOld = false;
        }

        currentDraws.merge(RESERVED_CHANNEL, currentDrawAmps, Double::sum);
    }

    public double getTotalCurrentDraw() {
        return currentDraws.values().stream()
                .mapToDouble(Double::doubleValue)
                .sum();
    }

    @Override
    public void periodic() {
        this.isDataOld = true;
        Logger.recordOutput(logKey + "/TotalCurrentDraw", getTotalCurrentDraw());
        Logger.recordOutput(logKey + "/CurrentDraws", currentDraws.values().stream()
                .mapToDouble(Double::doubleValue)
                .toArray()
        );

        RoboRioSim.setVInVoltage(
                Math.max(
                        BatterySim.calculateLoadedBatteryVoltage(
                                12,
                               0.02,
                                getTotalCurrentDraw()
                        ),
                        0
                )
        );
    }

    @Override
    public void updateInputs(PowerBudget inputs) {
        inputs.currentlyUsing = getTotalCurrentDraw();
        inputs.batteryVoltage = RoboRioSim.getVInVoltage();
    }

    @Override
    public boolean hasCurrent(double currentlyUsing, double wantToUse) {
        return getTotalCurrentDraw() - currentlyUsing < LIMIT - wantToUse;
    }
}