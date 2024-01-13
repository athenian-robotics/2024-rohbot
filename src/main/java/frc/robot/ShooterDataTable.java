package frc.robot;

import java.util.TreeMap;

public class ShooterDataTable {
    private final TreeMap<Double, ShooterSpec> dataTable;

    public ShooterDataTable () {
        this.dataTable = new TreeMap<>();
    }

    public void addSpecs(double distance, double shooterAngle, double leftSpeed, double rightSpeed) {
        ShooterSpec toAdd = new ShooterSpec(shooterAngle, leftSpeed, rightSpeed);
        dataTable.put(distance, toAdd);
    }

    public ShooterSpec getSpecs(double distance) {
        if (dataTable.containsKey(distance)) return dataTable.get(distance);

        double prevDist = dataTable.lowerKey(distance);
        double nextDist = dataTable.higherKey(distance);
        ShooterSpec s1 = dataTable.get(prevDist);
        ShooterSpec s2 = dataTable.get(nextDist);
        double diffDist = nextDist - prevDist;
        double weight2 = (nextDist - distance) / diffDist;
        double weight1 = 1 - weight2;
        return new ShooterSpec(
                s1.getShooterAngle() * weight1 + s2.getShooterAngle() * weight2,
                s1.getLeftSpeed() * weight1 + s2.getLeftSpeed() * weight2,
                s1.getRightSpeed() * weight1 + s2.getRightSpeed() * weight2);
    }
}
