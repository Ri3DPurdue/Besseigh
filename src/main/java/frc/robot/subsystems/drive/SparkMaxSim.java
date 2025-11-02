package frc.robot.subsystems.drive;

public class SparkMaxSim implements SparkMaxIO {
    private double volts = 0;
    private double position = 0;
    private double speed = 0;
    @Override
    public void setVoltage(double volts) {
        this.volts = volts;
    }

    @Override
    public double getVoltage() {
        return volts;
    }

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public double getSpeed() {
        return speed;
    }

    @Override
    public void setState(double position, double speed) {
        this.position = position;
        this.speed = speed;
    }


}
