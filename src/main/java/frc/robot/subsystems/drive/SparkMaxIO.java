package frc.robot.subsystems.drive;

public interface SparkMaxIO {
    public void setVoltage(double volts);
    public double getVoltage();
    public double getPosition();
    public double getSpeed();
    public void setState(double position, double speed);
}
