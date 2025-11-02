package frc.robot.hardware;

public interface SparkMaxIO {
    public void setVelocity(double speeds);
    public double getVoltage();
    public double getPosition();
    public double getSpeed();
    public double getCurrent();
    public void setState(double position, double speed);
}
