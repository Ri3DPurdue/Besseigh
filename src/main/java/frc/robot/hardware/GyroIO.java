package frc.robot.hardware;

public interface GyroIO {
    public double getAngle();
    public void updateSim(double velocity, double dt);
}
