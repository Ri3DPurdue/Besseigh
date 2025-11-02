package frc.robot.hardware;

public class GyroSim implements GyroIO {
    private double angle;
    @Override
    public double getAngle() {
        return angle;
    }

    @Override
    public void updateSim(double velocity, double dt) {
        angle += velocity * dt;
    }
    
}
