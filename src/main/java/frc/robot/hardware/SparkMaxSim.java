package frc.robot.hardware;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.system.plant.DCMotor;

public class SparkMaxSim implements SparkMaxIO {
    private SparkMax motor;
    private com.revrobotics.sim.SparkMaxSim sim;

    public SparkMaxSim(int id, SparkBaseConfig config, DCMotor model) {
        motor = new SparkMax(id, MotorType.kBrushless);
        sim = new com.revrobotics.sim.SparkMaxSim(motor, model);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVelocity(double speed) {
        motor.getClosedLoopController().setReference(speed, ControlType.kVelocity);
    }

    @Override
    public double getVoltage() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public double getSpeed() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    @Override
    public void setState(double position, double speed) {
        sim.iterate(speed, 12, 0.02);
    }


}
