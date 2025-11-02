package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

public class SparkMaxReal implements SparkMaxIO {
    private SparkMax motor;

    public SparkMaxReal(int iD, SparkBaseConfig config) {
        motor = new SparkMax(iD, MotorType.kBrushless);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
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
    public void setState(double position, double speed) {}
}
