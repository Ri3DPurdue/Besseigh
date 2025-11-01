package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private DifferentialDrive drive;
    
    private SparkMax leftMotor;
    private SparkMax rightMotor;
    private SparkMax cosmeticLeft;
    private SparkMax cosmeticRight;

    private int leftMotorId = 0;
    private int rightMotorId = 1;
    private int cosmeticLeftId = 2;
    private int cosmeticRightId = 3;

    public Drive() {
        leftMotor = new SparkMax(leftMotorId, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorId, MotorType.kBrushless);
        cosmeticLeft = new SparkMax(cosmeticLeftId, MotorType.kBrushless);
        cosmeticRight = new SparkMax(cosmeticRightId, MotorType.kBrushless);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(20);

        leftMotor.configure(new SparkMaxConfig().apply(globalConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(new SparkMaxConfig().apply(globalConfig).inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        cosmeticLeft.configure(new SparkMaxConfig().apply(globalConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        cosmeticRight.configure(new SparkMaxConfig().apply(globalConfig).inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        drive = new DifferentialDrive(
            (double leftOuput) -> {
                leftMotor.set(leftOuput);
            }, 
            (double rightOutput) -> {
                rightMotor.set(rightOutput);
            });
    }

    public Command runWithJoystick(DoubleSupplier linear, DoubleSupplier rotational) {
        return this.run(() -> {
            drive.arcadeDrive(linear.getAsDouble(), rotational.getAsDouble());
            cosmeticLeft.set(0.5 * Math.signum(linear.getAsDouble()));
            cosmeticRight.set(0.5 * Math.signum(linear.getAsDouble()));
        });
    }
}
