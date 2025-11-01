package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private DifferentialDrive drive;
    
    private double ntLinearSpeed;
    private double ntRotationalSpeed;

    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private final int leftMotorId = 9;
    private final int rightMotorId = 10;
    private final double gearing = 15;
    private final double wheelRadius = Units.inchesToMeters(8 / 2);
    private final double trackWidth = Units.inchesToMeters(20);
    private final double maxAutoSpeed = 0.2 * Units.rotationsPerMinuteToRadiansPerSecond(5676) * wheelRadius;

    public Drive() {
        leftMotor = new SparkMax(leftMotorId, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorId, MotorType.kBrushless);

        SmartDashboard.putData("Drive Speeds", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Linear Speed", () -> ntLinearSpeed, value -> ntLinearSpeed = value);
                builder.addDoubleProperty("Rotational Speed", () -> ntRotationalSpeed, value -> ntRotationalSpeed = value);
            }
        });

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(20);
        globalConfig.encoder.positionConversionFactor(1 / gearing / (Math.PI * 2) * wheelRadius);
        globalConfig.encoder.velocityConversionFactor(1 / gearing / (Math.PI * 2) / 60 * wheelRadius);

        leftMotor.configure(new SparkMaxConfig().apply(globalConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(new SparkMaxConfig().apply(globalConfig).inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        });
    }

    public Command runWithNetworktables() {
        return run(() -> {
            drive.arcadeDrive(ntLinearSpeed, ntRotationalSpeed);
        });
    }

    public Command driveInCircle(Distance diameter, Time time) {
        double maxDiameter = maxAutoSpeed * time.in(Seconds);
        if (diameter.in(Meters) > maxDiameter) {
            return driveInCircle(Meters.of(maxDiameter), time);
        }
        double innerDistance = Math.PI * (diameter.in(Meters) - trackWidth / 2);
        double outerDistance = Math.PI * (diameter.in(Meters) + trackWidth / 2);
        double duration = time.in(Seconds);
        return run(() -> {
            leftMotor.getClosedLoopController().setReference(innerDistance / duration, ControlType.kVelocity);
            rightMotor.getClosedLoopController().setReference(outerDistance / duration, ControlType.kVelocity);
        }).withTimeout(duration).andThen(() -> {
            leftMotor.set(0);
            rightMotor.set(0);
        });
    }
}
