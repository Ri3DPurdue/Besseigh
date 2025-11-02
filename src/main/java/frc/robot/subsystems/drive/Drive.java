package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.GyroIO;
import frc.robot.hardware.GyroSim;
import frc.robot.hardware.SparkMaxIO;
import frc.robot.hardware.SparkMaxReal;
import frc.robot.hardware.SparkMaxSim;

public class Drive extends SubsystemBase {
    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveOdometry odometry;
    private DifferentialDrivetrainSim sim;
    private SparkMaxIO leftMotor;
    private SparkMaxIO rightMotor;
    private GyroIO gyro;
    private double ntLinearSpeed;
    private double ntRotationalSpeed;

    // Physical Constants
    private final int leftID = 1;
    private final int rightID = 2;
    private final double gearing = 15;
    private final DCMotor model = DCMotor.getNEO(1);
    private final double wheelRadius = Inches.of(4).in(Meters);
    private final double mass = 50;
    private final double momentOfInertia = 3;
    private final double trackWidth = Inches.of(20).in(Meters);

    public Drive() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(20);
        config.encoder.positionConversionFactor(2 * Math.PI / gearing * wheelRadius);
        config.encoder.velocityConversionFactor(2 * Math.PI / gearing * wheelRadius / 60);
        // Sim Values: p = 0.3, ff = 0.25
        config.closedLoop.pidf(0.3, 0, 0, 0.25);
        if (RobotBase.isReal()) {
            leftMotor = new SparkMaxReal(leftID, new SparkMaxConfig().apply(config).inverted(false));
            rightMotor = new SparkMaxReal(rightID, new SparkMaxConfig().apply(config).inverted(true));
        } else {
            leftMotor = new SparkMaxSim(leftID, new SparkMaxConfig().apply(config).inverted(false), DCMotor.getNEO(1));
            rightMotor = new SparkMaxSim(rightID, new SparkMaxConfig().apply(config).inverted(true), DCMotor.getNEO(1));
        }
        gyro = new GyroSim();
        kinematics = new DifferentialDriveKinematics(trackWidth);
        odometry = new DifferentialDriveOdometry(Rotation2d.fromRadians(gyro.getAngle()), leftMotor.getPosition(), rightMotor.getPosition());
        sim = new DifferentialDrivetrainSim(model, gearing, momentOfInertia, mass, wheelRadius, trackWidth, VecBuilder.fill(0, 0, 0, 0, 0, 0, 0));
        SmartDashboard.putData("Drive Speeds", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Linear Speed", () -> ntLinearSpeed, value -> ntLinearSpeed = value);
                builder.addDoubleProperty("Rotational Speed", () -> -ntRotationalSpeed, value -> ntRotationalSpeed = -value);
            }
        });
    }

    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        leftMotor.setVelocity(wheelSpeeds.leftMetersPerSecond);
        rightMotor.setVelocity(wheelSpeeds.rightMetersPerSecond);
        DogLog.log("Left Setpoint", wheelSpeeds.leftMetersPerSecond);
        DogLog.log("Right Setpoint", wheelSpeeds.rightMetersPerSecond);
    }
    
    @Override
    public void simulationPeriodic() {
        if (DriverStation.isDisabled()) {
            sim.setInputs(0, 0);
        } else {
            sim.setInputs(leftMotor.getVoltage(), rightMotor.getVoltage());
        }
        sim.update(0.02);
        leftMotor.setState(sim.getLeftPositionMeters(), sim.getLeftVelocityMetersPerSecond());
        rightMotor.setState(sim.getRightPositionMeters(), sim.getRightVelocityMetersPerSecond());
        gyro.updateSim(kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftMotor.getSpeed(), rightMotor.getSpeed())).omegaRadiansPerSecond, 0.02);
    }
    
    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromRadians(gyro.getAngle()), new DifferentialDriveWheelPositions(leftMotor.getPosition(), rightMotor.getPosition()));
        DogLog.log("Odometry Pose", odometry.getPoseMeters());
        DogLog.log("Left Speed", leftMotor.getSpeed());
        DogLog.log("Left Current", leftMotor.getCurrent());
        DogLog.log("Left Voltage", leftMotor.getVoltage());
        DogLog.log("Right Speed", rightMotor.getSpeed());
        DogLog.log("Right Current", rightMotor.getCurrent());
        DogLog.log("Right Voltage", rightMotor.getVoltage());
    }

    public Command driveCommand(DoubleSupplier linearSpeed, DoubleSupplier angularSpeed) {
        return run(() -> {
            drive(new ChassisSpeeds(linearSpeed.getAsDouble(), 0, angularSpeed.getAsDouble()));
        });
    }

    public Command joystickDrive(XboxController xbox) {
        return driveCommand(
            () -> MathUtil.applyDeadband(-xbox.getLeftY(), 0.2),
            () -> MathUtil.applyDeadband(-xbox.getRightX(), 0.2)
        );
    }

    public Command ntDrive() {
        return driveCommand(() -> ntLinearSpeed, () -> ntRotationalSpeed);
    }

    public Command driveInCircle(double diameter, double velocity, boolean counterclockwise) {
        double radius = diameter / 2;
        double inner = velocity * (radius - trackWidth / 2) / radius;
        double outer = velocity * (radius + trackWidth / 2) / radius;
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(counterclockwise ? inner : outer, counterclockwise ? outer : inner));
        return run(() -> drive(speeds));
    }

}
