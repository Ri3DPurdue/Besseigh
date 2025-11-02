package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.config.SparkMaxConfig;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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

public class Drive extends SubsystemBase {
    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveFeedforward feedforward;
    private DifferentialDrivetrainSim sim;
    private SparkMaxIO leftMotor;
    private SparkMaxIO rightMotor;
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
    private final double baseRadius = trackWidth / 2;

    public Drive() {
        kinematics = new DifferentialDriveKinematics(trackWidth);
        feedforward = new DifferentialDriveFeedforward(
            gearing / (model.KvRadPerSecPerVolt * wheelRadius), 
            (model.rOhms * mass * wheelRadius) / (2 * gearing * model.KtNMPerAmp), 
            gearing / (model.KvRadPerSecPerVolt * 2), 
            (model.rOhms * momentOfInertia * wheelRadius) /  (2 * gearing * model.KtNMPerAmp * baseRadius * baseRadius)
        );
        sim = new DifferentialDrivetrainSim(model, gearing, momentOfInertia, mass, wheelRadius, trackWidth, VecBuilder.fill(0, 0, 0, 0, 0, 0, 0));
        if (RobotBase.isReal()) {
            SparkMaxConfig config = new SparkMaxConfig();
            config.smartCurrentLimit(20);
            config.encoder.positionConversionFactor(2 * Math.PI / gearing * wheelRadius);
            config.encoder.velocityConversionFactor(2 * Math.PI / gearing * wheelRadius / 60);
            leftMotor = new SparkMaxReal(leftID, new SparkMaxConfig().apply(config).inverted(false));
            rightMotor = new SparkMaxReal(rightID, new SparkMaxConfig().apply(config).inverted(true));
        } else {
            leftMotor = new SparkMaxSim();
            rightMotor = new SparkMaxSim();
        }
        SmartDashboard.putData("Drive Speeds", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("Linear Speed", () -> ntLinearSpeed, value -> ntLinearSpeed = value);
                builder.addDoubleProperty("Rotational Speed", () -> ntRotationalSpeed, value -> ntRotationalSpeed = value);
            }
        });
    }

    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        DifferentialDriveWheelVoltages voltages = feedforward.calculate(leftMotor.getSpeed(), wheelSpeeds.leftMetersPerSecond, rightMotor.getSpeed(), wheelSpeeds.rightMetersPerSecond, 0.02);
        leftMotor.setVoltage(voltages.left);
        rightMotor.setVoltage(voltages.right);
    }
    
    @Override
    public void simulationPeriodic() {
        leftMotor.setState(sim.getLeftPositionMeters(), sim.getLeftVelocityMetersPerSecond());
        rightMotor.setState(sim.getRightPositionMeters(), sim.getRightVelocityMetersPerSecond());
    }
    
    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            sim.setInputs(0, 0);
        } else {
            sim.setInputs(leftMotor.getVoltage(), rightMotor.getVoltage());
        }
        sim.update(0.02);
        DogLog.log("Pose", sim.getPose());
        DogLog.log("Left Speed", leftMotor.getSpeed());
        DogLog.log("Right Speed", rightMotor.getSpeed());
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

    public Command driveInCircle(double linearSpeed, double angularSpeed) {
        return driveCommand(() -> linearSpeed, () -> angularSpeed);
    }

    public Command driveInCircleExact(double diameter, double velocity) {
        double outer = (2 * diameter * velocity) / (2 * diameter - trackWidth);
        double inner = 2 * velocity - outer;
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(inner, outer));
        return run(() -> drive(speeds));
    }

}
