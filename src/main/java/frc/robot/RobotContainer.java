// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class RobotContainer {
  private final Drive drive = new Drive();
  private final XboxController controller = new XboxController(0);

  public RobotContainer() {
    configureBindings();
    DogLog.setEnabled(true);
  }

  private void configureBindings() {
    Command arcadeDrive = drive.joystickDrive(controller);
    Command ntDrive = drive.ntDrive();
    drive.setDefaultCommand(arcadeDrive);
  }

  public Command getAutonomousCommand() {
    return drive.driveInCircle(1.5, 0.5, true);
  }
}
