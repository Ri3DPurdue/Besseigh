// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drive;

public class RobotContainer {
  private final Drive drive = new Drive();
  private final XboxController controller = new XboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Command arcadeDrive = drive.runWithJoystick(
      () -> MathUtil.applyDeadband(controller.getLeftX(), 0.1),
      () -> MathUtil.applyDeadband(controller.getRightY(), 0.1)
    );
    drive.setDefaultCommand(arcadeDrive);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
