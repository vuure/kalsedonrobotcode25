// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;


public class ControlArmWithJoystick extends Command {

  private final Arm Arm;
  double power;

  public ControlArmWithJoystick(Arm arm) {
    Arm = arm;
    addRequirements(Arm);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    power = RobotContainer.driverJoystick.getRawAxis(Constants.RightTrigger_Axis) + (RobotContainer.driverJoystick.getRawAxis(Constants.LeftTrigger_Axis) * -1);
    Arm.setPower(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
