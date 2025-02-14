// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveForwardTimed extends Command {

  private final DriveTrain driveTrain;
  private boolean finish = false;
  Timer timer;

  public DriveForwardTimed(DriveTrain dt) {
    driveTrain = dt;
    addRequirements(driveTrain);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.reset();
    timer.start();

    /*while(timer.get() < Constants.DriveForwardTime) {
      driveTrain.driveForward(Constants.AutoSpeed);
    }

    finish = true;*/

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (timer.get() < Constants.DriveForwardTime){
      driveTrain.driveForward(Constants.AutoSpeed);
    } else {
      finish = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
