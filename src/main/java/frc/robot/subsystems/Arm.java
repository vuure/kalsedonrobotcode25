// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends SubsystemBase {

  WPI_TalonSRX elevator;
  

  public Arm() {
    elevator = new WPI_TalonSRX(Constants.ElevatorCAN_Num);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double arm_motorPower){
    elevator.set(arm_motorPower);
  }

  public void stop(){
    elevator.stopMotor();
  }
}
