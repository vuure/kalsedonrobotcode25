// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Arm extends SubsystemBase {

  SparkAbsoluteEncoder armEncoder;
  SparkMax armMotor;
  PIDController pidArm;
  

  public Arm() {
    armMotor = new SparkMax(Constants.ArmPWM_Num, MotorType.kBrushless);
    armEncoder = armMotor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double arm_motorPower){
    armMotor.set(arm_motorPower);
  }

  public double encoderReading(){
    return armEncoder.getPosition();
  }
  public void stop(){
    armMotor.stopMotor();
  }
}
