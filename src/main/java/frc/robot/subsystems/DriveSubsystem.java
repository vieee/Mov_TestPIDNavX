// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSubsystem extends SubsystemBase {
  private final WPI_TalonSRX FR;
  private final WPI_TalonSRX BR;
  private final SpeedControllerGroup rightSide;

  private final WPI_TalonSRX FL;
  private final WPI_TalonSRX BL;  
  private final SpeedControllerGroup leftSide;

  private final DifferentialDrive driveTrain;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    FR = new WPI_TalonSRX(3);
    BR = new WPI_TalonSRX(4);
    rightSide = new SpeedControllerGroup(FR, BR);

    FL = new WPI_TalonSRX(1);
    BL = new WPI_TalonSRX(2);
    leftSide = new SpeedControllerGroup(FL, BL);

    driveTrain = new DifferentialDrive(leftSide, rightSide);
    driveTrain.setSafetyEnabled(true);
    driveTrain.setExpiration(0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(final double l, final double r) {
    this.rightSide.setInverted(true);
    
    FR.set(r);
    BR.set(r);
    FL.set(l);
    BL.set(l);
  }

  public void arcadeInbuilt(final double y, final double z) {
    this.rightSide.setInverted(true);
    // System.out.println("Speed: " + y + " " + z);

    driveTrain.arcadeDrive(y * Constants.maxSpeed, z * Constants.maxTurnSpeed);
  }

  public void moveByAngle(double correction) {
    if (Math.abs(correction) < 0.05) correction = Math.signum(correction) * 0.05;
    if (Math.abs(correction) > 0.5) correction = Math.signum(correction) * 0.3;
    drive(-1 * correction, -1 * correction);
  }

  double integralEncoder = 0;
  double error = 0;
  double previousErrorEncoder = 0;

  public void drivePIDNavX(double yaxis) {
    double navxYaw = RobotContainer.navx.getYaw();
    error = navxYaw;
    // double navxAngle = RobotContainer.navx.getAngle();
    // SmartDashboard.putNumber("YAW RATE", navxYawRate);
    SmartDashboard.putNumber("YAW", navxYaw);
    // SmartDashboard.putNumber("ANGLE", navxAngle);
    
    // Calculate Correction
    integralEncoder += error;
    if (yaxis < Constants.integralResetBound) integralEncoder = 0;

    double derivative = error - previousErrorEncoder;
    previousErrorEncoder = error;

    double correction = ((error * Constants.kPEncoder) + (integralEncoder * Constants.kIEncoder) + (derivative * Constants.kDEncoder));

    // Apply the Correction to Motor Voltages
    double leftSpeed = yaxis - correction;
    double rightSpeed = yaxis + correction;

    // System.out.println("leftSpeed: " + leftSpeed + " rightSpeed: " + rightSpeed);
    // SmartDashboard.putNumber("Left PID", leftSpeed);
    // SmartDashboard.putNumber("Right PID", rightSpeed);

    drive(leftSpeed * Constants.maxSpeed, rightSpeed * Constants.maxSpeed);

  }
}
