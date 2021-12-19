// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  DriveSubsystem driveSubsystem;
  private final Supplier<Double> speedFunction, turnFunction;

  public DriveCommand(DriveSubsystem m_driveSubsystem, Supplier<Double> speedFunction, Supplier<Double> turnFunction) {
    this.driveSubsystem = m_driveSubsystem;
    this.speedFunction = speedFunction;
    this.turnFunction = turnFunction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Drive Started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double realTimeSpeed = -1 * speedFunction.get();
    double realTimeTurn = turnFunction.get();

    if (Math.abs(realTimeTurn) > Constants.turnThreshold) {
      driveSubsystem.arcadeInbuilt(realTimeSpeed, realTimeTurn);
    } else {
      if (realTimeSpeed != 0) driveSubsystem.drivePIDNavX(realTimeSpeed);
      else driveSubsystem.drive(0, 0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Drive ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
