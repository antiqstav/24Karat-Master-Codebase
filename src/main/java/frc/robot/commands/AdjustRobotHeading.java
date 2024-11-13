// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class AdjustRobotHeading extends Command {
  private Drivetrain drivetrain = Drivetrain.getInstance();

  double setAngle;
  double currentAngle;

  double frontSpeed;
  double sideSpeed;
  double rotationSpeed;

  double kP;
  double kI;
  double kD;

  PIDController rotController;

  public AdjustRobotHeading(double angleDegrees) {

    setAngle = angleDegrees;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP = 0.0; kI = 0.0; kD = 0.0;
    rotController = new PIDController(kP, kI, kD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = RobotContainer.drivetrain.getHeading();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.swerveDrive(
      frontSpeed, 
      sideSpeed, 
      0, 
      true, 
      new Translation2d(), 
      true
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentAngle == setAngle) {
      return true;
    }
    else if (RobotContainer.driverController.getRightTriggerAxis() == 1.0) {
      return true;
    }
    return false;
  }
}
