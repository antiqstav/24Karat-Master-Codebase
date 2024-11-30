// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;
import frc.robot.Constants.VisionPID;

public class AdjustRobotHeading extends Command {
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private final PhotonCamera v = new PhotonCamera("Arducam_OV9281_USB_Camera (1)");

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
    kP = VisionPID.KP_VISIONROT;
    kI = VisionPID.KI_VISIONROT;
    kD = VisionPID.KD_VISIONROT;
    rotController = new PIDController(kP, kI, kD);
    frontSpeed = sideSpeed = 0;
    rotationSpeed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = 1;
    boolean targetVisible = false;
    double targetYaw = 0.0;
    PhotonPipelineResult res = v.getLatestResult();
    if (res.hasTargets()) {
      PhotonTrackedTarget target = res.getBestTarget();
      targetYaw = target.getYaw();
      targetVisible = true;
    }

    if (targetVisible) {
      rotationSpeed = -1.0 * targetYaw * kP * Constants.SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;
    }

    drivetrain.swerveDrive(0,0,rotationSpeed,true, // !RobotContainer.driverController.getHID().getRawButton(XboxController.Button.kB.value)
  new Translation2d(),true);
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
        true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentAngle == setAngle) {
      return true;
    } else if (RobotContainer.driverController.getRightTriggerAxis() == 1.0) {
      return true;
    }
    return false;
  }
}
