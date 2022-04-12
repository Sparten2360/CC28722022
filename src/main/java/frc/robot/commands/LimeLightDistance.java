// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LLVision;

public class LimeLightDistance extends CommandBase {
  /** Creates a new LimeLightDistance. */
  LLVision limelight;
  DriveTrain drivetrain;
  PIDController LLDistanceController = new PIDController(Constants.LLDKp, Constants.LLDKi,Constants.LLDKd);
  public LimeLightDistance(DriveTrain dt,LLVision ll) {
    limelight = ll;
    drivetrain = dt;
    addRequirements(drivetrain,limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LLDistanceController.setTolerance(1,.0005);
    limelight.setCam(0);
    limelight.setLed(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelight.setCam(0);
    limelight.setLed(3);
    drivetrain.voltDrive((-LLDistanceController.calculate(limelight.getY())), (-LLDistanceController.calculate(limelight.getY())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    limelight.setCam(1);
    limelight.setLed(1);
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (LLDistanceController.atSetpoint()&&(limelight.getV()>0));
  }
}
