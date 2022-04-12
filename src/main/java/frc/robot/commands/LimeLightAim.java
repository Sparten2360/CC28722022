// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LLVision;

public class LimeLightAim extends CommandBase {
  /** Creates a new LimeLightAim. */
  DriveTrain drivetrain;
  LLVision limelight;
  private final PIDController LLaimPIDController= new PIDController(Constants.LLKp, Constants.LLKi, Constants.LLKd);
  public LimeLightAim(LLVision ll, DriveTrain dt) {
    drivetrain = dt;
    limelight = ll;
    addRequirements(drivetrain,limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LLaimPIDController.setTolerance(.5,.00125);
    
    LLaimPIDController.enableContinuousInput(-32.9, 32.9);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limelight.setCam(0);
    limelight.setLed(3);
    drivetrain.voltDrive(-LLaimPIDController.calculate(limelight.getX()), LLaimPIDController.calculate(limelight.getX()));
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
    return (LLaimPIDController.atSetpoint()&&(limelight.getV()>0));
  }
}
