// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;

public class HighShotAuto extends CommandBase {
  /** Creates a new HighShotAuto. */
  Indexer indexer;
  Flywheel flywheel;
  public HighShotAuto(Indexer i, Flywheel f) {
    indexer = i;
    flywheel = f;
    addRequirements(flywheel, indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheel.resetPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheel.setTopFlywheelRPM(4500);
    flywheel.setBottomFlywheelRPM(4500);
    if(flywheel.topflywheelAtSpeed()&&flywheel.bottomrflywheelAtSpeed()){
      indexer.setIndex(Constants.shootingSpeed);
    }
    else{
      indexer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    indexer.stop();
    flywheel.resetPID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !indexer.getBallReady();
  }
}
