// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class index extends CommandBase {
  /** Creates a new index. */
  Indexer indexer;
  public index(Indexer i) {
    indexer = i;
    addRequirements(indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!indexer.getBallReady()){
      indexer.setIndex(Constants.IndexSpeed);
    }
    else{      
      indexer.setIndex(-.05);
      indexer.stop();
    }
    /*boolean ballinIntake = indexer.getballinItake();
    indexer.getBallIntaked();
    if(ballinIntake){
        if(!indexer.getBallReady()){
          indexer.setIndex(Constants.IndexSpeed);
        }
        else{
          indexer.stop();
        }
    }

    /*if(!indexer.getBallReady()){
      indexer.setIndex(Constants.IndexSpeed);
    }
    else{
      indexer.stop();
    }*/
      
      /*else if((DriverStation.getAlliance().equals(Alliance.Blue)&&(indexer.getblue()<indexer.getred()))||(DriverStation.getAlliance().equals(Alliance.Red)&&(indexer.getred()<indexer.getblue()))){
        indexer.setIndex(Constants.shootingSpeed);
      }
      else if((DriverStation.getAlliance().equals(Alliance.Blue)&&(indexer.getBlue2()<indexer.getRed2()))||(DriverStation.getAlliance().equals(Alliance.Red)&&(indexer.getRed2()<indexer.getBlue2()))){
        intake.setIntakeRoller(Constants.OuttakeSpeed);
        intake.setSpinTake(Constants.OuttakeSpeed);
      }*/
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
