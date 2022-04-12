// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LLVision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAim extends PIDCommand {
  /** Creates a new AutoAim.  This function uses the x angle diffrence between the default postion of the limelight and the target
   * It uses a PID loop to do this since that is the easiest way to speed up the process as the non PID vesrion is slow 
  */
  LLVision limelight;
  public AutoAim(DriveTrain dt,LLVision ll) {
    super(
        // The controller that the command will use the P value is tuned first if you can get the motion to occur just through p usage do so as this will simplify the loop
        //if the speed you desire for the loop causes your P value to oscilate(overadjust up and down for ever) add a D value and increase until you reach setpoint fast
        //Most games will contain a vision target so this command can be used for those years until we get to the point where it needs to be upgraded
        new PIDController(0.3, 0, 0),
        // This should return the measurement since it's from the limelight we call that subsytem and then the get command for our variable you have to use :: instead of .
        ll::getX,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          
          dt.voltDrive(-output, output);// Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    limelight = ll;
    limelight.setCam(0);
    limelight.setLed(3);
    getController().enableContinuousInput(-32.9, 32.9);
    getController().setTolerance(1);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(getController().atSetpoint()){
      limelight.setCam(1);
      limelight.setLed(1);
    }
    return getController().atSetpoint();
  }
}
 