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
public class AutoDistance extends PIDCommand {
  /** Creates a new AutoDistance. */
  private final LLVision limelight;
  public AutoDistance(LLVision ll, DriveTrain dt) {
    super(
        // The controller that the command will use
        new PIDController(0.01, 0, 0),
        // This should return the measurement
        ll::getY,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          dt.voltDrive(output, output);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    limelight = ll;
    limelight.setCam(0);
    limelight.setLed(3);
    getController().setTolerance(1.0);
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
