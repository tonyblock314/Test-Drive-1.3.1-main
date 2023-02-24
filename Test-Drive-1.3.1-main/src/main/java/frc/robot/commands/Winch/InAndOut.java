// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Winch;

import frc.robot.Constants;
import frc.robot.subsystems.Winch;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class InAndOut extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Winch m_subsystem;
  private DoubleSupplier rtTrig;
  private DoubleSupplier ltTrig;

  public InAndOut(Winch winchsubsystem, DoubleSupplier rightTriggerDepression, DoubleSupplier leftTriggerDepression) {
    m_subsystem = winchsubsystem;
    this.rtTrig = rightTriggerDepression;
    this.ltTrig = leftTriggerDepression;
    addRequirements(winchsubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double upspeed;
    if(rtTrig.getAsDouble() > Constants.RT_DEADBAND){
      upspeed = rtTrig.getAsDouble() * Constants.MAX_INANDOUT_SPEED;
    }
    else{
      upspeed = 0;
    }
    System.out.println("Setting upspeed to " + upspeed);

    double downspeed = ltTrig.getAsDouble() > Constants.LT_DEADBAND ? ltTrig.getAsDouble() * Constants.MAX_INANDOUT_SPEED : 0;
    System.out.println("Setting downspeed to " + downspeed);

    m_subsystem.setWheelSpeed(upspeed - downspeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
