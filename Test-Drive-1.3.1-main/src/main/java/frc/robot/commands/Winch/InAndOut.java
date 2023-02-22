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
  private DoubleSupplier speed1;
  private DoubleSupplier speed2;

    public InAndOut(Winch winchsubsystem, DoubleSupplier rightTriggerDepression, DoubleSupplier leftTriggerDepression) {
      m_subsystem = winchsubsystem;
      this.speed1 = rightTriggerDepression;
      this.speed2 = leftTriggerDepression;
      addRequirements(winchsubsystem);
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double upspeed = speed1.getAsDouble() > Constants.LT_DEADBAND ? speed1.getAsDouble() * Constants.MAX_INANDOUT_SPEED : 0;
    double downspeed = speed2.getAsDouble() > Constants.LT_DEADBAND ? speed2.getAsDouble() * Constants.MAX_INANDOUT_SPEED : 0;
    m_subsystem.setWheelSpeed(upspeed - downspeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
