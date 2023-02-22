package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;


public class UpAndDown extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Elevator m_elevator;
    private final double speed;

    public UpAndDown(Elevator elevatorSubsystem, double speed) {
        m_elevator = elevatorSubsystem;
        this.speed = speed;
        addRequirements(m_elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Starting Elevator Commads");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_elevator.set(speed);
    }

    // Caleed once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns when the command should end.

}
