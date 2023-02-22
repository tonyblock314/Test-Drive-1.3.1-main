package frc.robot.commands.Elevator;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;


public class UpAndDown extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Elevator m_elevator;
    private BooleanSupplier rightBumperPressed;
    private BooleanSupplier leftBumperPressed;

    public UpAndDown(Elevator elevatorSubsystem, BooleanSupplier rightBumperPressed, BooleanSupplier leftBumperPressed) {
        this.m_elevator = elevatorSubsystem;
        this.rightBumperPressed = rightBumperPressed;
        this.leftBumperPressed = leftBumperPressed;
        //use addRequirements() here to declare subsystem dependencies.
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
        double speed = 0.2;
        boolean RBumperAsBoolean = rightBumperPressed.getAsBoolean();
        boolean LBumperAsBoolean = leftBumperPressed.getAsBoolean();
        if (RBumperAsBoolean = true) {
            Elevator.leadMotor.set(speed);
        } else if (LBumperAsBoolean = true) {
            Elevator.leadMotor.set(-speed);
        } else {
            Elevator.leadMotor.set(0);
        }
    }

    // Caleed once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns when the command should end.

}
