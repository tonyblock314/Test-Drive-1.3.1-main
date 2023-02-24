package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PistonClaw extends SubsystemBase{
    private final DoubleSolenoid Claw;

    public PistonClaw() {
        Claw = new DoubleSolenoid(null, 1, 2);
    }
        
    public void retract() {
        Claw.set(DoubleSolenoid.Value.kReverse);
    }
        
    public void extend() {
        Claw.set(DoubleSolenoid.Value.kForward);
    }
}
