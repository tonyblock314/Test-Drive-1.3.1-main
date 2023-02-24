package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;


public class Elevator extends SubsystemBase{
    private static final MotorType kMotorType = MotorType.kBrushless;

    public static CANSparkMax leadMotor = new CANSparkMax(Constants.Leader_SparkMax_ID, kMotorType);
    public static CANSparkMax followMotor = new CANSparkMax(Constants.Follower_SparkMax_ID, kMotorType);


    public Elevator() {
        /**
        * The RestoreFactoryDefaults method can be used to reset the configuration parameters
        * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        leadMotor.restoreFactoryDefaults();
        followMotor.restoreFactoryDefaults();

        /**
         * In CAN mode, one SPARK MAX can be configured to follow another. This is done by calling
         * the follow() method on the SPARK MAX you want to configure as a follower, and by passing
         * as a parameter the SPARK MAX you want to configure as a leader.
         */
        followMotor.follow(leadMotor, true);

    }
    public void set(double speed) {
        leadMotor.set(speed);
    }
}
