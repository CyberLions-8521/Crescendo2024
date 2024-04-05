// Mr. Vu playing with a mini-superstructure that represents the elevator-hoodwrist-joint subsystems
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.ElevatorGoToSetpoint;
import frc.robot.commands.HoodWristGoToSetpoint;
import frc.robot.commands.JointGoToSetpoint;

public class ElevatedHoodedJoint {
    public static Command goSideSpeaker(Elevator e, HoodWrist hw, Joint j) {
        return new ElevatorGoToSetpoint(e, 2)
            .alongWith(new HoodWristGoToSetpoint(hw, 0))
            .alongWith(new JointGoToSetpoint(32, 0, j));
    }
    public static Command goMidSpeakerCommand(Elevator e, HoodWrist hw, Joint j) {
        return new ElevatorGoToSetpoint(e, 2)
            .alongWith(new HoodWristGoToSetpoint(hw, 0))
            .alongWith(new JointGoToSetpoint(29, 0, j));
    }
}
