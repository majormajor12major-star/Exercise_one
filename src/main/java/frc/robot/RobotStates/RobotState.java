package frc.robot.RobotStates;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Components.PositionComponent;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.JoystickControl;

public class RobotState {
    /**
     * Sets up basic control for the robot using an Xboxcontroller. Also sets up the A button as a reset for position.
     * @param The xboxcontroller being used for driving.
     */
    public static void Initialize(XboxController controller) {

        Drivetrain.getInstance().setDefaultCommand(new JoystickControl(() -> {
            return -1* controller.getLeftY();
        }, () -> {
            return  -1 * controller.getLeftX();
        }, () -> {
            return -1* controller.getRightX();
        }
        ));

        new JoystickButton(controller, XboxController.Button.kA.value).onTrue(new InstantCommand(PositionComponent::zeroPose));
    }
}
