package frc.robot.Scratchbot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Drivetrain.GoToPoint;
import frc.robot.Drivetrain.JoystickControl;

public class Scratchbot {
    private static SequentialCommandGroup toRun = new SequentialCommandGroup();
    /**
     * Moves the robot forward by a specified distance in meters.
     * @param metersToMove The distance in meters to move the robot forward.
     */
    public static void sb_moveForward(double metersToMove){
        toRun.addCommands(new GoToPoint(metersToMove, 0, new Rotation2d(0), true));
    }
    /**
     * Moves the robot left by a specified distance in meters.
     * @param metersToMove The distance in meters to move the robot left.
     */
    public static void sb_moveLeft(double metersToMove){
        toRun.addCommands(new GoToPoint(0, metersToMove, new Rotation2d(0),true));
    }
    /**
     * Moves the robot right by a specified distance in meters.
     * @param metersToMove The distance in meters to move the robot right.s
     */
    public static void sb_moveRight(double metersToMove){
        toRun.addCommands(new GoToPoint(0,-metersToMove,new Rotation2d(0), true));
    }
    /**
     * Moves the robot backward by a specified distance in meters.
     * @param metersToMove The distance in meters to move the robot backward.
     */
    public static void sb_moveBackward(double metersToMove){
        toRun.addCommands(new GoToPoint(-metersToMove,0,new Rotation2d(0), true));
    }
    /**
     * Turns the robot clockwise by a specified angle in radians.
     * @param radiansToTurn The angle in radians to turn the robot clockwise.
     */
    public static void sb_turnClockwise(double radiansToTurn){
        toRun.addCommands(new GoToPoint(0,0,new Rotation2d(-radiansToTurn), true));
    }
    /**
     * Turns the robot counter-clockwise by a specified angle in radians.
     * @param radiansToTurn The angle in radians to turn the robot counter-clockwise.
     */
    public static void sb_turnCounterClockwise(double radiansToTurn){
        toRun.addCommands(new GoToPoint(0,0,new Rotation2d(radiansToTurn), true));
    }
    /**
     * @return The value of the left joystick's X axis of the controller.
     */
    public static double sb_getControllerLeftX(){
        return Robot.controller.getLeftX();
    }
    /**
     * @return The value of the left joystick's Y axis of the controller.
     */
    public static double sb_getControllerLeftY(){
        return Robot.controller.getLeftY();
    }
    /**
     * @return The value of the right joystick's X axis of the controller.
     */
    public static double sb_getControllerRightX(){
        return Robot.controller.getRightX();
    }
    /**
     * @return The value of the right joystick's Y axis of the controller.
     */
    public static double sb_getControllerRightY(){
        return Robot.controller.getRightY();
    }
    /**
     * Sets the robot's movement using joystick control (or any other suppliers). 
     * @param xVelocity Supplier for the X velocity of the robot. [-1,1]
     * @param yVelocity Supplier for the Y velocity of the robot. [-1,1]
     */
    public static void sb_setRobotMovement(Supplier<Double> xVelocity, Supplier<Double> yVelocity){
        toRun.addCommands(new JoystickControl(xVelocity,yVelocity, () -> {return 0.0;}));
    }

    public static SequentialCommandGroup getToRun(){
        return toRun;
    }
    public static void clearCommands(){
        toRun = new SequentialCommandGroup();
    }

}
