package frc.robot.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class MockMoveRobot extends Command {
    Pose2d goalPose;
    Supplier<Boolean> isPressedA;

    public MockMoveRobot(Pose2d goalPose, Supplier<Boolean> isPressedA) {
        // add Subsystem Requirements
    }

    @Override
    public void initialize() {}
    @Override
    public void execute() {}
    @Override
    public void end(boolean interrupted) {}
    @Override
    public boolean isFinished() {
        return false;
    }
}

