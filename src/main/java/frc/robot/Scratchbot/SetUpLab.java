package frc.robot.Scratchbot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Lab.Lab;

public class SetUpLab {
    public static void initialize() {
        Lab.runLab();
        runScratchbotCommands();
    }
    public static void runScratchbotCommands(){
        CommandScheduler.getInstance().cancelAll();
        SequentialCommandGroup sequence = Scratchbot.getToRun();
        CommandScheduler.getInstance().schedule(sequence);
        Scratchbot.clearCommands();
    }
}