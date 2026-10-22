package frc.robot.Lab;

import frc.robot.Scratchbot.Scratchbot;

public class Lab {
   public static void movement()
   {
        Scratchbot.sb_moveForward(1);
        System.out.println("movement finnished");
   }

   public static void turning()
   {
    Scratchbot.sb_turnClockwise(0.5);
    System.out.println("Turning complete!");
   }
   
    public static void runLab()
    {
        movement();
        turning();
        movement();
        turning();
        System.out.println("Methods complete!");
    }
}
