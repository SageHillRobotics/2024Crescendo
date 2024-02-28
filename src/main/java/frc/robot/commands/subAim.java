package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Wrist;

public class subAim extends Command{
    private Wrist wrist;
    public subAim(Wrist wrist){
        this.wrist = wrist;
    }
    @Override
    public void initialize(){
        wrist.wristPID(0.1);
        
    }
}
