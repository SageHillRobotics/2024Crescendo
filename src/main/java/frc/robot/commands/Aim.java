package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SwerveSubsystem;

public class Aim extends Command{
    SwerveSubsystem swerve;
    Vision vision;
    PIDController pidController;
    public Aim(SwerveSubsystem swerve, Vision vision){
        this.swerve = swerve;
        this.vision = vision;
        pidController = new PIDController(0.1, 0, 0);
        pidController.setSetpoint(0);
        pidController.setTolerance(0.4);
        addRequirements(swerve);
    }
    @Override
    public void execute(){
        int index = 0;
        var targets = vision.getTargets();
        boolean hasSpeakerTarget;
        if (targets != null){
            hasSpeakerTarget = false;
            for (int i = 0; i < targets.size(); i++){
                if(targets.get(i).getFiducialId() == 4 || targets.get(i).getFiducialId() == 8){
                    index = i;
                    hasSpeakerTarget = true;
                    break;
                }
            }
            if (hasSpeakerTarget){
                double steeringAdj = pidController.calculate(targets.get(index).getYaw());
                swerve.drive(new Translation2d(), steeringAdj, true);
            }
        }

        
    }
    @Override
    public boolean isFinished(){
        var targets = vision.getTargets();
        boolean hasSpeakerTarget = false;
        if (targets != null){
            for (int i = 0; i < targets.size(); i++){
                    if(targets.get(i).getFiducialId() == 4 || targets.get(i).getFiducialId() == 8){
                        hasSpeakerTarget = true;
                        break;
                    }
                }
        }
        return (pidController.atSetpoint() && hasSpeakerTarget) || !(hasSpeakerTarget);
    }
}
