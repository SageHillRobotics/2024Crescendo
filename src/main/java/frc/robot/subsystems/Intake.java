package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase{
    private final static double EXTENDED_ENCODER_VALUE = -47.5;
    private final CANSparkMax intakeCollectionMotor;
    private final CANSparkMax intakeRetractionMotor;
    private final SparkPIDController retractionPID;
    public double kP, kI, kD;

    public Intake(){
        intakeCollectionMotor = new CANSparkMax(49, MotorType.kBrushless);
        intakeRetractionMotor = new CANSparkMax(50, MotorType.kBrushless);
        retractionPID = intakeRetractionMotor.getPIDController();
        retractionPID.setP(.08);
        retractionPID.setI(0);
        retractionPID.setD(0);
        retractionPID.setOutputRange(-1,1);
    }
    public void retract(){
        retractionPID.setReference(0, ControlType.kPosition);
    }
    public void extend(){
        retractionPID.setReference(EXTENDED_ENCODER_VALUE, ControlType.kPosition);
    }
    public void spinIntakeMotor(){
        intakeCollectionMotor.set(-.55);
    }
    public void stopIntakeMotor(){
        intakeCollectionMotor.stopMotor();
    }
    public boolean atExtendedPosition(){
        return Math.abs(intakeRetractionMotor.getEncoder().getPosition() - EXTENDED_ENCODER_VALUE) < 5;
    }

    public boolean atRetractedPosition(){
        return Math.abs(intakeRetractionMotor.getEncoder().getPosition()) < 5;
    }
    public void eject(){
        intakeCollectionMotor.set(.4);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("intake encoder value", intakeRetractionMotor.getEncoder().getPosition()); 
    }
}
