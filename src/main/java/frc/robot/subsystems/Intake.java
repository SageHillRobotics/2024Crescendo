package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase{
    private final CANSparkMax intakeCollectionMotor;
    private final CANSparkMax intakeRetractionMotor;
    private final SparkPIDController retractionPID;
    public double kP, kI, kD;

    public Intake(){
        
        intakeCollectionMotor = new CANSparkMax(50, MotorType.kBrushless);
        intakeRetractionMotor = new CANSparkMax(0, MotorType.kBrushless);
        retractionPID = intakeRetractionMotor.getPIDController();
        retractionPID.setOutputRange(-0.5,0.5);

        //add motor feedback sensor
       


    }
    public void spinIntakeMotor(){
        intakeCollectionMotor.set(-.4);
    }
    public void stopIntakeMotor(){
        intakeCollectionMotor.stopMotor();
    }

}
