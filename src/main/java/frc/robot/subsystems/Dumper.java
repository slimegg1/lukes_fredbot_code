package frc.robot.subsystems;

/* Luke Removed
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Encoder;
*/
import frc.robot.Constants.DumperConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dumper extends SubsystemBase {

    public static WPI_TalonSRX dumperMotor = new WPI_TalonSRX(DumperConstants.DUMPER_MOTOR_PORT);

    private PIDController rotPID = new PIDController(0.000005, 0, 0.000001);

    public static double currentPosition = 0.0;
    private double wantedPosition = 0.0;

    public Dumper() {
        dumperMotor.setSelectedSensorPosition(0.0);
        currentPosition = dumperMotor.getSelectedSensorPosition();
        wantedPosition = 0;
    }

    public double clamp(double min, double max, double value) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public void periodic() {
        currentPosition = dumperMotor.getSelectedSensorPosition();
        SmartDashboard.putNumber("Dumper Position", currentPosition);
        SmartDashboard.putNumber("Wanted DUmper", wantedPosition);
        dumperMotor.set(clamp(-0.15, 0.15, rotPID.calculate(currentPosition, wantedPosition)));
    }

    public void rotate() {
        // Rotate 60 degrees (Just value from testing)
        wantedPosition = wantedPosition + (409600d / 6.0d);
    }

}
