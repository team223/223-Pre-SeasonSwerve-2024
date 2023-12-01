package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CurrentDraw extends SubsystemBase {
    private PowerDistribution PDH;

    public CurrentDraw() {
        PDH = new PowerDistribution(1, ModuleType.kRev);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current Draw", PDH.getTotalCurrent());
        SmartDashboard.putNumber("Voltage", PDH.getVoltage());
    }
}
