package frc.robot.subsystems.Effector;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpilib.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.Constants;


public class RollersIOSim implements RollersIO{
    private final FlywheelSim m_sim;
    private double appliedVoltage = 0.0;
    private final DCMotor gearbox = DCMotor. getNeo550(numMotors:2); 
    public final double reduction = 3.0 / 1.0; //ToDO
    public final double moi = 0.14159265; //ToDO

    public RollerIOSim() {
        gearbox = motorModel;
        m_sim = 
            new FlywheelSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        m_sim.update(Constants.kLoopPeriodSecs); //TODo

        inputs.velocityRadPerSec = m_sim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVoltage;
        inputs.currentAmps = new double[] {m_sim.getCurrentDrawAmps()};

    }

    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
        m_sim.setInputVoltage(voltage);
    }
}