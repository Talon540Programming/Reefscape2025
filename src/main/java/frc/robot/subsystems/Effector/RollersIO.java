package.frc.robot.subsystems.Effector;

import org.littletonrobotics.junction.AutoLog;


public interface RollersIO {

    @AutoLog
    class RollersIOInputs {
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {};
    }

    Public default void updateInputs(RollersIOInputs inputs) {}

    public default void setVoltage(double voltage) {}

}