package org.firstinspires.ftc.teamcode.base.custom;

import static org.firstinspires.ftc.teamcode.base.Components.timer;

import org.firstinspires.ftc.teamcode.base.Components.ControlFunction;
import org.firstinspires.ftc.teamcode.base.Components.CRActuator;

public abstract class PresetControl {
    public static class PID<E extends CRActuator<?>> extends ControlFunction<E>{
        public double kP;
        public double kI;
        public double kD;
        public double integralSum;
        public double previousError;
        public double prevLoopTime;
        public PID(double kP, double kI, double kD){
            this.kP=kP;
            this.kI=kI;
            this.kD=kD;
        }
        @Override
        protected void runProcedure() {
            double currentPosition = parentActuator.getCurrentPosition();
            integralSum += parentActuator.getTarget()-currentPosition;
            parentActuator.setPower(
                    kP * parentActuator.instantTarget-currentPosition +
                    kI * integralSum * timer.time()-prevLoopTime +
                    kD * ((parentActuator.instantTarget-currentPosition)-previousError)/(timer.time()-prevLoopTime)
            );
            previousError=parentActuator.instantTarget-currentPosition;
            prevLoopTime=timer.time();
        }
    }
}
