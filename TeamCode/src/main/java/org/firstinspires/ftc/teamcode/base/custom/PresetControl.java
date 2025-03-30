package org.firstinspires.ftc.teamcode.base.custom;

import static org.firstinspires.ftc.teamcode.base.Components.timer;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.base.Components.ControlFunction;
import org.firstinspires.ftc.teamcode.base.Components.CRActuator;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.ReturningFunc;

public abstract class PresetControl {
    public static class PIDF<E extends CRActuator<?>> extends ControlFunction<E>{
        public double kP;
        public double kI;
        public double kD;
        public double kF;
        public double integralSum;
        public double previousError;
        public double prevLoopTime;
        public ReturningFunc<Double> feedForwardFunc;
        public PIDF(double kP, double kI, double kD, double kF, ReturningFunc<Double> feedForwardFunc){
            this.kP=kP;
            this.kI=kI;
            this.kD=kD;
            this.kF=kF;
            this.feedForwardFunc = feedForwardFunc;
        }
        public PIDF(double kP, double kI, double kD){
            this.kP=kP;
            this.kI=kI;
            this.kD=kD;
            this.kF=0;
            this.feedForwardFunc = ()->(0.0);
        }
        @Override
        protected void runProcedure() {
            double currentPosition = parentActuator.getCurrentPosition();
            integralSum += parentActuator.getTarget()-currentPosition;
            parentActuator.setPower(
                    kP * parentActuator.instantTarget-currentPosition +
                    kI * integralSum * timer.time()-prevLoopTime +
                    kD * ((parentActuator.instantTarget-currentPosition)-previousError)/(timer.time()-prevLoopTime) +
                    kF * feedForwardFunc.call()
            );
            previousError=parentActuator.instantTarget-currentPosition;
            prevLoopTime=timer.time();
        }
    }
    public static class ServoControl extends ControlFunction<Components.Actuator<Servo>>{
        @Override
        protected void runProcedure() {
            parentActuator.part.setPosition(parentActuator.instantTarget);
        }
    }
}
