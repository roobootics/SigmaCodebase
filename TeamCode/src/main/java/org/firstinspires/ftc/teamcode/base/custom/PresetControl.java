package org.firstinspires.ftc.teamcode.base.custom;

import static org.firstinspires.ftc.teamcode.base.Components.timer;

import org.firstinspires.ftc.teamcode.base.Components.BotServo;
import org.firstinspires.ftc.teamcode.base.Components.ControlFunction;
import org.firstinspires.ftc.teamcode.base.Components.CRActuator;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.ReturningFunc;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class PresetControl {
    public static class PIDF<E extends CRActuator<?>> extends ControlFunction<E>{
        public static class PIDFConstants{
            public double kP;
            public double kI;
            public double kD;
            public double kF;
            public ReturningFunc<Double> feedForwardFunc;
            public PIDFConstants(double kP, double kI, double kD, double kF, ReturningFunc<Double> feedForwardFunc){
                this.kP=kP;
                this.kI=kI;
                this.kD=kD;
                this.kF=kF;
                this.feedForwardFunc = feedForwardFunc;
            }
            public PIDFConstants(double kP, double kI, double kD){
                this.kP=kP;
                this.kI=kI;
                this.kD=kD;
                this.kF=0;
                this.feedForwardFunc = ()->(0.0);
            }
        }
        public double[] integralSums;
        public double[] previousErrors;
        public double prevLoopTime;
        public ArrayList<PIDFConstants> constants;
        public PIDF(PIDFConstants...constants){
            this.constants=new ArrayList<>(Arrays.asList(constants));
        }
        @Override
        public void registerToParent(E actuator){
            super.registerToParent(actuator);
            if (actuator.parts.length>this.constants.size()){
                for (int i=0;i<actuator.parts.length-this.constants.size();i++){
                    constants.add(constants.get(constants.size()-1));
                }
            }
        }
        @Override
        protected void runProcedure() {
            for (int i=0;i<parentActuator.parts.length;i++){
                double currentPosition = parentActuator.getCurrentPosition(i);
                integralSums[i] += parentActuator.getTarget()-currentPosition;
                parentActuator.setPower(
                        constants.get(i).kP * parentActuator.instantTarget-currentPosition +
                                constants.get(i).kI * integralSums[i] * timer.time()-prevLoopTime +
                                constants.get(i).kD * ((parentActuator.instantTarget-currentPosition)-previousErrors[i])/(timer.time()-prevLoopTime) +
                                constants.get(i).kF * constants.get(i).feedForwardFunc.call(),
                        parentActuator.parts[i]
                );
                previousErrors[i]=parentActuator.instantTarget-currentPosition;
            }
            prevLoopTime=timer.time();
        }
    }
    public static class ServoControl extends ControlFunction<BotServo>{
        @Override
        protected void runProcedure() {
            parentActuator.setPosition(parentActuator.instantTarget);
        }
    }
}
