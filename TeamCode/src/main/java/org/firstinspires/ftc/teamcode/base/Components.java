package org.firstinspires.ftc.teamcode.base;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.NonLinearSequentialAction;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.SleepUntilTrue;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.InstantAction;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.CompoundAction;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.NonLinearAction;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.ConditionalAction;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.PressTrigger;
import org.firstinspires.ftc.teamcode.base.NonLinearActions.ConditionalPair;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Objects;
import java.util.function.Function;

import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.ReturningFunc;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.Condition;

import org.firstinspires.ftc.teamcode.base.custom.PresetControl.ServoControl;
import org.firstinspires.ftc.teamcode.base.custom.TimeBasedLocalizers;

public abstract class Components {
    public static HardwareMap hardwareMap;
    public static Telemetry telemetry;
    public static ElapsedTime timer = new ElapsedTime();
    static{
        timer.reset();
    }
    public static HashMap<String,Actuator<?>> actuators = new HashMap<>();
    public abstract static class RunConfiguration{
        public static RunConfiguration singleton;
        public static void initialize(HardwareMap hardwareMap, Telemetry telemetry){
            Components.hardwareMap=hardwareMap;
            Components.telemetry=telemetry;
            singleton.initParts();
        }
        abstract void initParts();
        abstract void updateTelemetry();
    }
    public abstract static class ControlFunction<E extends Actuator<?>>{
        public E parentActuator;
        public boolean isStart;
        public void registerToParent(E parentActuator){
            this.parentActuator=parentActuator;
        }
        public void run(){
            runProcedure();
            isStart=false;
        }
        protected abstract void runProcedure();
        public void stopAndReset(){stopProcedure(); isStart=true;}
        public void stopProcedure(){}
    }
    public abstract static class Actuator<E extends HardwareDevice>{
        public class FuncRegister<T extends Actuator<E>>{
            public HashMap<String,ArrayList<ControlFunction<T>>> controlFuncsMap = new HashMap<>();
            @SafeVarargs
            public FuncRegister(T instance, Function<E, Double> getCurrentPosition,String[] controlFuncKeys, ArrayList<ControlFunction<T>>... controlFuncs){
                for (int i=0;i<Actuator.this.parts.length;i++){
                    int finalI = i;
                    Actuator.this.getCurrentPositions[i] = ()->(positionConversion.apply(getCurrentPosition.apply(parts[finalI])));
                }

                for (int i=0;i<controlFuncKeys.length;i++) {
                    for (ControlFunction<T> func : controlFuncs[i]) {
                        func.registerToParent(instance);
                    }
                    controlFuncsMap.put(controlFuncKeys[i],controlFuncs[i]);
                }
                controlFuncsMap.put("controlOff",new ArrayList<>());
                currControlFuncKey="controlOff";
                defaultControlKey=controlFuncKeys[0];
            }
        }
        public FuncRegister<?> funcRegister;
        public E[] parts;
        double target;
        public double instantTarget;
        public boolean newTarget=false;
        public ReturningFunc<Double> maxTargetFunc;
        public ReturningFunc<Double> minTargetFunc;
        public String currControlFuncKey;
        public ReturningFunc<Double>[] getCurrentPositions;
        public double errorTol;
        double offset;
        public double defaultTimeout;
        public HashMap<String,Double> keyPositions = new HashMap<>();
        public String defaultControlKey;
        boolean isPowered = true;
        public String name;
        public Function<Double,Double> positionConversion = (Double pos)->(pos);
        public Function<Double,Double> positionConversionInverse = (Double pos)->(pos);
        public Actuator(String name, Class<E> type,
                        String[] names,
                        ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc,
                        double errorTol, double defaultTimeout,
                        String[] keyPositionKeys,
                        double[] keyPositionValues){
            this.name=name;
            this.maxTargetFunc = ()->(maxTargetFunc.call()+offset);
            this.minTargetFunc = ()->(minTargetFunc.call()+offset);
            this.defaultTimeout = defaultTimeout;
            this.errorTol=errorTol;
            for (int i=0; i<keyPositionKeys.length; i++){
                keyPositions.put(keyPositionKeys[i],keyPositionValues[i]);
            }
            for (int i=0;i<names.length;i++){
                this.parts[i]=Components.hardwareMap.get(type,names[i]);
            }
            actuators.put(name,this);
        }
        public void setTarget(double target){
            target=target+offset;
            target=Math.max(minTargetFunc.call(),Math.min(target, maxTargetFunc.call()));
            if (target!=this.target) {
                this.target = target;
                this.instantTarget = target;
                newTarget = true;
            }
        }
        public double getTarget(){
            return target;
        }
        public double getCurrentPosition(int partIndex){
            return getCurrentPositions[partIndex].call();
        }
        public double getCurrentPosition(){
            double avg = 0;
            for (ReturningFunc<Double> func: getCurrentPositions){
                avg+=func.call();
            }
            return avg/getCurrentPositions.length;
        }
        public void setOffset(double offset){
            this.offset=offset;
            setTarget(target);
        }
        public void runControl(){
            for (ControlFunction<? extends Actuator<E>> func : Objects.requireNonNull(this.funcRegister.controlFuncsMap.get(currControlFuncKey))) {
                func.run();
            }
        }
        public double getPos(String key){
            return Objects.requireNonNull(keyPositions.get(key));
        }
        public void switchControl(String key){
            for (ControlFunction<?> func: Objects.requireNonNull(this.funcRegister.controlFuncsMap.get(currControlFuncKey))){
                func.stopAndReset();
            }
            currControlFuncKey=key;
        }
        public class SetTargetAction extends CompoundAction {
            public SetTargetAction(ReturningFunc<Double> targetFunc, double timeout){
                sequence = new NonLinearSequentialAction(
                        new InstantAction(()-> setTarget(targetFunc.call())),
                        new SleepUntilTrue(
                                ()->(Math.abs(getCurrentPosition()-target)<errorTol),
                                timeout
                        )
                );
            }
            public SetTargetAction(double target, double timeout){
                this(()->(target), timeout);
            }
            public SetTargetAction(ReturningFunc<Double> targetFunc){
                this(targetFunc, defaultTimeout);
            }
            public SetTargetAction(double target){
                this(()->(target), defaultTimeout);
            }
            @Override
            public void stopProcedure(){
                setTarget(getCurrentPosition());
            }
        }
        public class SetOffsetAction extends CompoundAction {
            public SetOffsetAction(ReturningFunc<Double> offsetFunc, double timeout){
                sequence = new NonLinearSequentialAction(
                        new InstantAction(()-> setOffset(offsetFunc.call())),
                        new SleepUntilTrue(
                                ()->(Math.abs(getCurrentPosition()-target)<errorTol),
                                timeout
                        )
                );
            }
            public SetOffsetAction(double offset, double timeout){
                this(()->(offset), timeout);
            }
            public SetOffsetAction(ReturningFunc<Double> offsetFunc){
                this(offsetFunc, defaultTimeout);
            }
            public SetOffsetAction(double offset){
                this(()->(offset), defaultTimeout);
            }
            @Override
            public void stopProcedure(){
                setTarget(getCurrentPosition());
            }
        }
        public SetTargetAction setTargetAction(double target){
            return new SetTargetAction(target);
        }
        public SetTargetAction setTargetAction(ReturningFunc<Double> targetFunc){
            return new SetTargetAction(targetFunc);
        }
        public SetTargetAction setTargetAction(double target, double timeout){
            return new SetTargetAction(target,timeout);
        }
        public SetTargetAction setTargetAction(ReturningFunc<Double> targetFunc, double timeout){
            return new SetTargetAction(targetFunc);
        }
        public SetTargetAction toggleAction(double target1, double target2){
            return setTargetAction(()->{
                if (this.target==target1) return target2; else if (this.target==target2) return target1; else return this.target;
            });
        }
        public SetTargetAction upwardFSMAction(double...targets){
            Arrays.sort(targets);
            return setTargetAction(()->{
                for (double target: targets){
                    if (this.target<target){
                        return target;
                    }
                }
                return this.target;
            });
        }
        public SetTargetAction downwardFSMAction(double...targets){
            Arrays.sort(targets);
            return setTargetAction(()->{
                for (int i = targets.length-1; i>=0; i--){
                    if (this.target>targets[i]){
                        return targets[i];
                    }
                }
                return this.target;
            });
        }
        public SetOffsetAction setOffsetAction(double offset){
            return new SetOffsetAction(offset);
        }
        public SetOffsetAction setOffsetAction(ReturningFunc<Double> offsetFunc){
            return new SetOffsetAction(offsetFunc);
        }
        public PressTrigger triggeredSetTargetAction(Condition condition, double target){
            return new PressTrigger(new ConditionalPair(condition, new SetTargetAction(target)));
        }
        public PressTrigger triggeredSetTargetAction(Condition condition, ReturningFunc<Double> targetFunc) {
            return new PressTrigger(new ConditionalPair(condition, setTargetAction(targetFunc)));
        }
        public PressTrigger triggeredSetTargetAction(Condition condition, double target, double timeout){
                return new PressTrigger(new ConditionalPair(condition, setTargetAction(target,timeout)));
        }
        public PressTrigger triggeredSetTargetAction(Condition condition, ReturningFunc<Double> targetFunc, double timeout){
                return new PressTrigger(new ConditionalPair(condition, setTargetAction(targetFunc,timeout)));
        }
        public PressTrigger triggeredToggleAction(Condition condition, double target1, double target2){
            return new PressTrigger(new ConditionalPair(condition, toggleAction(target1,target2)));
        }
        public ConditionalAction triggeredDynamicAction(Condition upCondition, Condition downCondition, double change){
            return new ConditionalAction(new ConditionalPair(upCondition, setTargetAction(()->(target+change))),new ConditionalPair(downCondition, setTargetAction(()->(target-change))));
        }
        public PressTrigger triggeredFSMAction(Condition upCondition, Condition downCondition, double...targets){
            return new PressTrigger(new ConditionalPair(upCondition, upwardFSMAction(targets)),new ConditionalPair(downCondition, downwardFSMAction(targets)));
        }
        public PressTrigger triggeredSetOffsetAction(Condition condition, double offset){
            return new PressTrigger(new ConditionalPair(condition, new SetOffsetAction(offset)));
        }
        public PressTrigger triggeredDynamicOffsetAction(Condition upCondition, Condition downCondition, double offsetChange){
            return new PressTrigger(new ConditionalPair(upCondition, setOffsetAction(()->(offset+offsetChange))),new ConditionalPair(downCondition, setOffsetAction(()->(target-offsetChange))));
        }
    }
    public abstract static class CRActuator<E extends DcMotorSimple> extends Actuator<E>{
        double power;
        public CRActuator(String name, Class<E> type, String[] names, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues,
                          DcMotorSimple.Direction[] directions) {
            super(name, type, names, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout, keyPositionKeys, keyPositionValues);
            for (int i=0;i<directions.length;i++){
                parts[i].setDirection(directions[i]);
            }
            this.target=0;
        }
        public void setPower(double power, E part){
            if (isPowered){
                if (Math.abs(power-part.getPower())>0.05) {
                    part.setPower(power);
                }
            }
        }
        public void setPower(double power){
            if (isPowered) {
                if (Math.abs(power-this.power)>0.05) {
                    this.power = power;
                    for (E part:parts){
                        part.setPower(power);
                    }
                }
            }
        }
        public class SetPowerAction extends InstantAction{
            public SetPowerAction(ReturningFunc<Double> powerFunc) {
                super(()-> setPower(powerFunc.call()));
            }
            public SetPowerAction(double power) {
                super(()-> setPower(power));
            }
        }
        public SetPowerAction setPowerAction(ReturningFunc<Double> powerFunc){
            return new SetPowerAction(powerFunc);
        }
        public SetPowerAction setPowerAction(double power){
            return new SetPowerAction(power);
        }
        public SetPowerAction togglePowerAction(double power1, double power2){
            return new SetPowerAction(()->{
                if (power==power1) return power2; else if (power==power2) return power1; else return this.power;
            });
        }
        public SetPowerAction upwardFSMPowerAction(double...powers){
            Arrays.sort(powers);
            return setPowerAction(()->{
                for (double power: powers){
                    if (this.power<power){
                        return power;
                    }
                }
                return this.power;
            });
        }
        public SetPowerAction downwardFSMPowerAction(double...powers){
            Arrays.sort(powers);
            return setPowerAction(()->{
                for (int i = powers.length-1; i>=0; i--){
                    if (this.power>powers[i]){
                        return powers[i];
                    }
                }
                return this.power;
            });
        }
        public PressTrigger triggeredSetPowerAction(Condition condition, ReturningFunc<Double> powerFunc){
            return new PressTrigger(new ConditionalPair(condition, new SetPowerAction(powerFunc)));
        }
        public PressTrigger triggeredSetPowerAction(Condition condition, double power){
            return new PressTrigger(new ConditionalPair(condition, new SetPowerAction(power)));
        }
        public ConditionalAction triggeredDynamicPowerAction(Condition upCondition, Condition downCondition, double change){
            return new ConditionalAction(new ConditionalPair(upCondition, setPowerAction(()->(parts[0].getPower()+change))),new ConditionalPair(downCondition, setPowerAction(()->(parts[0].getPower()-change))));
        }
        public PressTrigger triggeredTogglePowerAction(Condition condition, double power1, double power2){
            return new PressTrigger(new ConditionalPair(condition, togglePowerAction(power1,power2)));
        }
        public PressTrigger triggeredFSMPowerAction(Condition upCondition, Condition downCondition, double...powers){
            return new PressTrigger(
                    new ConditionalPair(upCondition, upwardFSMPowerAction(powers)),
                    new ConditionalPair(downCondition, downwardFSMPowerAction(powers))
            );
        }
    }
    public static class BotMotor extends CRActuator<DcMotorEx>{
        public boolean isStallResetting;
        @SafeVarargs
        public BotMotor(String name, String[] names, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, ArrayList<ControlFunction<BotMotor>>... controlFuncs) {
            super(name, DcMotorEx.class, names, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout, keyPositionKeys, keyPositionValues, directions);
            for (DcMotorEx part:parts){
                part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            this.funcRegister=new FuncRegister<BotMotor>(this,(DcMotorEx motor)->((double) motor.getCurrentPosition()),controlFuncKeys, controlFuncs);
        }
        public class StallResetAction extends NonLinearAction {
            double resetPosition;
            public StallResetAction(double resetPosition) {
                this.resetPosition=resetPosition;
            }
            @Override
            boolean runProcedure() {
                if (isStart){
                    isStallResetting=true;
                    setPower(-0.2);
                }
                if (parts[0].getCurrent(CurrentUnit.AMPS)>1.5){
                    for (DcMotorEx part:parts){
                        part.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }
                    setOffset(-resetPosition);
                    setTarget(resetPosition);
                    setPower(0);
                    isStallResetting=false;
                }
                return isStallResetting;
            }
        }
        public StallResetAction stallResetAction(double resetPosition){
            return new StallResetAction(resetPosition);
        }
        public PressTrigger triggeredStallResetAction(Condition condition, double resetPosition){
            return new PressTrigger(new ConditionalPair(condition,stallResetAction(resetPosition)));
        }
    }
    public static class BotServo extends Actuator<Servo>{
        private double currCommandedPos;
        @SafeVarargs
        public BotServo(String name, String[] names, Function<Servo, Double> getCurrentPosition, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues, Servo.Direction[] directions, double range, String[] controlFuncKeys, ArrayList<ControlFunction<BotServo>>... controlFuncs) {
            super(name, Servo.class, names, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout, keyPositionKeys, keyPositionValues);
            this.positionConversion=(Double pos)->(pos*range);
            this.positionConversionInverse=(Double pos)->(pos*range);
            for (int i=0;i<directions.length;i++){
                parts[i].setDirection(directions[i]);
            }
            this.funcRegister=new FuncRegister<BotServo>(this,getCurrentPosition,controlFuncKeys, controlFuncs);
        }
        public BotServo(String name, String[] names, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double servoSpeed, String[] keyPositionKeys, double[] keyPositionValues, Servo.Direction[] directions, double range) {
            this(name,names,new TimeBasedLocalizers.ServoTimeBasedLocalizer(servoSpeed)::getCurrentPosition,maxTargetFunc,minTargetFunc,0.01,0,keyPositionKeys,keyPositionValues,directions,range,new String[]{"setPos"},new ArrayList<>(Collections.singleton(new ServoControl())));
        }
        public void setPosition(double position){
            if (isPowered && position!=currCommandedPos){
                currCommandedPos=position;
                for (Servo part:parts){part.setPosition(positionConversionInverse.apply(position));}
            }
        }
    }
    public static class CRBotServo extends CRActuator<CRServo>{
        @SafeVarargs
        public CRBotServo(String name, String[] names, Function<CRServo, Double> getCurrentPosition, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, ArrayList<ControlFunction<CRBotServo>>... controlFuncs) {
            super(name, CRServo.class, names, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout, keyPositionKeys, keyPositionValues, directions);
            this.funcRegister=new FuncRegister<CRBotServo>(this, getCurrentPosition, controlFuncKeys, controlFuncs);
        }
        @SafeVarargs
        public CRBotServo(String name, String[] names, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double servoSpeed, String[] keyPositionKeys, double[] keyPositionValues, DcMotorSimple.Direction[] directions, String[] controlFuncKeys, ArrayList<ControlFunction<CRBotServo>>... controlFuncs) {
            super(name, CRServo.class, names, maxTargetFunc, minTargetFunc, 0, Double.POSITIVE_INFINITY, keyPositionKeys, keyPositionValues, directions);
            this.funcRegister=new FuncRegister<CRBotServo>(this, new TimeBasedLocalizers.CRTimeBasedLocalizer<CRServo>(servoSpeed)::getCurrentPosition, controlFuncKeys, controlFuncs);
        }
    }
}
