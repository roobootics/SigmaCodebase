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
import java.util.HashMap;
import java.util.Objects;
import java.util.function.Function;

import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.ReturningFunc;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.Condition;
import org.firstinspires.ftc.teamcode.base.custom.TimeBasedLocalizers;

public abstract class Components {
    public static HardwareMap hardwareMap;
    public static Telemetry telemetry;
    public static ElapsedTime timer = new ElapsedTime();
    static{
        timer.reset();
    }
    public static HashMap<String,Actuator<? extends HardwareDevice>> actuators = new HashMap<>();
    public static class RunLoopRoutine<E extends RunConfiguration> extends NonLinearActions.ContinuousAction{
        public RunLoopRoutine() {
            super(()->{
                for (Actuator<?> actuator : actuators.values()){
                    actuator.setTarget(actuator.target);
                    actuator.runControl();
                    actuator.newTarget=false;
                }
                E.singleton.updateTelemetry();
            });
        }
    }
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
    public abstract static class ControlFunction<E extends Actuator<? extends HardwareDevice>>{
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
        public E part;
        double target;
        public double instantTarget;
        public boolean newTarget=false;
        public ReturningFunc<Double> maxTargetFunc;
        public ReturningFunc<Double> minTargetFunc;
        public String currControlFuncKey = "controlOff";
        public HashMap<String,ArrayList<ControlFunction<Actuator<E>>>> controlFuncsMap = new HashMap<>();
        public ReturningFunc<Double> getCurrentPosition;
        public double errorTol;
        double offset;
        public double defaultTimeout;
        public HashMap<String,Double> keyPositions = new HashMap<>();
        public boolean notCommanded = true;
        public String defaultControlKey;
        boolean isPowered = true;
        public String name;
        Class<E> type;
        public Function<Double,Double> positionConversion = (Double pos)->(pos);
        public Function<Double,Double> positionConversionInverse = (Double pos)->(pos);
        @SafeVarargs
        public Actuator(String name, Function<Actuator<E>,Double> getCurrentPosition,
                        ReturningFunc<Double> maxTargetFunc,ReturningFunc<Double> minTargetFunc,
                        double errorTol, double defaultTimeout,
                        String[] keyPositionKeys,
                        double[] keyPositionValues,
                        String[] controlFuncKeys,
                        ArrayList<ControlFunction<Actuator<E>>>... controlFuncs){
            this.name=name;
            this.getCurrentPosition = ()->(positionConversion.apply(getCurrentPosition.apply(this)));
            this.maxTargetFunc = ()->(maxTargetFunc.call()+offset);
            this.minTargetFunc = ()->(minTargetFunc.call()+offset);
            this.defaultTimeout = defaultTimeout;
            for (int i=0;i<controlFuncKeys.length;i++) {
                controlFuncsMap.put(controlFuncKeys[i],controlFuncs[i]);
                for (ControlFunction<Actuator<E>> func : controlFuncs[i]) {
                    func.registerToParent(this);
                }
            }
            controlFuncsMap.put("controlOff",new ArrayList<>());
            this.errorTol=errorTol;
            for (int i=0; i<keyPositionKeys.length; i++){
                keyPositions.put(keyPositionKeys[i],keyPositionValues[i]);
            }
            this.defaultControlKey=controlFuncKeys[0];
            actuators.put(name,this);
        }
        public void constructPart(){
            this.part=Components.hardwareMap.get(this.type,this.name);
        }
        public void setTarget(double target){
            if (notCommanded){
                currControlFuncKey=defaultControlKey;
                notCommanded=false;
            }
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
        public double getCurrentPosition(){
            return getCurrentPosition.call();
        }
        public void setOffset(double offset){
            this.offset=offset;
            setTarget(target);
        }
        public void runControl(){
            for (ControlFunction<Actuator<E>> func : Objects.requireNonNull(controlFuncsMap.get(currControlFuncKey))) {
                func.run();
            }
        }
        public double getPos(String key){
            return Objects.requireNonNull(keyPositions.get(key));
        }
        public void switchControl(String key){
            for (ControlFunction<?> func: Objects.requireNonNull(controlFuncsMap.get(currControlFuncKey))){
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
        @SafeVarargs
        public CRActuator(String name, Function<Actuator<E>, Double> getCurrentPosition, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues,
                          DcMotorSimple.Direction direction,
                          String[] controlFuncKeys, ArrayList<ControlFunction<Actuator<E>>>... controlFuncs) {
            super(name, getCurrentPosition, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout, keyPositionKeys, keyPositionValues, controlFuncKeys, controlFuncs);
            part.setDirection(direction);
            this.target=0;
        }
        public void setPower(double power){
            if (isPowered) {
                if (Math.abs(power-this.power)>0.05) {
                    this.power = power;
                    notCommanded = false;
                    part.setPower(power);
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
            return new ConditionalAction(new ConditionalPair(upCondition, setPowerAction(()->(part.getPower()+change))),new ConditionalPair(downCondition, setPowerAction(()->(part.getPower()-change))));
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
        public BotMotor(String name, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues, DcMotorSimple.Direction direction, String[] controlFuncKeys, ArrayList<ControlFunction<Actuator<DcMotorEx>>>... controlFuncs) {
            super(name, (Actuator<DcMotorEx> motor)->((double) motor.part.getCurrentPosition()), maxTargetFunc, minTargetFunc, errorTol, defaultTimeout, keyPositionKeys, keyPositionValues, direction, controlFuncKeys, controlFuncs);
            this.type=DcMotorEx.class;
            part.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            part.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.constructPart();
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
                if (part.getCurrent(CurrentUnit.AMPS)>1.5){
                    setOffset(getCurrentPosition()-resetPosition);
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
        public BotServo(String name, Function<Actuator<Servo>, Double> getCurrentPosition, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues, Servo.Direction direction, double range, String[] controlFuncKeys, ArrayList<ControlFunction<Actuator<Servo>>>... controlFuncs) {
            super(name, getCurrentPosition, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout, keyPositionKeys, keyPositionValues, controlFuncKeys, controlFuncs);
            this.positionConversion=(Double pos)->(pos*range);
            this.positionConversionInverse=(Double pos)->(pos*range);
            this.type=Servo.class;
            part.setDirection(direction);
            this.constructPart();
        }
        @SafeVarargs
        public BotServo(String name, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double servoSpeed, String[] keyPositionKeys, double[] keyPositionValues, Servo.Direction direction, double range, String[] controlFuncKeys, ArrayList<ControlFunction<Actuator<Servo>>>... controlFuncs) {
            super(name, new TimeBasedLocalizers.ServoTimeBasedLocalizer(servoSpeed)::getCurrentPosition, maxTargetFunc, minTargetFunc, 0, Double.POSITIVE_INFINITY, keyPositionKeys, keyPositionValues, controlFuncKeys, controlFuncs);
            this.positionConversion=(Double pos)->(pos*range);
            this.positionConversionInverse=(Double pos)->(pos*range);
            part.setDirection(direction);
            this.type=Servo.class;
            this.constructPart();
        }
        public void setPosition(double position){
            if (isPowered && position!=currCommandedPos){
                currCommandedPos=position;
                part.setPosition(positionConversionInverse.apply(position));
            }
        }
        public double getPosition() {
            return part.getPosition();
        }
    }
    public static class CRBotServo extends CRActuator<CRServo>{
        @SafeVarargs
        public CRBotServo(String name, Function<Actuator<CRServo>, Double> getCurrentPosition, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double errorTol, double defaultTimeout, String[] keyPositionKeys, double[] keyPositionValues, DcMotorSimple.Direction direction, String[] controlFuncKeys, ArrayList<ControlFunction<Actuator<CRServo>>>... controlFuncs) {
            super(name, getCurrentPosition, maxTargetFunc, minTargetFunc, errorTol, defaultTimeout, keyPositionKeys, keyPositionValues, direction, controlFuncKeys, controlFuncs);
            this.type=CRServo.class;
            this.constructPart();
        }
        @SafeVarargs
        public CRBotServo(String name, ReturningFunc<Double> maxTargetFunc, ReturningFunc<Double> minTargetFunc, double servoSpeed, String[] keyPositionKeys, double[] keyPositionValues, DcMotorSimple.Direction direction, String[] controlFuncKeys, ArrayList<ControlFunction<Actuator<CRServo>>>... controlFuncs) {
            super(name, new TimeBasedLocalizers.CRTimeBasedLocalizer(servoSpeed)::getCurrentPosition, maxTargetFunc, minTargetFunc, 0, Double.POSITIVE_INFINITY, keyPositionKeys, keyPositionValues, direction, controlFuncKeys, controlFuncs);
            this.type=CRServo.class;
            this.constructPart();
        }
    }
}
