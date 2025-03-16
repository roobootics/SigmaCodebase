package org.firstinspires.ftc.teamcode.base;

import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.Procedure;
import org.firstinspires.ftc.teamcode.base.LambdaInterfaces.ReturningFunc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

public abstract class NonLinearActions {
    public abstract static class NonLinearAction{
        public boolean isBusy = false;
        public boolean isStart = true;
        public void reset() {
            isStart=true;
        }
        final public boolean run(){
            isBusy = runProcedure();
            isStart=false;
            if (!isBusy){
                reset();
            }
            return isBusy;
        }
        abstract boolean runProcedure();
        public void stopProcedure() {}
        final public void stop(){
            if (isBusy) {
                stopProcedure();
                isBusy = false;
                reset();
            }
        }
    }
    public abstract static class PersistentNonLinearAction extends NonLinearAction{
        @Override
        public void reset(){
            if (!isBusy){
                isStart=true;
            }
        }
    }
    public static class InstantAction extends NonLinearAction{
        public Procedure procedure;
        public InstantAction(Procedure procedure){
            this.procedure=procedure;
        }
        @Override
        public boolean runProcedure() {
            procedure.call();
            return false;
        }
    }
    public static class ContinuousAction extends NonLinearAction{
        public Procedure procedure;
        public ContinuousAction(Procedure procedure){
            this.procedure=procedure;
        }
        @Override
        public boolean runProcedure() {
            procedure.call();
            return true;
        }
    }
    public static class LambdaAction extends NonLinearAction{
        public ReturningFunc<Boolean> action;
        public LambdaAction(ReturningFunc<Boolean> action){
            this.action=action;
        }
        @Override
        public boolean runProcedure() {
            return action.call();
        }
    }
    public abstract static class CompoundAction extends NonLinearAction{
        public NonLinearSequentialAction sequence;
        @Override
        boolean runProcedure() {
            if (isStart){
                sequence.reset();
            }
            return sequence.run();
        }
        @Override public void stopProcedure(){
            sequence.stop();
        }
    }



    public static class SleepUntilTrue extends NonLinearAction{
        public ReturningFunc<Boolean> condition;
        public double timeout;
        public double startTime;
        public SleepUntilTrue(ReturningFunc<Boolean> condition, double timeout){
            this.condition = condition;
            this.timeout = timeout;
        }
        public SleepUntilTrue(ReturningFunc<Boolean> condition){
            this.condition = condition;
            this.timeout = Double.POSITIVE_INFINITY;
        }
        @Override
        boolean runProcedure() {
            if (isStart && timeout!=Double.POSITIVE_INFINITY){
                startTime=Components.timer.time();
            }
            return !condition.call() && (Components.timer.time()-startTime)<timeout;
        }
    }
    public static class NonLinearSequentialAction extends NonLinearAction{
        public List<NonLinearAction> remainingActions;
        public List<NonLinearAction> actions;
        public boolean[] isStarts;
        public NonLinearSequentialAction(NonLinearAction...actions){
            this.actions = Arrays.asList(actions);
            this.remainingActions=new ArrayList<>(this.actions);
            isStarts = new boolean[actions.length];
            for (int i=0;i<actions.length;i++){
                isStarts[i]=true;
            }
        }
        @Override
        public boolean runProcedure() {
            if (isStart){
                remainingActions=actions;
                for (int i=0;i<actions.size();i++){
                    isStarts[i]=true;
                }
            }
            if (isStarts[actions.size()-remainingActions.size()]){
                remainingActions.get(0).reset();
                isStarts[actions.size()-remainingActions.size()]=false;
            }
            if (!remainingActions.get(0).run()){
                remainingActions.remove(0);
            }
            return !remainingActions.isEmpty();
        }
        @Override
        public void stopProcedure(){
            remainingActions.get(0).stop();
        }
    }
    public static class NonLinearParallelAction extends NonLinearAction{
        public List<NonLinearAction> remainingActions;
        public List<NonLinearAction> actions;
        public NonLinearParallelAction(NonLinearAction...actions){
            this.actions = Arrays.asList(actions);
            this.remainingActions=new ArrayList<>(this.actions);
        }
        @Override
        public boolean runProcedure() {
            if (isStart){
                remainingActions=actions;
                for (NonLinearAction action : remainingActions){
                    action.reset();
                }
            }
            remainingActions = remainingActions.stream().filter(NonLinearAction::run).collect(Collectors.toList());
            return !remainingActions.isEmpty();
        }
        @Override
        public void stopProcedure(){
            for (NonLinearAction action : remainingActions){
                action.stop();
            }
        }
    }
    public static class ConditionalPair{
        public ReturningFunc<Boolean> condition;
        public NonLinearAction action;
        public ConditionalPair(ReturningFunc<Boolean> condition,NonLinearAction action){
            this.condition=condition;
            this.action=action;
        }
    }
    public static class ConditionalAction extends NonLinearAction{
        public LinkedHashMap<ReturningFunc<Boolean>,NonLinearAction> actions = new LinkedHashMap<>();
        public NonLinearAction currentAction = null;
        public ConditionalAction(ConditionalPair...conditionalPairs){
            for (ConditionalPair conditionalPair : conditionalPairs){
                actions.put(conditionalPair.condition,conditionalPair.action);
            }
        }
        @Override
        boolean runProcedure() {
            if (isStart){
                for (ReturningFunc<Boolean> condition : actions.keySet()){
                    if (condition.call()){
                        if (actions.get(condition)!=currentAction) {
                            currentAction.stop();
                            currentAction = actions.get(condition);
                        }
                        break;
                    }
                }
                if (Objects.nonNull(currentAction)) {
                    currentAction.reset();
                }
            }
            else{
                for (ReturningFunc<Boolean> condition : actions.keySet()){
                    if (condition.call()){
                        if (actions.get(condition)!=currentAction) {
                            currentAction.stop();
                            currentAction = actions.get(condition);
                            assert currentAction != null;
                            currentAction.reset();
                        }
                        break;
                    }
                }
            }
            if (Objects.nonNull(currentAction)){
                if (!currentAction.run()){
                    currentAction=null;
                    return false;
                }
                else{
                    return true;
                }
            }
            else return false;
        }
    }
    public static class PressTrigger extends ConditionalAction{
        public boolean[] isPressed;
        public PressTrigger(ConditionalPair...conditionalPairs){
            super(conditionalPairs);
            actions.clear();
            isPressed=new boolean[conditionalPairs.length];
            for (int i=0;i< conditionalPairs.length;i++){
                int finalI = i;
                actions.put(
                        ()-> {
                            if (conditionalPairs[finalI].condition.call()){
                                if (!isPressed[finalI]){
                                    isPressed[finalI]=true;
                                    return true;
                                }
                                else return false;
                            }
                            else{
                                isPressed[finalI]=false;
                                return false;
                            }
                        },
                        conditionalPairs[i].action
                );
            }
        }
    }
}
