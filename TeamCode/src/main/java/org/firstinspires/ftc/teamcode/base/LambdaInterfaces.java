package org.firstinspires.ftc.teamcode.base;

public abstract class LambdaInterfaces {
    public interface Procedure{
        void call();
    }
    public interface ReturningFunc<E>{
        E call();
    }
    public interface Condition extends ReturningFunc<Boolean>{}
    public interface AcceptingFunc<E>{
        void call(E input);
    }
}
