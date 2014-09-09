package de.uni_luebeck.iti.smachapp.model;

import de.uni_luebeck.iti.smachGenerator.ISmachableTransition;

/**
 * Created by Morten Mey on 21.04.2014.
 */
public class Transition implements ISmachableTransition {


    private State previous;
    private State next;
    private String name;
    private Guard guard;
    private BezierPath path;

    private Guard disabledGuard = new Guard();

    public Transition(State from, State to, String name, BezierPath path) {
        this(from, to, name, path, new Guard());
    }

    public Transition(State from, State to, String name, BezierPath path, Guard guard) {
        previous = from;
        next = to;
        this.name = name;
        this.path = path;
        this.guard = guard;
    }

    public State getPreviousState() {
        return previous;
    }

    @Override
    public State getFollowerState() {
        return next;
    }

    @Override
    public Guard getSmachableGuard() {
        return guard;
    }

    @Override
    public String getLabel() {
        return name;
    }

    public void setLabel(String name) {
        this.name = name;
    }

    public BezierPath getPath() {
        return path;
    }

    public void setPreviousState(State s) {
        previous = s;
    }

    public void setFollowerState(State s) {
        next = s;
    }

    public Guard getDisabledGuard(){
        return disabledGuard;
    }
}
