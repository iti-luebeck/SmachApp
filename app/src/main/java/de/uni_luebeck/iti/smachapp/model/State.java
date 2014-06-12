package de.uni_luebeck.iti.smachapp.model;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import de.uni_luebeck.iti.smachGenerator.ISmachableState;

/**
 * Created by Morten Mey on 21.04.2014.
 */
public class State implements ISmachableState, Iterable<Transition> {

    private String name;
    private boolean initialState;
    private LinkedList<Transition> transitions = new LinkedList<Transition>();
    private LinkedList<Action> actions = new LinkedList<Action>();

    private float x, y;

    public State(String name, float x, float y) {
        this(name, x, y, false);
    }

    public State(String name, float x, float y, boolean initialState) {
        this.name = name;
        this.initialState = initialState;
        this.x = x;
        this.y = y;
    }

    @Override
    public List<Action> getActions() {
        return actions;
    }

    @Override
    public List<Transition> getTransitions() {
        return transitions;
    }

    @Override
    public boolean isInitialState() {
        return initialState;
    }

    @Override
    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public void addTransition(Transition t) {
        if (t.getPreviousState() != this) {
            throw new IllegalArgumentException("This is not the previous state.");
        }
        transitions.add(t);
    }

    public void removeTransition(Transition t) {
        transitions.remove(t);
    }

    public Iterator<Transition> transitionIterator() {
        return transitions.iterator();
    }

    public void addAction(Action a) {
        actions.add(a);
    }

    public void removeAction(Action a) {
        actions.remove(a);
    }

    public Iterator<Action> actionIterator() {
        return actions.iterator();
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public void setX(float x) {
        this.x = x;
    }

    public void setY(float y) {
        this.y = y;
    }

    public void setCenter(float x, float y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public Iterator<Transition> iterator() {
        return transitions.iterator();
    }

    public void setInitialState(boolean init) {
        initialState = init;
    }
}
