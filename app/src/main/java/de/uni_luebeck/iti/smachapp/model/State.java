package de.uni_luebeck.iti.smachapp.model;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import de.uni_luebeck.iti.smachGenerator.ISmachableState;

/**
 * Created by Morten Mey on 21.04.2014.
 */
public class State implements ISmachableState {

    private String name;
    private boolean initialState;
    private LinkedList<Transition> transitions=new LinkedList<Transition>();
    private LinkedList<Action> actions=new LinkedList<Action>();

    public State(String name){
        this(name,false);
    }

    public State(String name, boolean initialState){
        this.name=name;
        this.initialState=initialState;
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

    public void setName(String name){
        this.name=name;
    }

    public void addTransition(Transition t) {
        if(t.getPreviousState()!=this){
            throw new IllegalArgumentException("This is not the previous state.");
        }
        transitions.add(t);
    }

    public void removeTransition(Transition t){
        transitions.remove(t);
    }

    public Iterator<Transition> transitionIterator(){
        return transitions.iterator();
    }

    public void addAction(Action a){
        actions.add(a);
    }

    public void removeAction(Action a){
        actions.remove(a);
    }

    public Iterator<Action> actionIterator(){
        return actions.iterator();
    }
}
