package de.uni_luebeck.iti.smachapp.model;

import de.uni_luebeck.iti.smachGenerator.ISmachableTransition;

/**
 * Created by Morten Mey on 21.04.2014.
 */
public class Transition implements ISmachableTransition {


    private State previous;
    private State next;
    private String name;
    private Guard guard=new Guard();

    public Transition(State from,State to,String name){
        previous=from;
        next=to;
        this.name=name;
    }

    public State getPreviousState(){
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

    public void setName(String name){
        this.name=name;
    }

    public String getName(){
        return name;
    }


}
