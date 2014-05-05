package de.uni_luebeck.iti.smachapp.model;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

/**
 * Created by Morten Mey on 21.04.2014.
 */
public class StateMachine implements Iterable<State> {

    private LinkedList<State> states=new LinkedList<State>();

    @Override
    public Iterator<State> iterator(){
        return states.iterator();
    }

    public List<Transition> getTransitions(){
        List<Transition> trans=new LinkedList<Transition>();

        for(State x:this){
            trans.addAll(x.getTransitions());
        }

        return trans;
    }

    public void addState(State s){
        states.add(s);
    }

    public State getState(int i){
        return states.get(i);
    }

    public int getStateCount(){
        return states.size();
    }

    public void removeState(State s){
        if(s.isInitialState()){
            throw new IllegalArgumentException("You can not remove the initial State.");
        }
        for(State curr:this){
            Iterator<Transition> iter=curr.iterator();

            while(iter.hasNext()){
                if(iter.next().getFollowerState()==s){
                    iter.remove();
                }
            }
        }
        states.remove(s);
    }

    public void addTransition(Transition t){
        State s=t.getPreviousState();
        if(!states.contains(s)||!states.contains(t.getFollowerState())){
            throw new IllegalArgumentException("The previous or following state is not part of " +
                    "this StateMachine.");
        }

        s.addTransition(t);
    }

    public void removeTransition(Transition t){
        t.getPreviousState().removeTransition(t);
    }

    public List<Transition> getTransitions(State from, State to){
        List<Transition> trans=new ArrayList<Transition>(from.getTransitions());

        for(Iterator<Transition> iter=trans.iterator();iter.hasNext();){
            if(iter.next().getFollowerState()!=to){
                iter.remove();
            }
        }
        return trans;
    }
}
