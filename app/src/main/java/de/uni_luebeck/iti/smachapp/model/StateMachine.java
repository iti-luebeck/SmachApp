package de.uni_luebeck.iti.smachapp.model;

import android.graphics.PointF;
import android.graphics.RectF;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.view.StateMachineView;

/**
 * A simple Finite State Machine.
 * Created by Morten Mey on 21.04.2014.
 */
public class StateMachine implements Iterable<State> {

    private LinkedList<State> states = new LinkedList<State>();
    private String name;

    public StateMachine(String name) {
        this.name = name;
    }

    @Override
    public Iterator<State> iterator() {
        return states.iterator();
    }

    public List<Transition> getTransitions() {
        List<Transition> trans = new LinkedList<Transition>();

        for (State x : this) {
            trans.addAll(x.getTransitions());
        }

        return trans;
    }

    public void addState(State s) {
        states.add(s);
    }

    public void removeState(State s) {
        if (s.isInitialState()) {
            throw new IllegalArgumentException("You can not remove the initial State.");
        }
        for (State curr : this) {
            Iterator<Transition> iter = curr.iterator();

            while (iter.hasNext()) {
                if (iter.next().getFollowerState() == s) {
                    iter.remove();
                }
            }
        }
        states.remove(s);
    }

    public void addTransition(Transition t) {
        State s = t.getPreviousState();
        if (!states.contains(s) || !states.contains(t.getFollowerState())) {
            throw new IllegalArgumentException("The previous or following state is not part of " +
                    "this StateMachine.");
        }

        s.addTransition(t);
    }

    public void removeTransition(Transition t) {
        t.getPreviousState().removeTransition(t);
    }

    public List<Transition> getTransitions(State from, State to) {
        List<Transition> trans = new ArrayList<Transition>(from.getTransitions());

        for (Iterator<Transition> iter = trans.iterator(); iter.hasNext(); ) {
            if (iter.next().getFollowerState() != to) {
                iter.remove();
            }
        }
        return trans;
    }

    public String getName() {
        return name;
    }

    public List<State> getStates() {
        return states;
    }

    public List<Transition> getIncomingTransitions(State state) {
        List<Transition> res = new LinkedList<Transition>();

        for (State curr : this) {
            for (Transition trans : curr) {
                if (trans.getFollowerState() == state) {
                    res.add(trans);
                }
            }
        }
        return res;
    }

    public void setName(String name) {
        this.name = name;
    }

    public State getState(String stateName) {
        for (State s : this) {
            if (s.getName().equals(stateName)) {
                return s;
            }
        }
        return null;
    }

    public void fixTransitionEnds(State s) {

        if(s==null){
            return;
        }
        List<Transition> trans = getIncomingTransitions(s);

        RectF rect = new RectF();
        StateMachineView.getCurrentView().getStateRect(s, rect);

        for (Transition t : trans) {
            List<PointF> points = t.getPath().getPoints();
            BezierPath.moveOnOval(rect, points.get(points.size() - 1));
        }
        for (Transition t : s) {
            List<PointF> points = t.getPath().getPoints();
            BezierPath.moveOnOval(rect, points.get(0));
        }

    }

    public Transition getTransition(String name) {
        for(State s:this){
            for(Transition t:s){
                if(t.getLabel().equals(name)){
                    return t;
                }
            }
        }
        return null;
    }
}
