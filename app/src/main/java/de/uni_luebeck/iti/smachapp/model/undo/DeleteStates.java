package de.uni_luebeck.iti.smachapp.model.undo;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.StateMachine;
import de.uni_luebeck.iti.smachapp.model.Transition;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class DeleteStates implements UndoableAction {

    private List<State> states;
    private List<Transition> incoming;
    private StateMachine machine;

    public DeleteStates(List<State> states, StateMachine machine) {
        this.states = new ArrayList<State>(states);
        this.machine = machine;
        incoming = new LinkedList<Transition>();

        for (State s : states) {
            incoming.addAll(machine.getIncomingTransitions(s));
        }
    }


    @Override
    public void undo() {
        for (State s : states) {
            machine.addState(s);
        }
        for (Transition t : incoming) {
            machine.addTransition(t);
        }
    }

    @Override
    public void redo() {
        for (State s : states) {
            machine.removeState(s);
        }
    }
}
