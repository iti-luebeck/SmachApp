package de.uni_luebeck.iti.smachapp.model.undo;

import java.util.ArrayList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.model.StateMachine;
import de.uni_luebeck.iti.smachapp.model.Transition;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class DeleteTransitions implements UndoableAction {

    private List<Transition> transitions;
    private StateMachine machine;

    public DeleteTransitions(List<Transition> transitions, StateMachine machine) {
        this.transitions = new ArrayList<Transition>(transitions);
        this.machine = machine;
    }

    @Override
    public void undo() {
        for (Transition t : transitions) {
            machine.addTransition(t);
        }
    }

    @Override
    public void redo() {
        for (Transition t : transitions) {
            machine.removeTransition(t);
        }
    }
}
