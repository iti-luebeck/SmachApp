package de.uni_luebeck.iti.smachapp.model.undo;

import de.uni_luebeck.iti.smachapp.model.StateMachine;
import de.uni_luebeck.iti.smachapp.model.Transition;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class AddTransition implements UndoableAction {

    private Transition transition;
    private StateMachine machine;

    public AddTransition(Transition transition, StateMachine machine) {
        this.transition = transition;
        this.machine = machine;
    }

    @Override
    public void undo() {
        machine.removeTransition(transition);
    }

    @Override
    public void redo() {
        machine.addTransition(transition);
    }
}
