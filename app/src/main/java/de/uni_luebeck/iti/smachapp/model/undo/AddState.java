package de.uni_luebeck.iti.smachapp.model.undo;

import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.StateMachine;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class AddState implements UndoableAction {

    private State state;
    private StateMachine machine;

    public AddState(State state, StateMachine machine) {
        this.state = state;
        this.machine = machine;
    }

    @Override
    public void undo() {
        machine.removeState(state);
    }

    @Override
    public void redo() {
        machine.addState(state);
    }
}
