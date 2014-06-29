package de.uni_luebeck.iti.smachapp.model.undo;

import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.StateMachine;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class ChangeInitialState implements UndoableAction {

    private State oldInitialState;
    private State newInitialState;
    private StateMachine machine;

    public ChangeInitialState(State oldInitialState, State newInitialState, StateMachine machine) {
        this.oldInitialState = oldInitialState;
        this.newInitialState = newInitialState;
        this.machine = machine;
    }

    @Override
    public void undo() {
        newInitialState.setInitialState(false);
        oldInitialState.setInitialState(true);
        machine.fixTransitionEnds(newInitialState);
        machine.fixTransitionEnds(oldInitialState);
    }

    @Override
    public void redo() {
        newInitialState.setInitialState(true);
        oldInitialState.setInitialState(false);
        machine.fixTransitionEnds(oldInitialState);
        machine.fixTransitionEnds(newInitialState);
    }
}
