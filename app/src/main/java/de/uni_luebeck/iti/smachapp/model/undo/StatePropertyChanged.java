package de.uni_luebeck.iti.smachapp.model.undo;

import java.util.ArrayList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.model.Action;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.StateMachine;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class StatePropertyChanged implements UndoableAction {

    private State state;
    private StateMachine machine;

    private String oldName;
    private String newName;

    private List<Action> oldActions;
    private List<Action> newActions;

    public StatePropertyChanged(State state, StateMachine machine) {
        this.state = state;
        this.machine = machine;
        oldName = state.getName();
        oldActions = copyActions(state.getActions());
    }

    public boolean operationComplete() {
        newName = state.getName();
        newActions = copyActions(state.getActions());

        return true;
    }


    private List<Action> copyActions(List<Action> actions) {
        List<Action> res = new ArrayList<Action>(actions.size());

        for (Action a : actions) {
            res.add(new Action(a.getActuatorName(), a.getValue()));
        }
        return res;
    }

    @Override
    public void undo() {
        state.setName(oldName);
        state.getActions().clear();
        state.getActions().addAll(copyActions(oldActions));
        machine.fixTransitionEnds(state);
    }

    @Override
    public void redo() {
        state.setName(newName);
        state.getActions().clear();
        state.getActions().addAll(copyActions(newActions));
        machine.fixTransitionEnds(state);
    }
}
