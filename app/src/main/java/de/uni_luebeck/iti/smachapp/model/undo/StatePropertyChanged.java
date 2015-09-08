/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of Lübeck
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package de.uni_luebeck.iti.smachapp.model.undo;

import android.content.Context;
import android.widget.Toast;

import java.util.Hashtable;
import java.util.Map;

import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.Action;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.StateMachine;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class StatePropertyChanged implements UndoableAction {

    private State state;
    private StateMachine machine;
    private Context context;

    private String oldName;
    private String newName;

    private Hashtable<String, Integer> oldActions = new Hashtable<String, Integer>();
    private Hashtable<String, Integer> newActions = new Hashtable<String, Integer>();

    public StatePropertyChanged(State state, StateMachine machine,Context context) {
        this.context=context;
        this.state = state;
        this.machine = machine;
        oldName = state.getName();
        for (Action a : state.getActions()) {
            oldActions.put(a.getActuatorName(), a.getValue());
        }
    }

    public boolean operationComplete() {
        newName = state.getName();

        boolean change = !newName.equals(oldName);

        for (Action a : state.getActions()) {
            newActions.put(a.getActuatorName(), a.getValue());

            if(!(oldActions.containsKey(a.getActuatorName()) && oldActions.get(a.getActuatorName()).equals(a.getValue()))){
                change=true;
            }
        }
        return change;
    }

    @Override
    public void undo() {
        state.setName(oldName);
        state.getActions().clear();
        for (Map.Entry<String, Integer> entry : oldActions.entrySet()) {
            state.addAction(new Action(entry.getKey(), entry.getValue()));
        }
        machine.fixTransitionEnds(state);
        Toast.makeText(context, R.string.undid_state_properties,Toast.LENGTH_SHORT).show();
    }

    @Override
    public void redo() {
        state.setName(newName);
        state.getActions().clear();
        for (Map.Entry<String, Integer> entry : newActions.entrySet()) {
            state.addAction(new Action(entry.getKey(), entry.getValue()));
        }
        machine.fixTransitionEnds(state);
        Toast.makeText(context,R.string.redid_state_properties,Toast.LENGTH_SHORT).show();;
    }

    public State getState() {
        return state;
    }
}
