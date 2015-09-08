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
package de.uni_luebeck.iti.smachapp.app;


import android.app.Activity;
import android.os.Bundle;
import android.text.Editable;
import android.text.InputType;
import android.text.TextWatcher;
import android.view.KeyEvent;
import android.view.inputmethod.EditorInfo;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import java.util.HashMap;
import java.util.List;

import de.uni_luebeck.iti.smachapp.model.Action;
import de.uni_luebeck.iti.smachapp.model.BeepColorRGBActuator;
import de.uni_luebeck.iti.smachapp.model.BeepMotorActuator;
import de.uni_luebeck.iti.smachapp.model.BeepRobot;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.StateMachine;
import de.uni_luebeck.iti.smachapp.view.ActuatorUI;
import de.uni_luebeck.iti.smachapp.view.ColorSelector;
import de.uni_luebeck.iti.smachapp.view.IntSlider;


public class StateProperty extends Activity implements TextWatcher {

    private State state;
    private StateMachine machine;

    private HashMap<String, ActuatorUI> uis = new HashMap<String, ActuatorUI>();

    private static State setupState;
    private static BeepRobot setupRobot;
    private static StateMachine setupMachine;

    public static void setupState(State s, BeepRobot r, StateMachine m) {
        setupState = s;
        setupRobot = r;
        setupMachine = m;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_state_property);

        state = setupState;
        machine = setupMachine;
        BeepRobot robot = setupRobot;

        LinearLayout container = (LinearLayout) findViewById(R.id.actuatorContainer);

        for (BeepMotorActuator act : robot.getMotorActuators()) {
            IntSlider slider = new IntSlider(this, act);
            container.addView(slider);
            uis.put(act.getName(), slider);
        }

        for (BeepColorRGBActuator act : robot.getColorRGBActuators()) {
            ColorSelector sel = new ColorSelector(this, act);
            container.addView(sel);
            uis.put(act.getName(), sel);
        }

        for (Action a : state.getActions()) {
            uis.get(a.getActuatorName()).setToAction(a,true);
        }

        for(Action a:state.getDisabledActions()){
            uis.get(a.getActuatorName()).setToAction(a,false);
        }

        EditText text=(EditText)findViewById(R.id.stateName);
        text.setText(state.getName());
        text.addTextChangedListener(this);

    }


    @Override
    public void onBackPressed() {
        String newName = ((EditText) findViewById(R.id.stateName)).getText().toString().trim();

        if (!newName.equals(state.getName()) && !newName.isEmpty()) {
            if (machine.getState(newName) == null) {
                state.setName(newName);
            }else{
                Toast toast=Toast.makeText(this,R.string.nameAlreadyExists,Toast.LENGTH_LONG);
                toast.show();
                return;
            }
        }

        List<Action> actions = state.getActions();
        actions.clear();

        List<Action> disabledActions=state.getDisabledActions();
        disabledActions.clear();

        for (ActuatorUI ui : uis.values()) {
            if (ui.isChecked()) {
                actions.add(ui.createAction());
            }else{
                disabledActions.add(ui.createAction());
            }
        }

        finish();
    }

    @Override
    public void beforeTextChanged(CharSequence charSequence, int i, int i2, int i3) {

    }

    @Override
    public void onTextChanged(CharSequence charSequence, int i, int i2, int i3) {

    }

    @Override
    public void afterTextChanged(Editable editable) {
        String name=editable.toString().trim();

        if(!name.equals(state.getName()) && machine.getState(name)!=null){
            Toast toast=Toast.makeText(this,R.string.nameAlreadyExists,Toast.LENGTH_LONG);
            toast.show();
        }

        if(name.isEmpty()){
            Toast toast=Toast.makeText(this,R.string.empty_name_in_property,Toast.LENGTH_LONG);
            toast.show();
        }
    }
}
