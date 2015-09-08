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
import android.text.TextWatcher;
import android.view.View;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import java.util.HashMap;

import de.uni_luebeck.iti.smachapp.model.BeepColorSensor;
import de.uni_luebeck.iti.smachapp.model.BeepIRSensor;
import de.uni_luebeck.iti.smachapp.model.BeepRobot;
import de.uni_luebeck.iti.smachapp.model.StateMachine;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.view.ColorSelector;
import de.uni_luebeck.iti.smachapp.view.IntSlider;
import de.uni_luebeck.iti.smachapp.view.SensorUI;


public class TransitionProperty extends Activity implements TextWatcher {

    private Transition transition;
    private StateMachine machine;
    private int priority;
    private int maxPriority;

    private HashMap<String, SensorUI> uis = new HashMap<String, SensorUI>();

    private TextView priorityField;

    private static Transition setupTransition;
    private static BeepRobot setupRobot;
    private static StateMachine setupMachine;
    private static int setupPriority;
    private static int setupMaxPriority;

    public static int getPriority() {
        return setupPriority;
    }

    public static void setupTransition(Transition t, BeepRobot r, StateMachine m, int pri, int maxPri) {
        setupTransition = t;
        setupRobot = r;
        setupMachine = m;
        setupPriority = pri;
        setupMaxPriority = maxPri;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_transition_property);

        transition = setupTransition;
        machine = setupMachine;
        BeepRobot robot = setupRobot;
        priority = setupPriority;
        maxPriority = setupMaxPriority;

        priorityField = (TextView) findViewById(R.id.priority);
        priorityField.setText(String.valueOf(priority));

        LinearLayout container = (LinearLayout) findViewById(R.id.sensorContainer);

        for (BeepIRSensor sen : robot.getIntSensors()) {
            IntSlider slider = new IntSlider(this, sen);
            container.addView(slider);
            uis.put(sen.getName(), slider);
            slider.setToGuard(transition.getSmachableGuard(), true);
            slider.setToGuard(transition.getDisabledGuard(), false);
        }

        for (BeepColorSensor sen : robot.getColorSensors()) {
            ColorSelector sel = new ColorSelector(this, sen);
            container.addView(sel);
            uis.put(sen.getName(), sel);
            sel.setToGuard(transition.getSmachableGuard(), true);
            sel.setToGuard(transition.getDisabledGuard(), false);
        }

        EditText text = (EditText) findViewById(R.id.transitionName);
        text.setText(transition.getLabel());
        text.addTextChangedListener(this);
    }

    @Override
    public void onBackPressed() {
        String newName = ((EditText) findViewById(R.id.transitionName)).getText().toString().trim();

        if (!newName.equals(transition.getLabel()) && !newName.isEmpty()) {
            if (machine.getTransition(newName) == null) {
                transition.setLabel(newName);
            } else {
                Toast toast = Toast.makeText(this, R.string.nameAlreadyExists, Toast.LENGTH_LONG);
                toast.show();
                return;
            }
        }

        transition.getSmachableGuard().clear();
        transition.getDisabledGuard().clear();

        for (SensorUI ui : uis.values()) {
            if (ui.isChecked()) {
                ui.fillGuard(transition.getSmachableGuard());
            } else {
                ui.fillGuard(transition.getDisabledGuard());
            }
        }
        setupPriority = priority;
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
        String name = editable.toString().trim();
        if (!name.equals(transition.getLabel()) && machine.getTransition(name) != null) {
            Toast toast = Toast.makeText(this, R.string.nameAlreadyExists, Toast.LENGTH_LONG);
            toast.show();
        }

        if (name.isEmpty()) {
            Toast toast = Toast.makeText(this, R.string.empty_name_in_property, Toast.LENGTH_LONG);
            toast.show();
        }
    }

    public void increasePriority(View view) {
        priority = priority + 1;

        if(priority>maxPriority){
            priority=maxPriority;
            Toast.makeText(this,R.string.max_priority_reached,Toast.LENGTH_SHORT).show();
        }

        priorityField.setText(String.valueOf(priority));
    }

    public void decreasePriority(View view) {
        priority = priority - 1;

        if(priority<0){
            priority=0;
            Toast.makeText(this,R.string.min_priority_reached,Toast.LENGTH_SHORT).show();
        }

        priorityField.setText(String.valueOf(priority));
    }
}
