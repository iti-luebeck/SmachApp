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
