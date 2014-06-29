package de.uni_luebeck.iti.smachapp.app;


import android.app.Activity;
import android.os.Bundle;
import android.widget.EditText;
import android.widget.LinearLayout;

import java.util.HashMap;
import java.util.List;

import de.uni_luebeck.iti.smachapp.model.Action;
import de.uni_luebeck.iti.smachapp.model.BeepColorRGBActuator;
import de.uni_luebeck.iti.smachapp.model.BeepMotorActuator;
import de.uni_luebeck.iti.smachapp.model.BeepRobot;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.view.ActuatorUI;
import de.uni_luebeck.iti.smachapp.view.ColorSelector;
import de.uni_luebeck.iti.smachapp.view.IntSlider;


public class StateProperty extends Activity {

    private State state;

    private HashMap<String, ActuatorUI> uis = new HashMap<String, ActuatorUI>();

    private static State setupState;
    private static BeepRobot setupRobot;

    public static void setupState(State s, BeepRobot r) {
        setupState = s;
        setupRobot = r;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_state_property);

        state = setupState;
        BeepRobot robot = setupRobot;

        ((EditText) findViewById(R.id.stateName)).setText(state.getName());

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
            uis.get(a.getActuatorName()).setToAction(a);
        }

    }


    @Override
    protected void onPause() {
        super.onPause();

        String newName = ((EditText) findViewById(R.id.stateName)).getText().toString();

        if (!newName.isEmpty()) {
            state.setName(newName);
        }

        List<Action> actions = state.getActions();
        actions.clear();

        for (ActuatorUI ui : uis.values()) {
            if (ui.isChecked()) {
                actions.add(ui.createAction());
            }
        }
    }


}
