package de.uni_luebeck.iti.smachapp.app;

import android.app.Activity;
import android.os.Bundle;
import android.text.Editable;
import android.text.TextWatcher;
import android.widget.EditText;
import android.widget.LinearLayout;
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

    private HashMap<String, SensorUI> uis = new HashMap<String, SensorUI>();

    private static Transition setupTransition;
    private static BeepRobot setupRobot;
    private static StateMachine setupMachine;

    public static void setupTransition(Transition t, BeepRobot r, StateMachine m) {
        setupTransition = t;
        setupRobot = r;
        setupMachine = m;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_transition_property);

        transition = setupTransition;
        machine = setupMachine;
        BeepRobot robot = setupRobot;

        LinearLayout container = (LinearLayout) findViewById(R.id.sensorContainer);

        for (BeepIRSensor sen : robot.getIntSensors()) {
            IntSlider slider = new IntSlider(this, sen);
            container.addView(slider);
            uis.put(sen.getName(), slider);
            slider.setToGuard(transition.getSmachableGuard());
        }

        for (BeepColorSensor sen : robot.getColorSensors()) {
            ColorSelector sel = new ColorSelector(this, sen);
            container.addView(sel);
            uis.put(sen.getName(), sel);
            sel.setToGuard(transition.getSmachableGuard());
        }

        EditText text = (EditText) findViewById(R.id.transitionName);
        text.setText(transition.getLabel());
        text.addTextChangedListener(this);
    }

    @Override
    public void onBackPressed() {
        String newName = ((EditText) findViewById(R.id.transitionName)).getText().toString();

        if (!newName.equals(transition.getLabel())) {
            if (machine.getTransition(newName) == null) {
                transition.setLabel(newName);
            }else{
                Toast toast=Toast.makeText(this,R.string.nameAlreadyExists,Toast.LENGTH_LONG);
                toast.show();
                return;
            }
        }

        transition.getSmachableGuard().clear();

        for (SensorUI ui : uis.values()) {
            if (ui.isChecked()) {
                ui.fillGuard(transition.getSmachableGuard());
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
        String name = editable.toString();
        if (!name.equals(transition.getLabel()) && machine.getTransition(name) != null) {
            Toast toast = Toast.makeText(this, R.string.nameAlreadyExists, Toast.LENGTH_LONG);
            toast.show();
        }
    }
}
