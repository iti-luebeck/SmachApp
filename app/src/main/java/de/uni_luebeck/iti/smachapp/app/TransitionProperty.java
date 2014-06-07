package de.uni_luebeck.iti.smachapp.app;

import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.EditText;
import android.widget.LinearLayout;

import java.util.HashMap;
import java.util.List;

import de.uni_luebeck.iti.smachapp.model.Action;
import de.uni_luebeck.iti.smachapp.model.BeepRobot;
import de.uni_luebeck.iti.smachapp.model.ColorActuator;
import de.uni_luebeck.iti.smachapp.model.ColorSensor;
import de.uni_luebeck.iti.smachapp.model.IntActuator;
import de.uni_luebeck.iti.smachapp.model.IntSensor;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.view.ActuatorUI;
import de.uni_luebeck.iti.smachapp.view.ColorSelector;
import de.uni_luebeck.iti.smachapp.view.IntSlider;
import de.uni_luebeck.iti.smachapp.view.SensorUI;


public class TransitionProperty extends ActionBarActivity {

    private Transition transition;
    private BeepRobot robot;

    private HashMap<String,SensorUI> uis=new HashMap<String, SensorUI>();

    private static Transition setupTransition;
    private static BeepRobot setupRobot;

    public static void setupTransition(Transition t,BeepRobot r){
        setupTransition=t;
        setupRobot=r;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_transition_property);

        transition=setupTransition;
        robot=setupRobot;

        ((EditText)findViewById(R.id.transitionName)).setText(transition.getName());

        LinearLayout container=(LinearLayout)findViewById(R.id.sensorContainer);

        for(IntSensor sen:robot.getIntSensors()){
            IntSlider slider=new IntSlider(this,sen);
            container.addView(slider);
            uis.put(sen.getKey(),slider);
            slider.setToGuard(transition.getSmachableGuard());
        }

        for(ColorSensor sen:robot.getColorSensors()){
            ColorSelector sel=new ColorSelector(this,sen);
            container.addView(sel);
            uis.put(sen.getKey(),sel);
            sel.setToGuard(transition.getSmachableGuard());
        }
    }

    @Override
    protected void onPause(){
        super.onPause();

        String newName=((EditText)findViewById(R.id.transitionName)).getText().toString();

        if(!newName.isEmpty()){
            transition.setName(newName);
        }

        transition.getSmachableGuard().clear();

        for(SensorUI ui:uis.values()){
            if(ui.isChecked()){
                ui.fillGuard(transition.getSmachableGuard());
            }
        }
    }
}