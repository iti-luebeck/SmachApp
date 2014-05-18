package de.uni_luebeck.iti.smachapp.app;


import android.graphics.Color;
import android.graphics.drawable.ColorDrawable;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;

import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.SeekBar;


import java.util.HashMap;
import java.util.List;

import de.uni_luebeck.iti.smachapp.model.Action;
import de.uni_luebeck.iti.smachapp.model.BeepRobot;
import de.uni_luebeck.iti.smachapp.model.ColorActuator;
import de.uni_luebeck.iti.smachapp.model.IntActuator;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.view.ActuatorUI;
import de.uni_luebeck.iti.smachapp.view.ColorSelector;
import de.uni_luebeck.iti.smachapp.view.IntSlider;


public class StateProperty extends ActionBarActivity{

    private State state;
    private BeepRobot robot;

    private HashMap<String,ActuatorUI> uis=new HashMap<String, ActuatorUI>();

    private static State setupState;
    private static BeepRobot setupRobot;

    public static void setupState(State s,BeepRobot r){
        setupState=s;
        setupRobot=r;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_state_property);

        state=setupState;
        robot=setupRobot;

        ((EditText)findViewById(R.id.stateName)).setText(state.getName());

        LinearLayout container=(LinearLayout)findViewById(R.id.actuatorContainer);

        for(IntActuator act:robot.getIntActuators()){
            IntSlider slider=new IntSlider(this,act);
            container.addView(slider);
            uis.put(act.getKey(),slider);
        }

        for(ColorActuator act:robot.getColorActuators()){
            ColorSelector sel=new ColorSelector(this,act);
            container.addView(sel);
            uis.put(act.getKey(),sel);
        }

        for(Action a:state.getActions()){
            uis.get(a.getKey()).setToAction(a);
        }

    }


    @Override
    protected void onPause(){
        super.onPause();

        String newName=((EditText)findViewById(R.id.stateName)).getText().toString();

        if(!newName.isEmpty()){
            state.setName(newName);
        }

        List<Action> actions=state.getActions();
        actions.clear();

        for(ActuatorUI ui:uis.values()){
            if(ui.isChecked()){
                actions.add(ui.createAction());
            }
        }
    }


}
