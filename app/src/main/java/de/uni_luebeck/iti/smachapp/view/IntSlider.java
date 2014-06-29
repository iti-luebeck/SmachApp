package de.uni_luebeck.iti.smachapp.view;

import android.content.Context;
import android.widget.ArrayAdapter;
import android.widget.CheckBox;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;

import java.util.List;

import de.uni_luebeck.iti.smachapp.model.Action;
import de.uni_luebeck.iti.smachapp.model.BeepIRSensor;
import de.uni_luebeck.iti.smachapp.model.BeepMotorActuator;
import de.uni_luebeck.iti.smachapp.model.Guard;

/**
 * Created by Morten Mey on 18.05.2014.
 */
public class IntSlider extends LinearLayout implements SeekBar.OnSeekBarChangeListener, ActuatorUI, SensorUI {

    private BeepMotorActuator actuator;
    private BeepIRSensor sensor;
    private CheckBox check;
    private SeekBar slider;
    private TextView disp;

    private Spinner spinner;
    private ArrayAdapter<String> adapter;


    public IntSlider(Context context, BeepMotorActuator actuator) {
        super(context);
        this.actuator = actuator;
        sensor = null;
        spinner = null;
        adapter = null;

        LinearLayout.LayoutParams param = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        this.setLayoutParams(param);


        check = new CheckBox(context);
        check.setText(actuator.getName());
        disp = new TextView(context);
        param = new LinearLayout.LayoutParams(100, LinearLayout.LayoutParams.WRAP_CONTENT);
        disp.setLayoutParams(param);

        slider = new SeekBar(context);
        slider.setMax(actuator.getMax() - actuator.getMin());
        slider.setOnSeekBarChangeListener(this);
        param = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT);
        param.weight = 1;
        slider.setLayoutParams(param);


        if (0 <= actuator.getMax() && 0 >= actuator.getMin()) {
            slider.setProgress(-actuator.getMin());
            disp.setText(Integer.toString(-actuator.getMin()));
        } else {
            slider.setProgress((actuator.getMax() - actuator.getMin()) / 2);
            disp.setText(Integer.toString((actuator.getMax() - actuator.getMin()) / 2));
        }

        this.addView(check);
        this.addView(slider);
        this.addView(disp);
    }

    public IntSlider(Context context, BeepIRSensor sensor) {
        super(context);
        this.sensor = sensor;
        actuator = null;

        LinearLayout.LayoutParams param = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        this.setLayoutParams(param);


        check = new CheckBox(context);
        check.setText(sensor.getName());
        disp = new TextView(context);
        param = new LinearLayout.LayoutParams(100, LinearLayout.LayoutParams.WRAP_CONTENT);
        disp.setLayoutParams(param);

        slider = new SeekBar(context);
        slider.setMax(sensor.getMax() - sensor.getMin());
        slider.setOnSeekBarChangeListener(this);
        param = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT);
        param.weight = 1;
        slider.setLayoutParams(param);

        adapter = new ArrayAdapter<String>(context, android.R.layout.simple_spinner_item);
        adapter.add(">");
        adapter.add(">=");
        adapter.add("<");
        adapter.add("<=");
        adapter.add("==");
        adapter.add("!=");

        spinner = new Spinner(context);
        spinner.setAdapter(adapter);
        spinner.setSelection(0);
        param = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.MATCH_PARENT);
        spinner.setLayoutParams(param);

        if (0 <= sensor.getMax() && 0 >= sensor.getMin()) {
            slider.setProgress(-sensor.getMin());
            disp.setText(Integer.toString(-sensor.getMin()));
        } else {
            slider.setProgress((sensor.getMax() - sensor.getMin()) / 2);
            disp.setText(Integer.toString((sensor.getMax() - sensor.getMin()) / 2));
        }

        this.addView(check);
        this.addView(spinner);
        this.addView(slider);
        this.addView(disp);
    }


    @Override
    public void setToAction(Action action) {
        if (!action.getActuatorName().equals(actuator.getName())) {
            throw new IllegalArgumentException("The action is not for this actuator.");
        }

        slider.setProgress(action.getValue() - actuator.getMin());
        check.setChecked(true);
    }

    @Override
    public Action createAction() {
        return new Action(actuator.getName(), slider.getProgress() + actuator.getMin());
    }

    public void setToGuard(Guard guard) {

        List<String> names = guard.getSensorNames();
        for (int i = 0; i < names.size(); i++) {
            if (names.get(i).equals(sensor.getName())) {
                check.setChecked(true);
                slider.setProgress(guard.getCompValues().get(i) - sensor.getMin());
                spinner.setSelection(adapter.getPosition(guard.getOperators().get(i)));
            }
        }
    }

    public void fillGuard(Guard guard) {
        guard.getSensorNames().add(sensor.getName());
        guard.getCompValues().add(slider.getProgress() + sensor.getMin());
        guard.getOperators().add((String) spinner.getSelectedItem());
    }

    @Override
    public boolean isChecked() {
        return check.isChecked();
    }

    @Override
    public void onProgressChanged(SeekBar seekBar, final int i, boolean b) {
        disp.post(new Runnable() {
            @Override
            public void run() {
                if (actuator != null) {
                    disp.setText(Integer.toString(i + actuator.getMin()));
                } else {
                    disp.setText(Integer.toString(i + sensor.getMin()));
                }
            }
        });
    }

    @Override
    public void onStartTrackingTouch(SeekBar seekBar) {

    }

    @Override
    public void onStopTrackingTouch(SeekBar seekBar) {

    }
}
