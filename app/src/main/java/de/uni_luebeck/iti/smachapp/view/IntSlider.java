package de.uni_luebeck.iti.smachapp.view;

import android.content.Context;
import android.widget.ArrayAdapter;
import android.widget.CheckBox;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;

import java.util.List;

import de.uni_luebeck.iti.smachGenerator.Operator;
import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.Action;
import de.uni_luebeck.iti.smachapp.model.Guard;
import de.uni_luebeck.iti.smachapp.model.IntActuator;
import de.uni_luebeck.iti.smachapp.model.IntSensor;

/**
 * Created by Morten Mey on 18.05.2014.
 */
public class IntSlider extends LinearLayout implements SeekBar.OnSeekBarChangeListener, ActuatorUI, SensorUI {

    private IntActuator actuator;
    private IntSensor sensor;
    private CheckBox check;
    private SeekBar slider;
    private TextView disp;

    private Spinner spinner;
    private ArrayAdapter<Operator> adapter;


    public IntSlider(Context context, IntActuator actuator) {
        super(context);
        this.actuator = actuator;
        sensor = null;
        spinner = null;
        adapter = null;

        LinearLayout.LayoutParams param = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        this.setLayoutParams(param);


        check = new CheckBox(context);
        check.setText(actuator.getKey());
        disp = new TextView(context);
        param = new LinearLayout.LayoutParams(60, LinearLayout.LayoutParams.WRAP_CONTENT);
        disp.setLayoutParams(param);

        slider = new SeekBar(context);
        slider.setMax(actuator.getMax() - actuator.getMin());
        slider.setOnSeekBarChangeListener(this);
        param = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT);
        param.weight = 1;
        slider.setLayoutParams(param);


        if (0 < actuator.getMax() && 0 > actuator.getMin()) {
            slider.setProgress(-actuator.getMin());
        } else {
            slider.setProgress((actuator.getMax() - actuator.getMin()) / 2);
        }

        this.addView(check);
        this.addView(slider);
        this.addView(disp);
    }

    public IntSlider(Context context, IntSensor sensor) {
        super(context);
        this.sensor = sensor;
        actuator = null;

        LinearLayout.LayoutParams param = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        this.setLayoutParams(param);


        check = new CheckBox(context);
        check.setText(sensor.getKey());
        disp = new TextView(context);
        param = new LinearLayout.LayoutParams(60, LinearLayout.LayoutParams.WRAP_CONTENT);
        disp.setLayoutParams(param);

        slider = new SeekBar(context);
        slider.setMax(sensor.getMax() - sensor.getMin());
        slider.setOnSeekBarChangeListener(this);
        param = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT);
        param.weight = 1;
        slider.setLayoutParams(param);

        adapter = new ArrayAdapter<Operator>(context, R.layout.support_simple_spinner_dropdown_item);
        adapter.add(Operator.BIGGER);
        adapter.add(Operator.BIGGER_EQUAL);
        adapter.add(Operator.SMALLER);
        adapter.add(Operator.SMALLER_EQUAL);
        adapter.add(Operator.EQUAL);
        adapter.add(Operator.NOT_EQUAL);

        spinner = new Spinner(context);
        spinner.setAdapter(adapter);
        spinner.setSelection(0);
        param = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.MATCH_PARENT);
        spinner.setLayoutParams(param);

        if (0 < sensor.getMax() && 0 > sensor.getMin()) {
            slider.setProgress(-sensor.getMin());
        } else {
            slider.setProgress((sensor.getMax() - sensor.getMin()) / 2);
        }

        this.addView(check);
        this.addView(spinner);
        this.addView(slider);
        this.addView(disp);
    }


    @Override
    public void setToAction(Action action) {
        if (!action.getKey().equals(actuator.getKey())) {
            throw new IllegalArgumentException("The action is not for this actuator.");
        }

        slider.setProgress(action.getValue() - actuator.getMin());
        check.setChecked(true);
    }

    @Override
    public Action createAction() {
        return new Action(actuator.getKey(), slider.getProgress() + actuator.getMin());
    }

    public void setToGuard(Guard guard) {

        List<String> names = guard.getSensorNames();
        for (int i = 0; i < names.size(); i++) {
            if (names.get(i).equals(sensor.getKey())) {
                check.setChecked(true);
                slider.setProgress(guard.getCompValues().get(i) - sensor.getMin());
                spinner.setSelection(adapter.getPosition(guard.getOperators().get(i)));
            }
        }
    }

    public void fillGuard(Guard guard) {
        guard.getSensorNames().add(sensor.getKey());
        guard.getCompValues().add(slider.getProgress() + sensor.getMin());
        guard.getOperators().add((Operator) spinner.getSelectedItem());
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
