package de.uni_luebeck.iti.smachapp.view;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.text.InputType;
import android.view.KeyEvent;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.view.inputmethod.InputMethodManager;
import android.widget.ArrayAdapter;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;

import java.util.List;

import de.uni_luebeck.iti.smachapp.app.R;
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

    private Context context;

    private class MyOnClickListener implements OnClickListener {

        @Override
        public void onClick(View v) {
            AlertDialog.Builder builder = new AlertDialog.Builder(context);

            builder.setTitle(R.string.enter_a_value);
            final EditText text = new EditText(context);
            text.setInputType(InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_FLAG_SIGNED);
            text.setImeOptions(EditorInfo.IME_ACTION_DONE);
            text.setText(disp.getText());
            text.selectAll();
            builder.setView(text);

            builder.setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialogInterface, int i) {
                    InputMethodManager keyboard = (InputMethodManager)
                            context.getSystemService(Context.INPUT_METHOD_SERVICE);
                    keyboard.hideSoftInputFromWindow(text.getWindowToken(), 0);
                    handleOk(text.getText().toString());
                }
            });

            builder.setNegativeButton(R.string.disrecard, new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialogInterface, int i) {
                    InputMethodManager keyboard = (InputMethodManager)
                            context.getSystemService(Context.INPUT_METHOD_SERVICE);
                    keyboard.hideSoftInputFromWindow(text.getWindowToken(), 0);
                }
            });

            final AlertDialog dia = builder.show();

            text.postDelayed(new Runnable() {
                @Override
                public void run() {
                    InputMethodManager keyboard = (InputMethodManager)
                            context.getSystemService(Context.INPUT_METHOD_SERVICE);
                    keyboard.showSoftInput(text, 0);
                }
            }, 200);


            text.setOnEditorActionListener(new TextView.OnEditorActionListener() {
                @Override
                public boolean onEditorAction(TextView textView, int i, KeyEvent keyEvent) {
                    dia.dismiss();
                    InputMethodManager keyboard = (InputMethodManager)
                            context.getSystemService(Context.INPUT_METHOD_SERVICE);
                    keyboard.hideSoftInputFromWindow(text.getWindowToken(), 0);
                    handleOk(text.getText().toString());
                    return true;
                }
            });
        }

        private void handleOk(String text) {
            if (!text.isEmpty() && text.charAt(0) == '+') {
                text = text.substring(1);
            }
            try {
                if (actuator != null) {
                    slider.setProgress(Integer.parseInt(text) - actuator.getMin());
                } else {
                    slider.setProgress(Integer.parseInt(text) - sensor.getMin());
                }
            } catch (IllegalArgumentException ex) {
            }

        }
    }


    public IntSlider(Context context, BeepMotorActuator actuator) {
        super(context);
        this.context = context;
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
        disp.setOnClickListener(new MyOnClickListener());

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
        this.context = context;
        actuator = null;

        LinearLayout.LayoutParams param = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, LinearLayout.LayoutParams.WRAP_CONTENT);
        this.setLayoutParams(param);


        check = new CheckBox(context);
        check.setText(sensor.getName());
        disp = new TextView(context);
        param = new LinearLayout.LayoutParams(100, LinearLayout.LayoutParams.WRAP_CONTENT);
        disp.setLayoutParams(param);
        disp.setOnClickListener(new MyOnClickListener());

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
        param = new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT);
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
    public void setToAction(Action action, boolean checked) {
        if (!action.getActuatorName().equals(actuator.getName())) {
            throw new IllegalArgumentException("The action is not for this actuator.");
        }

        slider.setProgress(action.getValue() - actuator.getMin());
        check.setChecked(checked);
    }

    @Override
    public Action createAction() {
        return new Action(actuator.getName(), slider.getProgress() + actuator.getMin());
    }

    public void setToGuard(Guard guard, boolean checked) {

        List<String> names = guard.getSensorNames();
        for (int i = 0; i < names.size(); i++) {
            if (names.get(i).equals(sensor.getName())) {
                check.setChecked(checked);
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
