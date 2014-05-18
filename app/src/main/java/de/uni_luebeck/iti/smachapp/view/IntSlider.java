package de.uni_luebeck.iti.smachapp.view;

import android.content.Context;
import android.view.ViewGroup;
import android.widget.CheckBox;
import android.widget.LinearLayout;
import android.widget.SeekBar;
import android.widget.TextView;

import de.uni_luebeck.iti.smachapp.model.Action;
import de.uni_luebeck.iti.smachapp.model.IntActuator;

/**
 * Created by Morten Mey on 18.05.2014.
 */
public class IntSlider extends LinearLayout implements SeekBar.OnSeekBarChangeListener,ActuatorUI {

    private IntActuator actuator;
    private CheckBox check;
    private SeekBar slider;
    private TextView disp;


    public IntSlider(Context context,IntActuator actuator) {
        super(context);
        this.actuator=actuator;

        LinearLayout.LayoutParams param=new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT,LinearLayout.LayoutParams.WRAP_CONTENT);
        this.setLayoutParams(param);


        check=new CheckBox(context);
        check.setText(actuator.getKey());
        disp=new TextView(context);
        param=new LinearLayout.LayoutParams(60,LinearLayout.LayoutParams.WRAP_CONTENT);
        disp.setLayoutParams(param);

        slider=new SeekBar(context);
        slider.setMax(actuator.getMax()-actuator.getMin());
        slider.setOnSeekBarChangeListener(this);
        param=new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT);
        param.weight=1;
        slider.setLayoutParams(param);


        if(0<actuator.getMax()&&0>actuator.getMin()){
            slider.setProgress(-actuator.getMin());
        }else{
            slider.setProgress((actuator.getMax()-actuator.getMin())/2);
        }

        this.addView(check);
        this.addView(slider);
        this.addView(disp);
    }


    @Override
    public void setToAction(Action action){
        if(!action.getKey().equals(actuator.getKey())){
            throw new IllegalArgumentException("The action is not for this actuator.");
        }

        slider.setProgress(action.getValue()-actuator.getMin());
        check.setChecked(true);
    }

    @Override
    public Action createAction(){
        return new Action(actuator.getKey(),slider.getProgress()+actuator.getMin());
    }

    @Override
    public boolean isChecked(){
        return check.isChecked();
    }

    @Override
    public void onProgressChanged(SeekBar seekBar, final int i, boolean b) {
        disp.post(new Runnable() {
            @Override
            public void run() {
                disp.setText(Integer.toString(i+actuator.getMin()));
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
