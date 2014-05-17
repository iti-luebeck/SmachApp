package de.uni_luebeck.iti.smachapp.app;

import android.app.AlertDialog;
import android.content.DialogInterface;
import android.graphics.drawable.ColorDrawable;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;

import com.larswerkman.holocolorpicker.ColorPicker;
import com.larswerkman.holocolorpicker.OpacityBar;
import com.larswerkman.holocolorpicker.SaturationBar;
import com.larswerkman.holocolorpicker.ValueBar;

import java.util.List;

import de.uni_luebeck.iti.smachapp.model.Action;
import de.uni_luebeck.iti.smachapp.model.State;


public class StateProperty extends ActionBarActivity implements SeekBar.OnSeekBarChangeListener{

    private State state;

    private static State setupState;

    public static void setupState(State s){
        setupState=s;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_state_property);

        ((SeekBar)findViewById(R.id.motorLValue)).setOnSeekBarChangeListener(this);
        ((SeekBar)findViewById(R.id.motorRValue)).setOnSeekBarChangeListener(this);

        state=setupState;

        ((EditText)findViewById(R.id.stateName)).setText(state.getName());

        for(Action a:state.getActions()){

            if(a.getKey().startsWith("MOTOR")){
                setupMotor(a);
            }else if(a.getKey().startsWith("LED")){
                setupLED(a);
            }

        }

    }

    private void setupMotor(Action a) {
        if(a.getKey().endsWith("R")){
            ((SeekBar)findViewById(R.id.motorRValue)).setProgress(a.getValue()+128);
            ((CheckBox)findViewById(R.id.motorRCheck)).setChecked(true);
        }else if(a.getKey().endsWith("L")){
            ((SeekBar)findViewById(R.id.motorLValue)).setProgress(a.getValue()+128);
            ((CheckBox)findViewById(R.id.motorLCheck)).setChecked(true);
        }
    }

    private void setupLED(Action a){
        switch(a.getKey().charAt(a.getKey().length()-1)){
            case '0':
                ((CheckBox)findViewById(R.id.led0Check)).setChecked(true);
                ((ColorDrawable)findViewById(R.id.led0Button).getBackground()).setColor(a.getValue());
                break;

            case '1':
                ((CheckBox)findViewById(R.id.led1Check)).setChecked(true);
                ((ColorDrawable)findViewById(R.id.led1Button).getBackground()).setColor(a.getValue());
                break;

            case '2':
                ((CheckBox)findViewById(R.id.led2Check)).setChecked(true);
                ((ColorDrawable)findViewById(R.id.led2Button).getBackground()).setColor(a.getValue());
                break;

            case '3':
                ((CheckBox)findViewById(R.id.led3Check)).setChecked(true);
                ((ColorDrawable)findViewById(R.id.led3Button).getBackground()).setColor(a.getValue());
                break;

            case '4':
                ((CheckBox)findViewById(R.id.led4Check)).setChecked(true);
                ((ColorDrawable)findViewById(R.id.led4Button).getBackground()).setColor(a.getValue());
                break;

            case '5':
                ((CheckBox)findViewById(R.id.led5Check)).setChecked(true);
                ((ColorDrawable)findViewById(R.id.led5Button).getBackground()).setColor(a.getValue());
                break;

            case '6':
                ((CheckBox)findViewById(R.id.led6Check)).setChecked(true);
                ((ColorDrawable)findViewById(R.id.led6Button).getBackground()).setColor(a.getValue());
                break;

            case '7':
                ((CheckBox)findViewById(R.id.led7Check)).setChecked(true);
                ((ColorDrawable)findViewById(R.id.led7Button).getBackground()).setColor(a.getValue());
                break;

            case '8':
                ((CheckBox)findViewById(R.id.led8Check)).setChecked(true);
                ((ColorDrawable)findViewById(R.id.led8Button).getBackground()).setColor(a.getValue());
                break;
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

        if(isCheked(R.id.motorLCheck)){
            actions.add(new Action("MOTOR_L",((SeekBar)findViewById(R.id.motorLValue)).getProgress()-128));
        }
        if (isCheked(R.id.motorRCheck)){
            actions.add(new Action("MOTOR_R",((SeekBar)findViewById(R.id.motorRValue)).getProgress()-128));
        }
        if (isCheked(R.id.led0Check)){
            actions.add(new Action("LED0",((ColorDrawable)findViewById(R.id.led0Button).getBackground()).getColor()));
        }
        if (isCheked(R.id.led1Check)){
            actions.add(new Action("LED1",((ColorDrawable)findViewById(R.id.led1Button).getBackground()).getColor()));
        }
        if (isCheked(R.id.led2Check)){
            actions.add(new Action("LED2",((ColorDrawable)findViewById(R.id.led2Button).getBackground()).getColor()));
        }
        if (isCheked(R.id.led3Check)){
            actions.add(new Action("LED3",((ColorDrawable)findViewById(R.id.led3Button).getBackground()).getColor()));
        }
        if (isCheked(R.id.led4Check)){
            actions.add(new Action("LED4",((ColorDrawable)findViewById(R.id.led4Button).getBackground()).getColor()));
        }
        if (isCheked(R.id.led5Check)){
            actions.add(new Action("LED5",((ColorDrawable)findViewById(R.id.led5Button).getBackground()).getColor()));
        }
        if (isCheked(R.id.led6Check)){
            actions.add(new Action("LED6",((ColorDrawable)findViewById(R.id.led6Button).getBackground()).getColor()));
        }
        if (isCheked(R.id.led7Check)){
            actions.add(new Action("LED7",((ColorDrawable)findViewById(R.id.led7Button).getBackground()).getColor()));
        }
        if (isCheked(R.id.led8Check)){
            actions.add(new Action("LED8",((ColorDrawable)findViewById(R.id.led8Button).getBackground()).getColor()));
        }
    }

    private boolean isCheked(int id){
        return ((CheckBox)findViewById(id)).isChecked();
    }

    public void sliderClick(View view){
        switch(view.getId()){
            case R.id.motorLDisp:
                ((SeekBar)findViewById(R.id.motorLValue)).setProgress(128);
                break;

            case R.id.motorRDisp:
                ((SeekBar)findViewById(R.id.motorRValue)).setProgress(128);
                break;
        }
    }

    public void buttonPress(final View view){

        final ColorDrawable color=(ColorDrawable)view.getBackground();

        AlertDialog.Builder builder=new AlertDialog.Builder(this);
        builder.setTitle(R.string.colorDialogTitle);

        LayoutInflater factory = LayoutInflater.from(this);
        View myView = factory.inflate(R.layout.color_picker, null);
        final ColorPicker picker = (ColorPicker) myView.findViewById(R.id.picker);
        picker.addOpacityBar((OpacityBar)myView.findViewById(R.id.opacitybar));
        picker.addSaturationBar((SaturationBar) myView.findViewById(R.id.saturationbar));
        picker.addValueBar((ValueBar) myView.findViewById(R.id.valuebar));
        picker.setColor(color.getColor());
        builder.setView(myView);

        builder.setPositiveButton(R.string.acceptColor,new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                color.setColor(picker.getColor());
                view.postInvalidate();
            }
        });

        builder.setNegativeButton(R.string.disrecardColor,new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {

            }
        });

        builder.show();
    }

    @Override
    public void onProgressChanged(SeekBar seekBar, final int i, boolean b) {

        switch(seekBar.getId()){
            case R.id.motorLValue:
                final TextView view=(TextView)findViewById(R.id.motorLDisp);
                view.post(new Runnable() {
                    @Override
                    public void run() {
                        view.setText(Integer.toString(i-128));
                    }
                });
                break;

            case R.id.motorRValue:
                final TextView view2=(TextView)findViewById(R.id.motorRDisp);
                view2.post(new Runnable() {
                    @Override
                    public void run() {
                        view2.setText(Integer.toString(i-128));
                    }
                });
                break;
        }
    }

    @Override
    public void onStartTrackingTouch(SeekBar seekBar) {

    }

    @Override
    public void onStopTrackingTouch(SeekBar seekBar) {

    }
}
