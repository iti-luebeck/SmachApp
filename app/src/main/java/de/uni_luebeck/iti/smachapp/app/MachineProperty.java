package de.uni_luebeck.iti.smachapp.app;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.os.Bundle;
import android.widget.EditText;
import android.widget.Toast;

import de.uni_luebeck.iti.smachapp.model.EditorModel;
import de.uni_luebeck.iti.smachapp.model.XMLSaverLoader;

public class MachineProperty extends Activity {

    private static EditorModel setupModel;

    private EditorModel model;

    public static void setupModel(EditorModel m) {
        setupModel = m;
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_machine_property);

        model = setupModel;
        EditText text = (EditText) findViewById(R.id.automatName);
        text.setText(model.getStateMachine().getName());
        text.setSelection(model.getStateMachine().getName().length());
    }

    @Override
    public void onBackPressed() {
        String newName = ((EditText) findViewById(R.id.automatName)).getText().toString();

        if (!newName.isEmpty()) {
            updateMachineName(newName);
        } else {
            Toast toast=Toast.makeText(this,R.string.empty_name_in_property,Toast.LENGTH_LONG);
            toast.show();
            finish();
        }
    }

    private void updateMachineName(final String newName) {
        if (!newName.equals(model.getStateMachine().getName())) {
            if (XMLSaverLoader.doesFileExist(newName)) {
                AlertDialog.Builder builder = new AlertDialog.Builder(this);

                final Activity activity = this;

                builder.setTitle(R.string.file_already_exists);
                builder.setMessage(R.string.file_already_exists_message);
                builder.setPositiveButton(android.R.string.yes, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        setNameAndSave(newName);
                        activity.finish();
                    }
                });
                builder.setNegativeButton(android.R.string.cancel, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        activity.finish();
                    }
                });
                builder.show();

            } else {
                setNameAndSave(newName);
                finish();
            }
        } else {
            finish();
        }
    }

    private void setNameAndSave(String newName) {
        model.getStateMachine().setName(newName);
        model.updateFileNames();
        try {
            XMLSaverLoader.save(model);
        } catch (Exception ex) {
            ex.printStackTrace();
        }
        Toast.makeText(this,R.string.changes_saved,Toast.LENGTH_SHORT).show();
    }
}
