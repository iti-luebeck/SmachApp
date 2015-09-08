/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of Lübeck
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
