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
package de.uni_luebeck.iti.smachapp.model.undo;

import java.util.LinkedList;
import java.util.List;
import java.util.Stack;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class UndoManager {

    private List<UndoListener> listeners = new LinkedList<UndoListener>();

    private Stack<UndoableAction> undos = new Stack<UndoableAction>();
    private Stack<UndoableAction> redos = new Stack<UndoableAction>();

    public boolean addListener(UndoListener list) {
        return listeners.add(list);
    }

    public boolean removeListener(UndoListener list) {
        return listeners.remove(list);
    }

    protected void triggerListeners() {
        for (UndoListener list : listeners) {
            list.operationPerformed(this);
        }
    }

    public boolean isUndoAvailable() {
        return !undos.empty();
    }

    public boolean isRedoAvailable() {
        return !redos.empty();
    }

    public void newAction(UndoableAction action) {
        undos.push(action);
        redos.clear();
        triggerListeners();
    }

    public void undo() {
        if (isUndoAvailable()) {
            UndoableAction action = undos.pop();
            action.undo();
            redos.push(action);
            triggerListeners();
        }
    }

    public void redo() {
        if (isRedoAvailable()) {
            UndoableAction action = redos.pop();
            action.redo();
            undos.push(action);
            triggerListeners();
        }
    }

}