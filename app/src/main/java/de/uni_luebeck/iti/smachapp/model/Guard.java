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
package de.uni_luebeck.iti.smachapp.model;

import java.util.LinkedList;
import java.util.List;

import de.uni_luebeck.iti.smachGenerator.ISmachableGuard;

/**
 * Created by Morten Mey on 21.04.2014.
 */
public class Guard implements ISmachableGuard {

    private LinkedList<String> names = new LinkedList<String>();
    private LinkedList<String> operators = new LinkedList<String>();
    private LinkedList<Integer> values = new LinkedList<Integer>();


    @Override
    public List<String> getSensorNames() {
        return names;
    }

    @Override
    public List<String> getOperators() {
        return operators;
    }

    @Override
    public List<Integer> getCompValues() {
        return values;
    }

    public void add(String name, String op, int value) {
        names.add(name);
        operators.add(op);
        values.add(value);
    }

    public void remove(int i) {
        names.remove(i);
        operators.remove(i);
        values.remove(i);
    }

    public void updateName(int i, String name) {
        names.set(i, name);
    }

    public void updateOperator(int i, String op) {
        operators.set(i, op);
    }

    public void updateValue(int i, int value) {
        values.set(i, value);
    }

    public void updateAll(int i, String name, String op, int value) {
        updateName(i, name);
        updateOperator(i, op);
        updateValue(i, value);
    }

    public void clear() {
        names.clear();
        operators.clear();
        values.clear();
    }
}
