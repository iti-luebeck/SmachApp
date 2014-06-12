package de.uni_luebeck.iti.smachapp.model;

import java.util.LinkedList;
import java.util.List;

import de.uni_luebeck.iti.smachGenerator.ISmachableGuard;
import de.uni_luebeck.iti.smachGenerator.Operator;

/**
 * Created by Morten Mey on 21.04.2014.
 */
public class Guard implements ISmachableGuard {

    private LinkedList<String> names = new LinkedList<String>();
    private LinkedList<Operator> operators = new LinkedList<Operator>();
    private LinkedList<Integer> values = new LinkedList<Integer>();


    @Override
    public List<String> getSensorNames() {
        return names;
    }

    @Override
    public List<Operator> getOperators() {
        return operators;
    }

    @Override
    public List<Integer> getCompValues() {
        return values;
    }

    public void add(String name, Operator op, int value) {
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

    public void updateOperator(int i, Operator op) {
        operators.set(i, op);
    }

    public void updateValue(int i, int value) {
        values.set(i, value);
    }

    public void updateAll(int i, String name, Operator op, int value) {
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
