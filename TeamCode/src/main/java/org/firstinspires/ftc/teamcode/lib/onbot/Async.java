package org.firstinspires.ftc.teamcode.lib.onbot;

import java.util.ArrayList;

public class Async {
    private ArrayList<Async> dependencies;
    private ArrayList<Async> dependents;
    private AsyncBody body = null;
    private int dependenciesFulfilled;
    private int dependenciesRequired = -1;

    public Async() {
        this.dependencies = new ArrayList<Async>();
        this.dependents = new ArrayList<Async>();
        this.body = async -> finish();
        this.standby();
    }

    public Async(AsyncBody body) {
        this.dependencies = new ArrayList<Async>();
        this.dependents = new ArrayList<Async>();
        this.body = body;
        this.standby();
    }

    public static interface AsyncBody {
        public void execute(Async async);
    }

    public static void addLink(Async a, Async b) {
        a.addDependent(b);
        b.addDependency(a);
    }

    public void setDependenciesRequired() {
        this.dependenciesRequired = -1;
    }

    public void setDependenciesRequired(int threshold) {
        this.dependenciesRequired = threshold;
    }

    public void addDependency(Async dependency) {
        this.dependencies.add(dependency);
    }

    public void addDependent(Async dependent) {
        this.dependents.add(dependent);
    }

    public int getDependenciesFulfilled() {
        return this.dependenciesFulfilled;
    }

    public int getDependenciesRequired() {
        return this.dependenciesRequired;
    }

    public void fulfill() {
        this.dependenciesFulfilled++;
        int dependenciesRequired = 1;
        if (this.dependenciesRequired == -1) dependenciesRequired = this.dependencies.size();
        if (this.dependenciesFulfilled >= dependenciesRequired) this.execute();
    }

    public void standby() {
        this.dependenciesFulfilled = 0;
    }

    public void execute() {
        if (this.body != null) this.body.execute(this);
    }

    public void finish() {
        this.standby();
        for (int i = 0; i < dependents.size(); i++) dependents.get(i).fulfill();
    }
}