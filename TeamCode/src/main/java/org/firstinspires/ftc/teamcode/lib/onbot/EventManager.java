package org.firstinspires.ftc.teamcode.lib.onbot;

import java.util.ArrayList;

public class EventManager {
    private ArrayList<EventHandler> eventHandlers;

    public EventManager() {
        this.eventHandlers = new ArrayList<EventHandler>();
    }

    public static interface EventHandler {
        public boolean execute();
    }

    public void subscribe(EventHandler eventHandler) {
        this.eventHandlers.add(eventHandler);
    }

    public void execute() {
        for (int i = eventHandlers.size() - 1; i >= 0; i--) {
            EventHandler eventHandler = this.eventHandlers.get(i);
            if (eventHandler.execute()) this.eventHandlers.remove(i);
        }
    }
}
