package com.accio.isegye.turtle.service;

import java.util.List;

public interface TurtleService {

    List<Integer>  getAvailableTurtleList();

    Integer createMenuLog(int turtleId, long orderMenuId);
}
