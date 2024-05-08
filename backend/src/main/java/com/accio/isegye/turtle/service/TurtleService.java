package com.accio.isegye.turtle.service;

import com.accio.isegye.turtle.dto.UpdateTurtleRequest;
import java.util.List;

public interface TurtleService {

    Integer createTurtle(int storeId);

    void updateTurtle(int turtleId, UpdateTurtleRequest request);

    void deleteTurtle(int turtleId);

    List<Integer> getAvailableTurtleList(int storeId);

    Long createTurtleLog(int turtleId, Long orderMenuId, Long orderGameId, int commandType);

    void sendOrderToTurtle(int turtleId, Long orderMenuId, Long orderGameId, Long turtleLogId);
}
