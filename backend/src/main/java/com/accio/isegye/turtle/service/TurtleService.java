package com.accio.isegye.turtle.service;

import com.accio.isegye.turtle.dto.StartTurtleOrderRequest;
import com.accio.isegye.turtle.dto.TurtleIdResponse;
import com.accio.isegye.turtle.dto.UpdateTurtleRequest;
import java.util.List;

public interface TurtleService {

    Integer createTurtle(int storeId);

    void updateTurtle(int turtleId, UpdateTurtleRequest request);

    void deleteTurtle(int turtleId);

    List<TurtleIdResponse> getAvailableTurtleList(int storeId);

    Long createTurtleLog(int turtleId, Long orderMenuId, Long orderGameId, Long receiveGameId, int commandType);

    void sendTurtleToCounter(int turtleId, Long turtleLogId);

    void sendOrderToTurtle(int turtleId, Long orderMenuId, Long orderGameId, Long turtleLogId);

    void startOrder(StartTurtleOrderRequest request);

    void receiveOrder(String message);
}
