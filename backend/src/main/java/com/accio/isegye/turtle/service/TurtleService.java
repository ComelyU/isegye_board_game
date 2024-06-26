package com.accio.isegye.turtle.service;

import com.accio.isegye.store.entity.Room;
import com.accio.isegye.turtle.dto.TurtleOrderRequest;
import com.accio.isegye.turtle.dto.TurtleIdResponse;
import com.accio.isegye.turtle.dto.TurtleOrderResponse;
import com.accio.isegye.turtle.dto.UpdateTurtleRequest;
import java.util.List;

public interface TurtleService {

    Integer createTurtle(int storeId);

    void updateTurtle(int turtleId, UpdateTurtleRequest request);

    void deleteTurtle(int turtleId);

    List<TurtleIdResponse> getAvailableTurtleList(int storeId);

    Long createTurtleLog(int turtleId, Long orderMenuId, Long orderGameId, int commandType);

    TurtleOrderResponse sendTurtleToCounter(int turtleId, Long turtleOrderLogId, Long turtleReceiveLogId);

    void sendOrderToTurtle(int turtleId, Room room, Long turtleOrderLogId, Long turtleReceiveOrderId);

    void updateTurtleOrder(String message);
}
