package com.accio.isegye.turtle.dto;

import jakarta.validation.constraints.NotNull;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class TurtleOrderRequest {
    @NotNull
    private Integer turtleId;

    private Long turtleOrderLogId = -1L;

    private Long turtleReceiveLogId = -1L;

    @Override
    public String toString() {
        return "{" +
            "\"turtleId\" : " + turtleId +
            ", \"turtleOrderLogId\" : " + turtleOrderLogId +
            ", \"turtleReceiveLogId\" : " + turtleReceiveLogId +
            '}';
    }
}
