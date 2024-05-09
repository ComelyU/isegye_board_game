package com.accio.isegye.turtle.dto;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class StartTurtleOrderRequest {
    private Integer turtleId;
    private Long turtleLogId;
}
