package com.accio.isegye.turtle.dto;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class CreateOrderTurtleRequest {
    private Long orderMenuId;
    private Long orderGameId;
}
