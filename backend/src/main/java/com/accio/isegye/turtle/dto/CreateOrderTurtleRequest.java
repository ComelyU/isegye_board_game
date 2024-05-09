package com.accio.isegye.turtle.dto;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class CreateOrderTurtleRequest {
    private Long orderMenuId;
    private Long orderGameId;
    private Long receiveGameId; // 반환 요청 id
}
