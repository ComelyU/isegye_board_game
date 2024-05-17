package com.accio.isegye.game.dto;

import jakarta.validation.constraints.NotNull;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@RequiredArgsConstructor
public class CreateOrderGameRequest {

    // 0: 주문, 1: 회수
    @NotNull
    private Integer orderType;

}
