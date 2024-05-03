package com.accio.isegye.menu.dto;

import jakarta.validation.constraints.NotNull;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class CreateOrderMenuRequest {
    @NotNull
    private int menuId; // 메뉴 id

    @NotNull
    private int quantity; // 주문 수량
    
}
