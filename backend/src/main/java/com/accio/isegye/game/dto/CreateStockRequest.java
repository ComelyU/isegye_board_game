package com.accio.isegye.game.dto;

import jakarta.validation.constraints.NotBlank;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@RequiredArgsConstructor
public class CreateStockRequest {

    @NotBlank(message = "재고 보유 위치 필수")
    private String stockLocation;

}
