package com.accio.isegye.game.dto;

import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@RequiredArgsConstructor
public class UpdateStockRequest {

    @NotNull
    private Integer isAvailable;

    @NotBlank(message = "재고 보유 위치 필수")
    private String stockLocation;

}
