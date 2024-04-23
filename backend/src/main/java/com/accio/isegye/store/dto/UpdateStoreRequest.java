package com.accio.isegye.store.dto;

import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class UpdateStoreRequest {

    @NotNull(message = "id는 필수")
    private int id;
    @NotBlank(message = "매장 이름 필수")
    private String storeName;

    private int hourFee;
}
