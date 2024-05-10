package com.accio.isegye.store.dto;

import jakarta.validation.constraints.NotBlank;
import lombok.Builder;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class CreateStoreRequest {

    @NotBlank
    private String storeName;
    private int hourFee;
    private String latitude;
    private String longitude;
    private String address;
    private String hours;
    private String phone;
}
