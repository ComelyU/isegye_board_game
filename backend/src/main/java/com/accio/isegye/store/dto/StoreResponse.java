package com.accio.isegye.store.dto;

import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
public class StoreResponse {

    @NotNull
    private Integer id;

    @NotBlank
    private String storeName;

    private int hourFee;
    private String latitude;
    private String longitude;
    private String address;
    private String hours;
    private String phone;
}
