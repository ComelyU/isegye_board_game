package com.accio.isegye.customer.dto;

import jakarta.validation.constraints.NotNull;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class CreateCustomerRequest {

    @NotNull
    private Integer roomId;

    @NotNull
    private int isTheme;

    private int peopleNum;
}
