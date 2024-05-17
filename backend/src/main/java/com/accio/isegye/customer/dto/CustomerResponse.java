package com.accio.isegye.customer.dto;

import jakarta.validation.constraints.NotNull;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
public class CustomerResponse {

    @NotNull
    private int id;

    @NotNull
    private int roomId;

    @NotNull
    private int isTheme;

    private int peopleNum;
}
