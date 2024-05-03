package com.accio.isegye.menu.dto;

import jakarta.validation.constraints.NotNull;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
public class OrderMenuDetailResponse {

    @NotNull
    private long id;

    @NotNull
    private String menuName;

    @NotNull
    private int quantity;

    @NotNull
    private int totalPrice;

}
