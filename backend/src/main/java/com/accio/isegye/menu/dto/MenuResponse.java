package com.accio.isegye.menu.dto;

import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
public class MenuResponse {
    @NotNull
    private int id;

    @NotBlank
    private String menuName;
    private char menuType;
    private int menuPrice;
    private int isAvailable;
    private String menuImgUrl;

}
