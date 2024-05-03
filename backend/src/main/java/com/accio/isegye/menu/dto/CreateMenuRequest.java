package com.accio.isegye.menu.dto;

import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class CreateMenuRequest {
    @NotBlank
    private String menuName;

    private char menuType;
    private int menuPrice;
    private int isAvailable;
    private String menuImgUrl;
}
