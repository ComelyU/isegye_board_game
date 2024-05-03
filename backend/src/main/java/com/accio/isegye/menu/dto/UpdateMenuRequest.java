package com.accio.isegye.menu.dto;

import jakarta.validation.constraints.NotBlank;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class UpdateMenuRequest {

    @NotBlank
    String menuName;

    Character menuType;

    Integer menuPrice;

    Integer isAvailable;

    String menuImgUrl;
}
