package com.accio.isegye.game.dto;

import jakarta.validation.constraints.NotBlank;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@RequiredArgsConstructor
public class CreateThemeRequest {

    @NotBlank(message = "테마 종류는 필수")
    private String themeType;
}
