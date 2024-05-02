package com.accio.isegye.game.dto;

import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
public class ThemeResponse {

    @NotNull
    private int id;

    @NotBlank
    private String themeType;

    private String themeVideoUrl;

}
