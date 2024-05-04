package com.accio.isegye.game.dto;

import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@RequiredArgsConstructor
public class UpdateGameRequest {

    @NotBlank(message = "게임 설명 필수")
    private String gameDetail;

    @NotNull
    private Integer themeId;
}
