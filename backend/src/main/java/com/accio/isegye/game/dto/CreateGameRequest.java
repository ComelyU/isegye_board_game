package com.accio.isegye.game.dto;

import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotEmpty;
import jakarta.validation.constraints.NotNull;
import java.util.List;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@RequiredArgsConstructor
public class CreateGameRequest {

    @NotBlank(message = "게임명 필수")
    private String gameName;

    @NotBlank(message = "게임 설명 필수")
    private String gameDetail;

    @NotNull
    private Integer minPlayer;

    @NotNull
    private Integer maxPlayer;

    @NotNull
    private Integer minPlayTime;

    @NotNull
    private Integer maxPlayTime;

    @NotNull
    private Float gameDifficulty;

    private Integer themeId;

    private List<Integer> tagCategoryIdList;
}
