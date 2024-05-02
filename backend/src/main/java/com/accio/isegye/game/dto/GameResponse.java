package com.accio.isegye.game.dto;

import com.accio.isegye.game.entity.GameTagCategory;
import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import java.util.ArrayList;
import java.util.List;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
public class GameResponse {
    @NotNull
    int id;

    @NotNull
    private String themeType;

    @NotBlank
    private String gameName;

    private String gameDetail;
    @NotNull
    private int minPlayer;
    @NotNull
    private int maxPlayer;
    @NotNull
    private int minPlaytime;
    @NotNull
    private int maxPlaytime;

    private float gameDifficulty;

    private String gameImgUrl;

    @NotNull
    private List<GameTagCategoryResponse> gameTagCategory= new ArrayList<>();
}
