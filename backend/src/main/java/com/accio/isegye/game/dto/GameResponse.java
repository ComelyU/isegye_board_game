package com.accio.isegye.game.dto;

import com.accio.isegye.game.entity.Game;
import java.time.LocalDateTime;
import java.util.List;


public record GameResponse(
    Integer id,
    String gameName,
    String gameDetail,
    Integer minPlayer,
    Integer maxPlayer,
    Integer minPlaytime,
    Integer maxPlaytime,
    Float gameDifficulty,
    String gameImgUrl,
    String themeType,
    List<GameTagCategoryResponse> gameTagCategory,
    LocalDateTime createdAt,
    LocalDateTime updatedAt,
    LocalDateTime deletedAt
) {

    public GameResponse(Game game) {
        this(
            game.getId(),
            game.getGameName(),
            game.getGameDetail(),
            game.getMinPlayer(),
            game.getMaxPlayer(),
            game.getMinPlaytime(),
            game.getMaxPlaytime(),
            game.getGameDifficulty(),
            game.getGameImgUrl(),
            game.getTheme().getThemeType(),
            game.getGameTagCategoryList() != null ? game.getGameTagCategoryList()
                .stream().map(GameTagCategoryResponse::new).toList() : null,
            game.getCreatedAt(),
            game.getUpdatedAt(),
            game.getDeletedAt()
        );
    }
}
