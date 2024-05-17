package com.accio.isegye.game.dto;

import com.accio.isegye.game.entity.Theme;
import java.time.LocalDateTime;

public record ThemeResponse(Integer id, String themeType, String themeVideoUrl, LocalDateTime createdAt, LocalDateTime updatedAt, LocalDateTime deletedAt) {

    public ThemeResponse(Theme theme) {
        this(theme.getId(), theme.getThemeType(), theme.getThemeVideoUrl(), theme.getCreatedAt(), theme.getUpdatedAt(), theme.getDeletedAt());
    }
}
