package com.accio.isegye.game.dto;

import com.accio.isegye.game.entity.GameTagCategory;

public record GameTagCategoryResponse(
    Integer id, String groupName, String codeItemName
) {
    public GameTagCategoryResponse(GameTagCategory gameTagCategory) {
        this(gameTagCategory.getId(), gameTagCategory.getCodeGroup().getGroupName(), gameTagCategory.getCodeItem().getItemName());
    }
}