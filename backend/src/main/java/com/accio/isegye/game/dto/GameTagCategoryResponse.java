package com.accio.isegye.game.dto;

import jakarta.validation.constraints.NotNull;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
public class GameTagCategoryResponse {

    @NotNull
    private int id;

    private String groupName;

    private String codeItemName;
}
