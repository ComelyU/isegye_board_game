package com.accio.isegye.game.dto;

import com.accio.isegye.game.entity.Game;
import com.accio.isegye.game.entity.GameTagCategory;
import jakarta.validation.constraints.NotNull;
import java.util.ArrayList;
import java.util.List;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
public class StockResponse {

    @NotNull
    private int id;

    @NotNull
    private int isAvailable;

    @NotNull
    private String stockLocation;

    @NotNull
    private GameResponse game;

}
