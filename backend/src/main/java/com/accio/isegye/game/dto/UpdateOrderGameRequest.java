package com.accio.isegye.game.dto;

import jakarta.validation.constraints.NotNull;
import java.time.LocalDateTime;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@RequiredArgsConstructor
public class UpdateOrderGameRequest {

    @NotNull
    private Integer orderStatus;
}
