package com.accio.isegye.turtle.dto;

import jakarta.validation.constraints.NotNull;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class TurtleOrderResponse {
    @NotNull
    private Integer turtleId;

    private Long turtleOrderLogId;

    private Long turtleReceiveLogId;
}
