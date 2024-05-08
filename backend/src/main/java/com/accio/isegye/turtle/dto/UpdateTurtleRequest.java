package com.accio.isegye.turtle.dto;

import jakarta.validation.constraints.NotNull;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class UpdateTurtleRequest {

    private Integer storeId;

    private Integer isWorking;
}
