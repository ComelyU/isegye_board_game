package com.accio.isegye.turtle.dto;

import jakarta.validation.constraints.NotNull;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;
import lombok.ToString;

@Getter
@Builder
@AllArgsConstructor
@NoArgsConstructor
@ToString
@Data
public class StartOrderDto {
    @NotNull
    private Long turtleLogId;
    @NotNull
    private String coordinateX;
    @NotNull
    private String coordinateY;
}
