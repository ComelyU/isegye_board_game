package com.accio.isegye.store.dto;

import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
public class RoomResponse {
    @NotNull
    private int id;

    @NotNull
    private Integer storeId;

    @NotBlank
    private String coordinateX;

    @NotBlank
    private String coordinateY;

    @NotNull
    private Integer roomNumber;

    private String fcmToken;

    private String iotId;

    private Integer isUsed = 1;

}
