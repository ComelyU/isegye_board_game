package com.accio.isegye.store.dto;

import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

@Getter
@RequiredArgsConstructor
public class CreateRoomRequest {

    @NotNull
    int storeId;

    @NotBlank
    String coordinateX;

    @NotBlank
    String coordinateY;

    @NotNull
    Integer roomNumber;

    String fcmToken;

    String iotId;
}
