package com.accio.isegye.common.dto;

import jakarta.validation.constraints.NotBlank;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@RequiredArgsConstructor
public class UpdateCodeItemRequest {

    @NotBlank(message = "공통 코드 아이템명 필수")
    private String itemName;

    @NotBlank(message = "공통 코드 아이템 설명 필수")
    private String itemDescription;
}
