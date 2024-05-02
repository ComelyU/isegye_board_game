package com.accio.isegye.common.dto;

import jakarta.validation.constraints.NotBlank;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@RequiredArgsConstructor
public class UpdateCodeGroupRequest {

    @NotBlank(message = "공통 코드 그룹 설명 필수")
    private String groupDescription;

}
