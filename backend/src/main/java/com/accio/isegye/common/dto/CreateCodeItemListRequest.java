package com.accio.isegye.common.dto;

import jakarta.validation.constraints.NotEmpty;
import java.util.List;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@RequiredArgsConstructor
public class CreateCodeItemListRequest {

    @NotEmpty(message = "하나 이상의 아이템은 필수")
    private List<CreateCodeItemListItemRequest> itemList;
}
