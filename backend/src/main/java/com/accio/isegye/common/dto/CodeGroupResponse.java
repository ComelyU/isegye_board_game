package com.accio.isegye.common.dto;

import com.accio.isegye.common.entity.CodeGroup;
import com.accio.isegye.common.entity.CodeItem;
import java.util.List;

public record CodeGroupResponse(String groupName, String groupDescription, List<CodeItemResponse> itemList) {

    public CodeGroupResponse(CodeGroup codeGroup) {
        this(codeGroup.getGroupName(), codeGroup.getGroupDescription(), codeGroup.getCodeItemList().stream().map(CodeItemResponse::new).toList());
    }
}
