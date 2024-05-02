package com.accio.isegye.common.dto;

import com.accio.isegye.common.entity.CodeGroup;

public record CodeGroupListItemResponse(String groupName, String groupDescription) {

    public CodeGroupListItemResponse(CodeGroup codeGroup) {
        this(codeGroup.getGroupName(), codeGroup.getGroupDescription());
    }
}
