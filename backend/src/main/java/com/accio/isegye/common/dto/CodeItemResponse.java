package com.accio.isegye.common.dto;

import com.accio.isegye.common.entity.CodeItem;

public record CodeItemResponse(Integer id, String groupName, String groupDescription, String itemName, String itemDescription) {

    public CodeItemResponse(CodeItem item) {
        this(item.getId(), item.getCodeGroup().getGroupName(), item.getCodeGroup().getGroupDescription(), item.getItemName(), item.getItemDescription());
    }
}
