package com.accio.isegye.common.service;

import com.accio.isegye.common.dto.CodeGroupListResponse;
import com.accio.isegye.common.dto.CodeGroupResponse;
import com.accio.isegye.common.dto.CodeItemListResponse;
import com.accio.isegye.common.dto.CodeItemResponse;
import com.accio.isegye.common.dto.CreateCodeGroupRequest;
import com.accio.isegye.common.dto.CreateCodeItemListRequest;
import com.accio.isegye.common.dto.UpdateCodeGroupRequest;
import com.accio.isegye.common.dto.UpdateCodeItemRequest;

public interface CommonService {

    // 공통 코드 그룹 생성
    CodeGroupResponse createCodeGroup(CreateCodeGroupRequest codeGroupReqeust);

    // 공통 코드 그룹 리스트 조회
    CodeGroupListResponse getCodeGroupList();

    // 공통 코드 그룹 상세 조회
    CodeGroupResponse getCodeGroup(String groupName);

    // 공통 코드 그룹 수정
    Void updateCodeGroup(String groupName, UpdateCodeGroupRequest codeGroupReqeust);

    // 공통 코드 그룹 삭제
    Void deleteCodeGroup(String groupName);

    // 공통 코드 아이템 생성
    CodeItemListResponse createCodeItem(String groupName, CreateCodeItemListRequest codeItemListRequest);

    // 공통 코드 아이템 목록 조회
    CodeItemListResponse getCodeItemList();

    // 공통 코드 아이템 조회
    CodeItemResponse getCodeItem(Integer itemId);

    // 공통 코드 아이템 수정
    Void updateCodeItem(Integer itemId, UpdateCodeItemRequest codeItemRequest);

    // 공통 코드 아이템 삭제
    Void deleteCodeItem(Integer itemId);
}
