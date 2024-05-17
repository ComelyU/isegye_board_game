package com.accio.isegye.common.service;

import com.accio.isegye.common.dto.CodeGroupListItemResponse;
import com.accio.isegye.common.dto.CodeGroupListResponse;
import com.accio.isegye.common.dto.CodeGroupResponse;
import com.accio.isegye.common.dto.CodeItemListResponse;
import com.accio.isegye.common.dto.CodeItemResponse;
import com.accio.isegye.common.dto.CreateCodeGroupRequest;
import com.accio.isegye.common.dto.CreateCodeItemListRequest;
import com.accio.isegye.common.dto.UpdateCodeGroupRequest;
import com.accio.isegye.common.dto.UpdateCodeItemRequest;
import com.accio.isegye.common.entity.CodeGroup;
import com.accio.isegye.common.entity.CodeItem;
import com.accio.isegye.common.repository.CodeGroupRepository;
import com.accio.isegye.common.repository.CodeItemRepository;
import com.accio.isegye.exception.CustomException;
import com.accio.isegye.exception.ErrorCode;
import java.util.ArrayList;
import java.util.List;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Service
@RequiredArgsConstructor
@Slf4j
public class CommonServiceImpl implements CommonService{

    private final CodeGroupRepository codeGroupRepository;
    private final CodeItemRepository codeItemRepository;

    /*
    공통 코드 테이블
    - 공통 코드 그룹
    - 공통 코드 아이템
     */

    // 공통 코드 그룹 생성
    @Override
    public CodeGroupResponse createCodeGroup(CreateCodeGroupRequest codeGroupReqeust) {
        CodeGroup codeGroup = codeGroupRepository.save(
            CodeGroup.builder()
                .groupName(codeGroupReqeust.getGroupName())
                .groupDescription(codeGroupReqeust.getGroupDescription())
                .build()
        );

        return new CodeGroupResponse(codeGroup);
    }

    // 공통 코드 그룹 리스트 조회
    @Override
    public CodeGroupListResponse getCodeGroupList() {
        return new CodeGroupListResponse(
            codeGroupRepository.findAll()
                .stream()
                .map(CodeGroupListItemResponse::new)
                .toList()
        );
    }

    // 공통 코드 그룹 상세 조회
    @Override
    public CodeGroupResponse getCodeGroup(String groupName) {
        CodeGroup codeGroup = codeGroupRepository.findById(groupName)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당 공통 코드 그룹을 찾을 수 없습니다"));

        return new CodeGroupResponse(codeGroup);
    }

    // 공통 코드 그룹 수정
    @Override
    @Transactional
    public Void updateCodeGroup(
        String groupName,
        UpdateCodeGroupRequest codeGroupReqeust
    ) {
        CodeGroup codeGroup = codeGroupRepository.findById(groupName)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당 공통 코드 그룹을 찾을 수 없습니다"));

        codeGroup.updateGroupDescription(codeGroupReqeust.getGroupDescription());

        return null;
    }

    // 공통 코드 그룹 삭제
    @Override
    @Transactional
    public Void deleteCodeGroup(String groupName) {
        CodeGroup codeGroup = codeGroupRepository.findById(groupName)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당 공통 코드 그룹을 찾을 수 없습니다"));

        codeGroupRepository.delete(codeGroup);

        return null;
    }

    // 공통 코드 아이템 생성
    @Override
    @Transactional
    public CodeItemListResponse createCodeItem(
        String groupName,
        CreateCodeItemListRequest codeItemListRequest
    ) {
        CodeGroup codeGroup = codeGroupRepository.findById(groupName)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당 공통 코드 그룹을 찾을 수 없습니다"));

        List<CodeItem> codeItemList = new ArrayList<>();
        codeItemListRequest.getItemList().forEach(item -> {
            CodeItem codeItem = codeItemRepository.save(
                CodeItem.builder()
                    .codeGroup(codeGroup)
                    .itemName(item.getItemName())
                    .itemDescription(item.getItemDescription())
                    .build()
            );

            codeItemList.add(codeItem);
        });

        return new CodeItemListResponse(
            codeItemList.stream()
                .map(CodeItemResponse::new)
                .toList()
        );
    }

    // 공통 코드 아이템 목록 전체 조회
    @Override
    public CodeItemListResponse getCodeItemList() {
        return new CodeItemListResponse(
            codeItemRepository.findAll()
                .stream()
                .map(CodeItemResponse::new)
                .toList()
        );
    }

    // 공통 코드 아이템 조회
    @Override
    public CodeItemResponse getCodeItem(Integer itemId) {
        CodeItem codeItem = codeItemRepository.findById(itemId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당 공통 코드 아이템을 찾을 수 없습니다"));

        return new CodeItemResponse(codeItem);
    }

    // 공통 코드 아이템 수정
    @Override
    @Transactional
    public Void updateCodeItem(Integer itemId, UpdateCodeItemRequest codeItemRequest) {
        CodeItem codeItem = codeItemRepository.findById(itemId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당 공통 코드 아이템을 찾을 수 없습니다"));

        codeItem.updateItemNameAndDescription(codeItemRequest.getItemName(), codeItemRequest.getItemDescription());

        return null;
    }

    // 공통 코드 아이템 삭제
    @Override
    @Transactional
    public Void deleteCodeItem(Integer itemId) {
        CodeItem codeItem = codeItemRepository.findById(itemId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당 공통 코드 아이템을 찾을 수 없습니다"));

        codeItemRepository.delete(codeItem);

        return null;
    }
}
