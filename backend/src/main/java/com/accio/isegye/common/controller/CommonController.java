package com.accio.isegye.common.controller;

import com.accio.isegye.common.dto.CodeGroupListResponse;
import com.accio.isegye.common.dto.CodeGroupResponse;
import com.accio.isegye.common.dto.CodeItemListResponse;
import com.accio.isegye.common.dto.CodeItemResponse;
import com.accio.isegye.common.dto.CreateCodeGroupRequest;
import com.accio.isegye.common.dto.CreateCodeItemListRequest;
import com.accio.isegye.common.dto.UpdateCodeGroupRequest;
import com.accio.isegye.common.dto.UpdateCodeItemRequest;
import com.accio.isegye.common.service.CommonService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PatchMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import static org.springframework.http.HttpStatus.*;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/common")
@Tag(name = "Common", description = "Common API")
public class CommonController {

    private final CommonService commonService;

    /*
    공통 코드 테이블
    - 공통 코드 그룹
    - 공통 코드 아이템
     */

    @Operation(
        summary = "공통 코드 그룹 생성",
        description = "코드 그룹명, 코드 그룹 설명 등록. 해당 그룹의 아이템은 아이템 생성을 이용."
    )
    @PostMapping("/code-group")
    public ResponseEntity<CodeGroupResponse> createCodeGroup(
        @Valid @RequestBody CreateCodeGroupRequest codeGroupReqeust
    ) {
        return new ResponseEntity<>(
            commonService.createCodeGroup(codeGroupReqeust),
            CREATED
        );
    }

    @Operation(
        summary = "공통 코드 그룹 리스트 조회",
        description = "공통 코드 그룹 리스트 조회. 그룹명과 그룹 설명만 조회."
    )
    @GetMapping("/code-group")
    public ResponseEntity<CodeGroupListResponse> getCodeGroupList() {
        return new ResponseEntity<>(
            commonService.getCodeGroupList(),
            OK
        );
    }

    @Operation(
        summary = "공통 코드 그룹 상세 조회",
        description = "{groupName} 값에 해당하는 공통 코드 그룹의 그룹명, 그룹 설명, 아이템 리스트를 조회"
    )
    @GetMapping("/code-group/{groupName}")
    public ResponseEntity<CodeGroupResponse> getCodeGroup(
        @PathVariable String groupName
    ) {
        return new ResponseEntity<>(
            commonService.getCodeGroup(groupName),
            OK
        );
    }

    @Operation(
        summary = "공통 코드 그룹 수정",
        description = "{groupName} 값에 해당하는 공통 코드 그룹의 그룹 설명 수정"
    )
    @PatchMapping("/code-group/{groupName}")
    public ResponseEntity<Void> updateCodeGroup(
        @PathVariable String groupName,
        @Valid @RequestBody UpdateCodeGroupRequest codeGroupReqeust
    ) {
        return new ResponseEntity<>(
            commonService.updateCodeGroup(groupName, codeGroupReqeust),
            NO_CONTENT
        );
    }

    @Operation(
        summary = "공통 코드 그룹 삭제",
        description = "{groupName} 값에 해당하는 공통 코드 그룹 삭제"
    )
    @DeleteMapping("/code-group/{groupName}")
    public ResponseEntity<Void> deleteCodeGroup(
        @PathVariable String groupName
    ) {
        return new ResponseEntity<>(
            commonService.deleteCodeGroup(groupName),
            NO_CONTENT
        );
    }

    @Operation(
        summary = "공통 코드 아아템 생성",
        description = "{groupName} 값에 해당하는 공통 코드 그룹 하위 아이템 리스트를 등록. 아이템명과 아이템 설명을 등록."
    )
    @PostMapping("/code-group/{groupName}/code-item")
    public ResponseEntity<CodeItemListResponse> createCodeItem(
        @PathVariable String groupName,
        @Valid @RequestBody CreateCodeItemListRequest codeItemListRequest
    ) {
        return new ResponseEntity<>(
            commonService.createCodeItem(groupName, codeItemListRequest),
            CREATED
        );
    }

    @Operation(
        summary = "전체 공통 코드 아이템 조회",
        description = "공통 코드 아이템 전체 목록에 대해서 조회"
    )
    @GetMapping("/code-item")
    public ResponseEntity<CodeItemListResponse> getCodeItemList() {
        return new ResponseEntity<>(
            commonService.getCodeItemList(),
            OK
        );
    }

    @Operation(
        summary = "공통 코드 아이템 조회",
        description = "{itemId} 값에 해당하는 공통 코드 아이템에 대해서 조회"
    )
    @GetMapping("/code-item/{itemId}")
    public ResponseEntity<CodeItemResponse> getCodeItem(
        @PathVariable Integer itemId
    ) {
        return new ResponseEntity<>(
            commonService.getCodeItem(itemId),
            OK
        );
    }

    @Operation(
        summary = "공통 코드 아이템 수정",
        description = "{itemId} 값에 해당하는 공통 코드 아이템의 아이템명과 아이템 설명 수정"
    )
    @PatchMapping("/code-item/{itemId}")
    public ResponseEntity<Void> updateCodeItem(
        @PathVariable Integer itemId,
        @Valid @RequestBody UpdateCodeItemRequest codeItemRequest
    ) {
        return new ResponseEntity<>(
            commonService.updateCodeItem(itemId, codeItemRequest),
            NO_CONTENT
        );
    }

    @Operation(
        summary = "공통 코드 아이템 삭제",
        description = "{itemId} 값에 해당하는 공통 코드 아이템 삭제"
    )
    @DeleteMapping("/code-item/{itemId}")
    public ResponseEntity<Void> deleteCodeItem(
        @PathVariable Integer itemId
    ) {
        return new ResponseEntity<>(
            commonService.deleteCodeItem(itemId),
            NO_CONTENT
        );
    }
}
