package com.accio.isegye.store.controller;

import com.accio.isegye.store.dto.CreateStoreRequest;
import com.accio.isegye.store.dto.StoreResponse;
import com.accio.isegye.store.dto.UpdateStoreRequest;
import com.accio.isegye.store.service.StoreService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PatchMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/store")
@Tag(name = "Store", description = "Store API")
public class StoreController {

    private final StoreService service;

    @Operation(
        summary = "새로운 매장 생성",
        description = "매장명, 시간당 요금 등록"
    )
    @PostMapping
    public ResponseEntity<StoreResponse> createStore(@Valid @RequestBody CreateStoreRequest storeRequest){
        return new ResponseEntity<>(service.createStore(storeRequest), HttpStatus.CREATED);
    }

    @Operation(
        summary = "매장 정보 변경",
        description = "{id} 값에 해당되는 매장의 이름, 시간당 요금 변경"
    )
    @PatchMapping
    public ResponseEntity<StoreResponse> updateStore(@Valid @RequestBody UpdateStoreRequest storeRequest){
        return new ResponseEntity<>(service.updateStore(storeRequest), HttpStatus.OK);
    }

    @Operation(
        summary = "매장 정보 조회",
        description = "{id} 값에 해당되는 매장의 이름, 시간당 요금 조회"
    )
    @GetMapping("/{id}")
    public ResponseEntity<StoreResponse> getStore(@Valid @RequestParam int id){
        return new ResponseEntity<>(service.getStore(id), HttpStatus.OK);
    }

    @Operation(
        summary = "매장 정보 삭제",
        description = "{id} 값에 해당되는 매장의 deleted at 정보 갱신"
    )
    @PatchMapping("/{id}")
    public ResponseEntity<Void> deleteStore(@Valid @RequestParam int id){
        service.deleteStore(id);
        return new ResponseEntity<>(HttpStatus.NO_CONTENT);
    }
}
