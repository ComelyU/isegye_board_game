package com.accio.isegye.store.controller;

import com.accio.isegye.store.service.StoreService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/store")
@Tag(name = "Store", description = "Store API")
public class StoreController {

    private final StoreService storeService;

    @Operation(
        summary = "새로운 매장 생성",
        description = "매장명, 시간당 요금 등록"
    )
    @PostMapping("/register")
    public ResponseEntity<Void> createStore(){
        return new ResponseEntity<>(HttpStatus.CREATED);
    }
}
