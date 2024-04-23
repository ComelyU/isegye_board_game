package com.accio.isegye.store.service;

import com.accio.isegye.store.dto.CreateStoreRequest;
import com.accio.isegye.store.dto.StoreResponse;
import com.accio.isegye.store.dto.UpdateStoreRequest;
import jakarta.transaction.Transactional;

public interface StoreService {

    // 새로운 매장 생성
    @Transactional
    public StoreResponse createStore(CreateStoreRequest storeRequest);

    // 가게 매장 갱신
    @Transactional
    StoreResponse updateStore(UpdateStoreRequest storeRequest);

    // 매장 정보 확인
    StoreResponse getStore(int id);

    // 매장 삭제
    void deleteStore(int id);
}
