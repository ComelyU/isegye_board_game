package com.accio.isegye.store.service;

import com.accio.isegye.exception.CustomException;
import com.accio.isegye.exception.ErrorCode;
import com.accio.isegye.store.dto.CreateStoreRequest;
import com.accio.isegye.store.dto.StoreResponse;
import com.accio.isegye.store.dto.UpdateStoreRequest;
import com.accio.isegye.store.entity.Store;
import com.accio.isegye.store.repository.RoomRepository;
import com.accio.isegye.store.repository.StoreRepository;
import jakarta.transaction.Transactional;
import java.time.LocalDateTime;
import java.util.NoSuchElementException;
import lombok.RequiredArgsConstructor;
import org.modelmapper.ModelMapper;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class StoreServiceImpl implements StoreService{

    private final StoreRepository storeRepository;
    private final RoomRepository roomRepository;
    private final ModelMapper modelMapper;

    // Store와 StoreResponse 매핑
    private StoreResponse getStoreResponse(Store store){
        return modelMapper.map(store, StoreResponse.class);
    }

    private Store findById(int id){
        return storeRepository.findById(id).orElseThrow(()
        -> new NoSuchElementException("No such Store"));
    }
    
    // 새로운 매장 생성
    @Override
    @Transactional
    public StoreResponse createStore(CreateStoreRequest storeRequest) {
        Store store = Store.builder()
            .storeName(storeRequest.getStoreName())
            .hourFee(storeRequest.getHourFee())
            .build();

        Store save = storeRepository.save(store);

        return getStoreResponse(save);
    }

    // 매장 정보 갱신
    @Override
    @Transactional
    public StoreResponse updateStore(UpdateStoreRequest storeRequest) {
        Store store = findById(storeRequest.getId());
        store.setStoreName(storeRequest.getStoreName());
        store.setHourFee(storeRequest.getHourFee());
        Store update = storeRepository.save(store);

        return getStoreResponse(update);
    }

    // 매장 정보 확인
    @Override
    public StoreResponse getStore(int id) {
        return storeRepository
            .findById(id)
            .map(this::getStoreResponse)
            .orElseThrow(() -> new CustomException(ErrorCode.BAD_REQUEST_ERROR));
    }

    // 매장 삭제
    @Override
    @Transactional
    public void deleteStore(int id) {
        Store store = findById(id);
        Store delete = Store.builder()
            .id(id)
            .storeName(store.getStoreName())
            .hourFee(store.getHourFee())
            .deletedAt(LocalDateTime.now())
            .build();

        storeRepository.save(delete);
    }


}
