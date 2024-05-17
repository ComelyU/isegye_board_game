package com.accio.isegye.store.service;

import com.accio.isegye.customer.repository.CustomerRepository;
import com.accio.isegye.exception.CustomException;
import com.accio.isegye.exception.ErrorCode;
import com.accio.isegye.store.dto.CreateRoomRequest;
import com.accio.isegye.store.dto.CreateStoreRequest;
import com.accio.isegye.store.dto.RoomResponse;
import com.accio.isegye.store.dto.StoreResponse;
import com.accio.isegye.store.dto.UpdateRoomRequest;
import com.accio.isegye.store.dto.UpdateStoreRequest;
import com.accio.isegye.store.entity.Room;
import com.accio.isegye.store.entity.Store;
import com.accio.isegye.store.repository.RoomRepository;
import com.accio.isegye.store.repository.StoreRepository;
import java.util.List;
import java.util.stream.Collectors;
import lombok.extern.slf4j.Slf4j;
import org.springframework.transaction.annotation.Transactional;
import java.time.LocalDateTime;
import java.util.NoSuchElementException;
import lombok.RequiredArgsConstructor;
import org.modelmapper.ModelMapper;
import org.springframework.stereotype.Service;

@Slf4j
@Service
@RequiredArgsConstructor
public class StoreServiceImpl implements StoreService{

    private final StoreRepository storeRepository;
    private final RoomRepository roomRepository;
    private final CustomerRepository customerRepository;
    private final ModelMapper modelMapper;

    // Store와 StoreResponse 매핑
    private StoreResponse getStoreResponse(Store store){
        return modelMapper.map(store, StoreResponse.class);
    }

    // Room와 RoomResponse 매핑
    private RoomResponse getRoomResponse(Room room){
        return modelMapper.map(room, RoomResponse.class);
    }

    private Store findStoreById(int id){
        return storeRepository.findById(id).orElseThrow(()
        -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당 매장을 찾을 수 없습니다"));
    }

    private Room findRoomById(int id){
        return roomRepository.findById(id).orElseThrow(()
        -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당 방을 찾을 수 없습니다"));
    }
    
    // 새로운 매장 생성
    @Override
    @Transactional
    public StoreResponse createStore(CreateStoreRequest storeRequest) {
        Store store = Store.builder()
            .storeName(storeRequest.getStoreName())
            .hourFee(storeRequest.getHourFee())
            .latitude(storeRequest.getLatitude())
            .longitude(storeRequest.getLongitude())
            .address(storeRequest.getAddress())
            .hours(storeRequest.getHours())
            .phone(storeRequest.getPhone())
            .build();

        Store save = storeRepository.save(store);

        return getStoreResponse(save);
    }

    //현재 영업중인 매장 정보 리스트
    @Override
    @Transactional(readOnly = true)
    public List<StoreResponse> getStoreList() {
        return storeRepository.findAllByDeletedAtIsNull()
            .stream()
            .map(this::getStoreResponse)
            .toList();
    }

    // 매장 정보 확인
    @Override
    @Transactional(readOnly = true)
    public StoreResponse getStore(int id) {
        return storeRepository
            .findById(id)
            .map(this::getStoreResponse)
            .orElseThrow(() -> new CustomException(ErrorCode.BAD_REQUEST_ERROR));
    }

    // 매장 정보 갱신
    @Override
    @Transactional
    public StoreResponse updateStore(int id, UpdateStoreRequest storeRequest) {
        Store store = findStoreById(id);
        store.setStoreName(storeRequest.getStoreName());
        if(storeRequest.getHourFee() != null) store.setHourFee(storeRequest.getHourFee());
        Store update = storeRepository.save(store);

        return getStoreResponse(update);
    }

    // 매장 삭제
    @Override
    @Transactional
    public void deleteStore(int id) {
        Store store = findStoreById(id);
        Store delete = Store.builder()
            .id(id)
            .storeName(store.getStoreName())
            .hourFee(store.getHourFee())
            .deletedAt(LocalDateTime.now())
            .build();

        storeRepository.save(delete);
    }

    // 새로운 방 생성
    @Override
    public RoomResponse createRoom(CreateRoomRequest roomRequest) {
        Room room = Room.builder()
            .store(storeRepository.findById(roomRequest.getStoreId())
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당되는 매장을 찾을 수 없습니다")))
            .coordinateX(roomRequest.getCoordinateX())
            .coordinateY(roomRequest.getCoordinateY())
            .roomNumber(roomRequest.getRoomNumber())
            .fcmToken(roomRequest.getFcmToken())
            .iotId(roomRequest.getIotId())
            .build();

        Room save = roomRepository.save(room);

        return getRoomResponse(save);
    }

    @Override
    @Transactional(readOnly = true)
    public RoomResponse getRoom(int id) {

        return roomRepository.findById(id)
            .map(this::getRoomResponse)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당 방은 찾을 수 없습니다"));
    }

    @Override
    public Integer getRoomId(int storeId, int roomNumber) {
        return roomRepository.findIdByStoreIdAndRoomNumber(storeId, roomNumber);
    }

    @Override
    @Transactional(readOnly = true)
    public List<RoomResponse> getRoomList(int storeId) {
        //방에서 아직 끝나지 않은 Customer를 찾는다
        List<RoomResponse> roomList = roomRepository.findAllByDeletedAtIsNullAndStoreId(storeId)
            .stream()
            .map(this::getRoomResponse)
            .toList();

        roomList.stream()
            .filter(roomResponse ->
                customerRepository.existsByEndTimeIsNullAndRoomId(roomResponse.getId())
            )
            .findAny()
            .ifPresent(roomResponse -> roomResponse.setIsUsed(0));

        return roomList;
    }

    @Override
    @Transactional
    public RoomResponse updateRoom(int id, UpdateRoomRequest roomRequest) {
        Room room = findRoomById(id);
        room.setCoordinateX(roomRequest.getCoordinateX());
        room.setCoordinateY(roomRequest.getCoordinateY());
        room.setRoomNumber(roomRequest.getRoomNumber());
        if(roomRequest.getFcmToken() != null) room.setFcmToken(roomRequest.getFcmToken());
        if(roomRequest.getIotId() != null) room.setIotId(roomRequest.getIotId());

        Room update = roomRepository.save(room);

        return getRoomResponse(update);
    }

    @Override
    @Transactional(readOnly = true)
    public String getFcmToken(int id) {
        String token = roomRepository.findFcmTokenById(id);
        if(token == null){
            throw new CustomException(ErrorCode.NOT_FOUND_ERROR, "firbase token이 존재하지 않습니다");
        }

        return token;
    }

    @Override
    @Transactional
    public String updateFcmToken(int id, String token) {
        Room room = findRoomById(id);
        room.setFcmToken(token);

        Room update = roomRepository.save(room);
        return update.getFcmToken();
    }

    @Override
    @Transactional(readOnly = true)
    public String getIotId(int id) {
        String iotId = roomRepository.findIotIdById(id);
        if (iotId == null){
            throw new CustomException(ErrorCode.NOT_FOUND_ERROR, "iot Id가 존재하지 않습니다");
        }
        return roomRepository.findIotIdById(id);
    }

    @Override
    @Transactional
    public String updateIotId(int id, String iotId) {
        Room room = findRoomById(id);
        room.setIotId(iotId);

        Room update = roomRepository.save(room);
        return update.getIotId();
    }

    @Override
    @Transactional
    public void deleteRoom(int id) {
        Room room = findRoomById(id);
        Room delete = Room.builder()
            .id(id)
            .store(room.getStore())
            .coordinateX(room.getCoordinateX())
            .coordinateY(room.getCoordinateY())
            .roomNumber(room.getRoomNumber())
            .fcmToken(room.getFcmToken())
            .iotId(room.getIotId())
            .deletedAt(LocalDateTime.now())
            .build();

        roomRepository.save(delete);
    }

}
