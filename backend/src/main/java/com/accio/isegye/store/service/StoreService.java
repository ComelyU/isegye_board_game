package com.accio.isegye.store.service;

import com.accio.isegye.store.dto.CreateRoomRequest;
import com.accio.isegye.store.dto.CreateStoreRequest;
import com.accio.isegye.store.dto.RoomResponse;
import com.accio.isegye.store.dto.StoreResponse;
import com.accio.isegye.store.dto.UpdateRoomRequest;
import com.accio.isegye.store.dto.UpdateStoreRequest;
import com.accio.isegye.store.entity.Room;
import java.util.List;
import org.springframework.transaction.annotation.Transactional;

public interface StoreService {

    // 새로운 매장 생성
    public StoreResponse createStore(CreateStoreRequest storeRequest);

    // 전체 매장 리스트 확인
    List<StoreResponse> getStoreList();
    
    // 매장 정보 확인
    StoreResponse getStore(int id);

    // 매장 정보 갱신
    StoreResponse updateStore(int id, UpdateStoreRequest storeRequest);

    // 매장 삭제
    void deleteStore(int id);

    //새로운 방 생성
    RoomResponse createRoom(CreateRoomRequest roomRequest);

    //방 정보 확인
    RoomResponse getRoom(int id);

    //방 ID 확인
    Integer getRoomId(int storeId, int roomNumber);

    //전체 방 정보 확인
    List<RoomResponse> getRoomList(int storeId);

    //방 정보 갱신
    RoomResponse updateRoom(int id, UpdateRoomRequest roomRequest);

    //fcm 토큰 확인
    String getFcmToken(int id);

    //fcm 토큰 갱신
    String updateFcmToken(int id, String token);

    //iotID 확인
    String getIotId(int id);

    //iotID 갱신
    String updateIotId(int id, String iotId);

    //방 삭제
    void deleteRoom(int id);

    //사용 가능한 방 확인
    //Customer 종료 시간 기준? 가장 마지막이 Null이면 현재 사용중
    List<RoomResponse> getAvailableRoomList(int storeId);

}
