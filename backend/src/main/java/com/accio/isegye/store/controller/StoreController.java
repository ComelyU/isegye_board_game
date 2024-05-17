package com.accio.isegye.store.controller;

import com.accio.isegye.store.dto.CreateRoomRequest;
import com.accio.isegye.store.dto.CreateStoreRequest;
import com.accio.isegye.store.dto.RoomResponse;
import com.accio.isegye.store.dto.StoreResponse;
import com.accio.isegye.store.dto.UpdateRoomRequest;
import com.accio.isegye.store.dto.UpdateStoreRequest;
import com.accio.isegye.store.service.StoreService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import jakarta.validation.Valid;
import jakarta.validation.constraints.NotBlank;
import java.util.List;
import lombok.RequiredArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PatchMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/stores")
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
        summary = "매장 정보 조회",
        description = "{id} 값에 해당되는 매장의 이름, 시간당 요금 조회"
    )
    @GetMapping("/{id}")
    public ResponseEntity<StoreResponse> getStore(@PathVariable int id){
        return new ResponseEntity<>(service.getStore(id), HttpStatus.OK);
    }

    @Operation(
        summary = "전체 매장 정보 조회",
        description = "전체 매장의 이름, 시간당 요금 조회"
    )
    @GetMapping
    public ResponseEntity<List<StoreResponse>> getStoreList(){
        return new ResponseEntity<>(service.getStoreList(), HttpStatus.OK);
    }

    @Operation(
        summary = "매장 정보 변경",
        description = "{id} 값에 해당되는 매장의 이름, 시간당 요금 변경"
    )
    @PatchMapping("/{id}")
    public ResponseEntity<StoreResponse> updateStore(@PathVariable int id, @Valid @RequestBody UpdateStoreRequest storeRequest){
        return new ResponseEntity<>(service.updateStore(id, storeRequest), HttpStatus.OK);
    }

    @Operation(
        summary = "매장 정보 삭제",
        description = "{id} 값에 해당되는 매장의 deleted at 정보 갱신"
    )
    @DeleteMapping("/{id}")
    public ResponseEntity<Void> deleteStore(@PathVariable int id){
        service.deleteStore(id);
        return new ResponseEntity<>(HttpStatus.NO_CONTENT);
    }

    @Operation(
        summary = "새로운 방 생성",
        description = "storeId에 해당되는 x좌표, y좌표, 방번호를 등록한다. fcm token, iot id는 선택이다"
    )
    @PostMapping("/rooms")
    public ResponseEntity<RoomResponse> createRoom(@Valid @RequestBody CreateRoomRequest roomRequest){
        return new ResponseEntity<>(service.createRoom(roomRequest), HttpStatus.CREATED);
    }

    @Operation(
        summary = "방 정보 확인",
        description = "방 id 필수"
    )
    @GetMapping("/rooms/{id}")
    public ResponseEntity<RoomResponse> getRoom(@PathVariable int id){
        return new ResponseEntity<>(service.getRoom(id), HttpStatus.OK);
    }

    @Operation(
        summary = "StoreId와 RoomNumber를 기준으로 방 찾기",
        description = "방 id 가져오기"
    )
    @GetMapping("/rooms/valid/{storeId}")
    public ResponseEntity<Integer> getRoomId(@PathVariable int storeId, @RequestParam int roomNumber){
        return new ResponseEntity<>(service.getRoomId(storeId, roomNumber), HttpStatus.OK);
    }

    @Operation(
        summary = "특정 매장의 모든 방 정보 확인",
        description = "매장 id 필수"
    )
    @GetMapping("/{id}/room-lists")
    public ResponseEntity<List<RoomResponse>> getRoomList(@PathVariable int id){
        List<RoomResponse> roomList = service.getRoomList(id);
        return new ResponseEntity<>(roomList, HttpStatus.OK);
    }

    @Operation(
        summary = "방 정보 수정",
        description = "x좌표, y좌표, 방 번호는 필수, 토큰과 iot id는 선택"
    )
    @PatchMapping("/rooms/{id}")
    public ResponseEntity<RoomResponse> updateRoom(@PathVariable int id, @Valid @RequestBody
        UpdateRoomRequest roomRequest){
        return new ResponseEntity<>(service.updateRoom(id, roomRequest), HttpStatus.OK);
    }
    
    @Operation(
        summary = "특정 방의 fcm token을 가져온다",
        description = "방 id는 필수"
    )
    @GetMapping("/rooms/{id}/token")
    public ResponseEntity<String> getRoomToken(@PathVariable int id){
        return new ResponseEntity<>(service.getFcmToken(id), HttpStatus.OK);
    }
    
    @Operation(
        summary = "특정 방의 fcm token의 값을 변경한다",
        description = "방 id, 변경할 fcm token 값은 필수"
    )
    @PatchMapping("/rooms/{id}/token")
    public ResponseEntity<String> updateRoomToken(@PathVariable int id, @RequestBody @NotBlank UpdateRoomRequest roomRequest){
    return new ResponseEntity<>(
        service.updateFcmToken(id, roomRequest.getFcmToken()), HttpStatus.OK);
    }

    @Operation(
        summary = "특정 방의 iot id를 가져온다",
        description = "방 id는 필수"
    )
    @GetMapping("/rooms/{id}/iot")
    public ResponseEntity<String> getRoomIotId(@PathVariable int id){
        return new ResponseEntity<>(service.getIotId(id), HttpStatus.OK);
    }

    @Operation(
        summary = "특정 방의 iot id 값을 변경한다",
        description = "방 id, 변경할 iot id 값은 필수"
    )
    @PatchMapping("/rooms/{id}/iot")
    public ResponseEntity<String> updateRoomIotId(@PathVariable int id, @RequestBody @NotBlank UpdateRoomRequest roomRequest){
        return new ResponseEntity<>(service.updateIotId(id, roomRequest.getIotId()), HttpStatus.OK);
    }

    @Operation(
        summary = "특정 방을 삭제한다",
        description = "{id}값에 해당되는 방의 deleted at 값을 입력"
    )
    @DeleteMapping("/rooms/{id}")
    public ResponseEntity<Void> deleteRoom(@PathVariable int id){
        service.deleteRoom(id);
        return new ResponseEntity<>(HttpStatus.NO_CONTENT);
    }

}
