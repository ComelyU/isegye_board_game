package com.accio.isegye.turtle.controller;

import com.accio.isegye.game.service.GameService;
import com.accio.isegye.menu.service.MenuService;
import com.accio.isegye.turtle.dto.CreateOrderTurtleRequest;
import com.accio.isegye.turtle.dto.UpdateTurtleRequest;
import com.accio.isegye.turtle.service.TurtleService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import jakarta.validation.Valid;
import java.util.List;
import lombok.RequiredArgsConstructor;
import lombok.Value;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.Mapping;
import org.springframework.web.bind.annotation.PatchMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

@Slf4j
@RestController
@RequiredArgsConstructor
@RequestMapping("/api/turtle")
@Tag(name = "Turtle", description = "Turtle API")
public class TurtleController {

    private final TurtleService turtleService;
    private final MenuService menuService;
    private final GameService gameService;

    @Operation(
        summary = "로봇 등록",
        description = "storeId 매장에 새로운 로봇을 등록한다"
    )
    @PostMapping("/{storeId}")
    public ResponseEntity<Integer> createTurtle(@PathVariable int storeId){
        int turtleId = turtleService.createTurtle(storeId);
        return new ResponseEntity<>(turtleId, HttpStatus.CREATED);
    }

    @Operation(
        summary = "로봇 상태 변경",
        description = "turtleId 로봇의 상태를 변경한다"
    )
    @PatchMapping("/{turtleId}")
    public ResponseEntity<Void> updateTurtle(@PathVariable int turtleId,
        @Valid @RequestBody UpdateTurtleRequest request){

        turtleService.updateTurtle(turtleId, request);
        return new ResponseEntity<>(HttpStatus.NO_CONTENT);
    }

    @Operation(
        summary = "로봇 폐기 처분",
        description = "turtleId 로봇을 폐기 처분한다"
    )
    @DeleteMapping("/{turtleId}")
    public ResponseEntity<Void> deleteTurtle(@PathVariable int turtleId){
        turtleService.deleteTurtle(turtleId);
        return new ResponseEntity<>(HttpStatus.NO_CONTENT);
    }

    @Operation(
        summary = "로봇 검색",
        description = "storeId에 대기중인 로봇을 찾아낸다"
    )
    @GetMapping("/{storeId}/list")
    public ResponseEntity<List<Integer>> getTurtleList(@PathVariable int storeId){
        List<Integer> turtleList = turtleService.getAvailableTurtleList(storeId);
        return new ResponseEntity<>(turtleList, HttpStatus.OK);
    }

    @Operation(
        summary = "배송을 위한 로봇 호출",
        description = "turtleId에 해당되는 로봇을 호출한다"
    )
    @PostMapping("/order/{turtleId}")
    public ResponseEntity<Integer> orderTurtle(
        @PathVariable int turtleId, @Valid @RequestBody CreateOrderTurtleRequest request){

        //주문 정보가 들어있지 않은 경우
        if(request.getOrderMenuId() == null && request.getOrderGameId() == null && request.getReturnGameId() == null){
            return new ResponseEntity<>(-1, HttpStatus.BAD_REQUEST);
        }

        //1. 터틀봇 로그, 메뉴 로그를 작성하고 주문 테이블 갱신한다
        long turtleLogId = turtleService.createTurtleLog(turtleId,
            request.getOrderMenuId(),
            request.getOrderGameId(),
            request.getReturnGameId(),
            0);
        if(request.getOrderMenuId() != null) {
            menuService.updateOrderMenu(request.getOrderMenuId(), 2);
        }
        if(request.getOrderGameId() != null){
            //********************************게임 주문 테이블 갱신 추가 ***********************************
        }

        //2. 로봇에게 카운터의 주소 및 행동로그 id를 보낸다
        turtleService.sendOrderToTurtle(turtleId, request.getOrderMenuId(), request.getOrderGameId(), turtleLogId);

        //2. 끝나면 터틀봇 id를 반환한다
        return new ResponseEntity<>(turtleId, HttpStatus.OK);
    }

}
