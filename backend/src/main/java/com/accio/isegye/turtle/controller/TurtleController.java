package com.accio.isegye.turtle.controller;

import com.accio.isegye.menu.service.MenuService;
import com.accio.isegye.turtle.service.TurtleService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import java.util.List;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@Slf4j
@RestController
@RequiredArgsConstructor
@RequestMapping("/api/turtle")
@Tag(name = "Turtle", description = "Turtle API")
public class TurtleController {

    private final TurtleService turtleService;
    private final MenuService menuService;

    //터틀봇이 SEND할 수 있는 경로
    // "/pub/
    @PostMapping("/pub")
    public ResponseEntity<String> publishMessage(){

        return new ResponseEntity<>("Message", HttpStatus.OK);
    }

    //현재 대기 상태 중인 터틀 봇 리스트를 반환
    @GetMapping("/list")
    public ResponseEntity<List<Integer>> getTurtleList(){
        return new ResponseEntity<>(turtleService.getAvailableTurtleList(), HttpStatus.OK);
    }

    /*
     * 메뉴 주문 갱신
     * */
    @Operation(
        summary = "로봇 호출",
        description = "storeId에 현재 사용 가능한 로봇을 호출"
    )
    @GetMapping("order/{storeId}")
    public ResponseEntity<Integer> findAvailableTurtle(@PathVariable long orderMenuId){

        //1. 사용 가능한 로봇 호출
        List<Integer> turtleList = turtleService.getAvailableTurtleList();
        //1.1 사용 가능한 로봇이 없는 경우 별도의 메시지를 돌려보낸다.
        if(turtleList.isEmpty()){
            return new ResponseEntity<>(0, HttpStatus.OK);
        }
        //1.2 있는 경우 터틀봇 로그, 메뉴 로그를 작성하고 주문 테이블 갱신한다
        int turtleLogId = turtleService.createMenuLog(turtleList.get(0), orderMenuId);
        menuService.turtleOrderMenu(orderMenuId);
        //1.3 로봇에게 카운터의 주소 및 행동로그 id를 보낸다
        /*
         *
         * 로봇에게 메시지 보내는 것을 넣을 것
         * coordinateX, coordinateY, turtleLogId
         *
         * 로봇에게서 반환 메시지를 받으면 로그 업데이트 고려
         *
         * */

        //2. 끝나면 터틀봇 id를 반환한다
        return new ResponseEntity<>(turtleList.get(0), HttpStatus.OK);
    }



}
