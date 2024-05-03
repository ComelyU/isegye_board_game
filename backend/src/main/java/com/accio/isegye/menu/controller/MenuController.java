package com.accio.isegye.menu.controller;

import com.accio.isegye.menu.dto.CreateMenuRequest;
import com.accio.isegye.menu.dto.CreateOrderMenuRequest;
import com.accio.isegye.menu.dto.MenuResponse;
import com.accio.isegye.menu.dto.OrderMenuResponse;
import com.accio.isegye.menu.dto.UpdateMenuRequest;
import com.accio.isegye.menu.service.MenuService;
import com.accio.isegye.turtle.service.TurtleService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import jakarta.validation.Valid;
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
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/menu")
@Tag(name = "Menu", description = "Menu API")
public class MenuController {

    private final MenuService menuService;
    private final TurtleService turtleService;

    @Operation(
        summary = "해당 매장 메뉴 생성",
        description = "새로운 매뉴 생성"
    )
    @PostMapping("/{storeId}")
    public ResponseEntity<MenuResponse> createMenu(@PathVariable int storeId, @RequestBody CreateMenuRequest createMenuRequest){
        return new ResponseEntity<>(menuService.createMenu(storeId, createMenuRequest), HttpStatus.OK);
    }

    @Operation(
        summary = "해당 매장의 모든 메뉴 목록",
        description = "storeId 값에 해당되는 메뉴 목록, 폐기된 메뉴는 보이지 않는다"
    )
    @GetMapping("/{storeId}")
    public ResponseEntity<List<MenuResponse>> getMenuList(@PathVariable int storeId){
        return new ResponseEntity<>(menuService.getMenuList(storeId), HttpStatus.OK);
    }

    @Operation(
        summary = "해당 메뉴의 정보 변경",
        description = "menuId 값에 해당되는 메뉴 정보를 수정한다"
    )
    @PatchMapping("/{menuId}")
    public ResponseEntity<MenuResponse> updateMenu(@PathVariable int menuId, @RequestBody UpdateMenuRequest updateMenuRequest){
        return new ResponseEntity<>(menuService.updateMenu(menuId, updateMenuRequest), HttpStatus.OK);
    }

    @Operation(
        summary = "해당 메뉴 삭제",
        description = "menuId 값에 해당되는 메뉴를 폐기한다"
    )
    @DeleteMapping("/{menuId}")
    public ResponseEntity<Void> deleteMenu(@PathVariable int menuId){
        menuService.deleteMenu(menuId);
        return new ResponseEntity<>(HttpStatus.NO_CONTENT);
    }

    @Operation(
        summary = "메뉴 주문",
        description = "입력: 메뉴 id, 수량, 출력: 알바생(관리자)에게 해당 주문 내역을 전송"
    )
    @PostMapping("/order/{customerId}")
    public ResponseEntity<OrderMenuResponse> createOrderMenu(@PathVariable int customerId, @Valid @RequestBody List<CreateOrderMenuRequest> orderMenuRequest){

        OrderMenuResponse orderMenuResponse = menuService.createOrderMenu(orderMenuRequest, customerId);

        /*
        *
        * 카프카를 통해서 점주에게도 주문 내역을 보낸다.
        *
        * */

        return new ResponseEntity<>(orderMenuResponse, HttpStatus.OK);
    }

    @Operation(
        summary = "주문 내역 확인",
        description = "customerId에 해당하는 주문 내역 목록 반환"
    )
    @GetMapping("/order/{customerId}")
    public ResponseEntity<List<OrderMenuResponse>> getOrderMenu(@PathVariable int customerId){
        return new ResponseEntity<>(menuService.getOrderMenu(customerId), HttpStatus.OK);
    }

    @Operation(
        summary = "매장의 모든 주문 확인",
        description = "storeId에 해당하는 모든 주문 내역 확인"
    )
    @GetMapping("/order/store/{storeId}")
    public ResponseEntity<List<OrderMenuResponse>> getStoreOrderMenu(@PathVariable int storeId){
        return new ResponseEntity<>(menuService.getStoreOrderMenu(storeId), HttpStatus.OK);
    }

    @Operation(
        summary = "메뉴 준비 시작",
        description = "menuId에 해당하는 주문을 메뉴 준비 시작 상태로 변경한다"
    )
    @PatchMapping("order/{orderMenuId}")
    public ResponseEntity<Void> readyOrderMenu(@PathVariable long orderMenuId){
        menuService.readyOrderMenu(orderMenuId);
        return new ResponseEntity<>(HttpStatus.NO_CONTENT);
    }

    @Operation(
        summary = "주문완료",
        description = "menuId에 해당하는 주문을 완료한다"
    )
    @PatchMapping("order/{orderMenuId}/complete")
    public ResponseEntity<Void> completeOrderMenu(@PathVariable long orderMenuId){
        menuService.completeOrderMenu(orderMenuId);
        return new ResponseEntity<>(HttpStatus.NO_CONTENT);
    }


    /*
    * 메뉴 주문 갱신
    * */
    @Operation(
        summary = "로봇 호출",
        description = "storeId에 현재 사용 가능한 로봇을 호출"
    )
    @GetMapping("order/{storeId}/turtle")
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
