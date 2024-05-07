package com.accio.isegye.customer.controller;

import com.accio.isegye.customer.dto.CreateCustomerRequest;
import com.accio.isegye.customer.dto.CustomerResponse;
import com.accio.isegye.customer.service.CustomerService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import jakarta.validation.Valid;
import lombok.RequiredArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.Mapping;
import org.springframework.web.bind.annotation.PatchMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/customer")
@Tag(name = "Customer", description = "Customer API")
public class CustomerController {

    private final CustomerService customerService;

    @Operation(
        summary = "고객 시작",
        description = "고객 테이블 생성"
    )
    @PostMapping("/{roomId}")
    public ResponseEntity<CustomerResponse> createCustomer(@PathVariable int roomId, @Valid @RequestBody CreateCustomerRequest request){
        return new ResponseEntity<>(customerService.createCustomer(roomId, request), HttpStatus.OK);
    }

    @Operation(
        summary = "고객 끝",
        description = "고객의 사용시간 끝, 사용 요금을 반환한다, 보드게임 반환 요청을 보낸다"
    )
    @PatchMapping("/{customerId}")
    public ResponseEntity<Integer> endCustomer(@PathVariable int customerId){

        //터틀봇에게 반환 요청을 보낸다!
        int roomFee = customerService.endCustomer(customerId);
        return new ResponseEntity<>(roomFee, HttpStatus.OK);
    }

    @Operation(
        summary = "테마 on/off 토글",
        description = "고객의 방의 테마를 on/off를 변경한다"
    )
    @PatchMapping("/{customerId}/theme")
    public ResponseEntity<Integer> toggleTheme(@PathVariable int customerId){
        int theme = customerService.toggleTheme(customerId);
        return new ResponseEntity<>(theme, HttpStatus.OK);
    }

    /*
    * 테마 음량 조절 기능
    * */

}
