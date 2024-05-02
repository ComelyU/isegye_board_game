package com.accio.isegye.turtle.controller;

import com.accio.isegye.turtle.service.TurtleService;
import io.swagger.v3.oas.annotations.tags.Tag;
import java.util.List;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
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



}
