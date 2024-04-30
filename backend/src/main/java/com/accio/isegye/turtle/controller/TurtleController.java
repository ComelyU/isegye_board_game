package com.accio.isegye.turtle.controller;

import com.accio.isegye.turtle.entity.Turtle;
import com.accio.isegye.turtle.service.TurtleService;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
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
    //websocketconfig에서 설정한 setApplicationDestinationPrefixes와 @MessageMapping 경로가 병합됨
    // "/pub/
    @PostMapping("/pub")
    public ResponseEntity<String> publishMessage(){

        return new ResponseEntity<>("Message", HttpStatus.OK);
    }



}
