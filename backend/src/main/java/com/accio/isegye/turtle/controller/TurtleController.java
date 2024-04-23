package com.accio.isegye.turtle.controller;

import com.accio.isegye.turtle.entity.Turtle;
import com.accio.isegye.turtle.service.TurtleService;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/turtle")
@Tag(name = "Turtle", description = "Turtle API")
public class TurtleController {

    private TurtleService turtleService;
}
