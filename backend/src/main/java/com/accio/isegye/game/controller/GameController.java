package com.accio.isegye.game.controller;

import com.accio.isegye.game.service.GameService;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/game")
@Tag(name = "Game", description = "Game API")
public class GameController {

    private final GameService gameService;
}
