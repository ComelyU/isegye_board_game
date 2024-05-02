package com.accio.isegye.game.controller;

import com.accio.isegye.game.dto.GameResponse;
import com.accio.isegye.game.dto.StockResponse;
import com.accio.isegye.game.repository.StockRepository;
import com.accio.isegye.game.service.GameService;
import io.swagger.v3.oas.annotations.tags.Tag;
import java.util.List;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/game")
@Tag(name = "Game", description = "Game API")
public class GameController {

    private final GameService gameService;

    @GetMapping("/{id}/stock-list")
    public ResponseEntity<List<StockResponse>> getStockList(@PathVariable int id){
        return new ResponseEntity<>(gameService.getStockList(id), HttpStatus.OK);
    }

    @GetMapping("/{id}")
    public ResponseEntity<List<GameResponse>> getGameList(@PathVariable int id){
        return new ResponseEntity<>(gameService.getGameList(id), HttpStatus.OK);
    }
}
