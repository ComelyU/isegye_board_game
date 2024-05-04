package com.accio.isegye.game.controller;

import com.accio.isegye.game.dto.CreateGameRequest;
import com.accio.isegye.game.dto.CreateStockRequest;
import com.accio.isegye.game.dto.CreateThemeRequest;
import com.accio.isegye.game.dto.GameListResponse;
import com.accio.isegye.game.dto.GameResponse;
import com.accio.isegye.game.dto.StockListResponse;
import com.accio.isegye.game.dto.StockResponse;
import com.accio.isegye.game.dto.ThemeListResponse;
import com.accio.isegye.game.dto.ThemeResponse;
import com.accio.isegye.game.dto.UpdateGameRequest;
import com.accio.isegye.game.dto.UpdateStockRequest;
import com.accio.isegye.game.service.GameService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import jakarta.validation.Valid;
import java.util.List;
import lombok.RequiredArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.DeleteMapping;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PatchMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestPart;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.multipart.MultipartFile;

import static org.springframework.http.HttpStatus.*;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/game")
@Tag(name = "Game", description = "Game API")
public class GameController {

    private final GameService gameService;

    /*
    Game Theme
     */

    @Operation(
        summary = "테마 등록",
        description = "테마를 등록한다. 테마 종류와 테마 동영상 파일을 등록."
    )
    @PostMapping(value = "/theme", consumes = {MediaType.MULTIPART_FORM_DATA_VALUE, MediaType.APPLICATION_JSON_VALUE})
    public ResponseEntity<ThemeResponse> createTheme(
        @RequestPart("themeVideo") MultipartFile themeVideo,
        @Valid @RequestPart("dto") CreateThemeRequest dto
    ) {
        return new ResponseEntity<>(
            gameService.createTheme(themeVideo, dto),
            CREATED
        );
    }

    @Operation(
        summary = "테마 목록 조회",
        description = "테마 목록을 조회한다."
    )
    @GetMapping("/theme")
    public ResponseEntity<ThemeListResponse> getThemeList() {
        return new ResponseEntity<>(
            gameService.getThemeList(),
            OK
        );
    }

    @Operation(
        summary = "테마 조회",
        description = "{themeId} 값에 해당하는 테마를 조회한다."
    )
    @GetMapping("/theme/{themeId}")
    public ResponseEntity<ThemeResponse> getTheme(@PathVariable int themeId) {
        return new ResponseEntity<>(
            gameService.getTheme(themeId),
            OK
        );
    }

    @Operation(
        summary = "테마 수정",
        description = "{themeId} 값에 해당하는 테마를 수정한다. 동영상 파일만 수정 가능."
    )
    @PatchMapping(value = "/theme/{themeId}", consumes = {MediaType.MULTIPART_FORM_DATA_VALUE})
    public ResponseEntity<Void> updateTheme(
        @PathVariable int themeId,
        @RequestPart("themeVideo") MultipartFile themeVideo
    ) {
        return new ResponseEntity<>(
            gameService.updateTheme(themeId, themeVideo),
            NO_CONTENT
        );
    }

    @Operation(
        summary = "테마 삭제",
        description = "{themeId} 값에 해당하는 테마를 삭제한다. Soft Delete 처리."
    )
    @DeleteMapping("/theme/{themeId}")
    public ResponseEntity<Void> deleteTheme(@PathVariable int themeId) {
        return new ResponseEntity<>(
            gameService.deleteTheme(themeId),
            NO_CONTENT
        );
    }

    /*
    Game
     */

    @Operation(
        summary = "게임 등록",
        description = "게임을 등록한다."
    )
    @PostMapping(consumes = {MediaType.MULTIPART_FORM_DATA_VALUE, MediaType.APPLICATION_JSON_VALUE})
    public ResponseEntity<GameResponse> createGame(
        @RequestPart("gameImg") MultipartFile gameImg,
        @Valid @RequestPart("dto") CreateGameRequest dto
    ) {
        return new ResponseEntity<>(
            gameService.createGame(gameImg, dto),
            CREATED
        );
    }

    @Operation(
        summary = "게임 목록 조회",
        description = "게임 리스트를 조회한다."
    )
    @GetMapping
    public ResponseEntity<GameListResponse> getGameList() {
        return new ResponseEntity<>(
            gameService.getGameList(),
            OK
        );
    }

    @Operation(
        summary = "게임 조회",
        description = "{gameId}에 값에 해당하는 게임을 조회한다."
    )
    @GetMapping("/{gameId}")
    public ResponseEntity<GameResponse> getGame(@PathVariable Integer gameId) {
        return new ResponseEntity<>(
            gameService.getGame(gameId),
            OK
        );
    }

    @Operation(
        summary = "게임 수정",
        description = "{gameId}에 값에 해당하는 게임을 수정한다. 게임 설명과 테마만 수정 가능."
    )
    @PatchMapping("/{gameId}")
    public ResponseEntity<Void> updateGame(
        @PathVariable Integer gameId,
        @Valid @RequestBody UpdateGameRequest dto
    ) {
        return new ResponseEntity<>(
            gameService.updateGame(gameId, dto),
            NO_CONTENT
        );
    }

    @Operation(
        summary = "게임 삭제",
        description = "{gameId} 값에 해당하는 게임을 삭제한다. Soft Delete 처리."
    )
    @DeleteMapping("/{gameId}")
    public ResponseEntity<Void> deleteGame(@PathVariable Integer gameId) {
        return new ResponseEntity<>(
            gameService.deleteGame(gameId),
            NO_CONTENT
        );
    }

    /*
    Game Stock By Store
     */

    @Operation(
        summary = "매장 게임 재고 등록",
        description = "{storeId}에 해당하는 매장에 {gameId}에 해당하는 게임을 재고로 등록한다."
    )
    @PostMapping("/{gameId}/stores/{storeId}")
    public ResponseEntity<StockResponse> createStockToStore(
        @PathVariable Integer gameId,
        @PathVariable Integer storeId,
        @Valid @RequestBody CreateStockRequest dto
    ) {
        return new ResponseEntity<>(
            gameService.createStockToStore(gameId, storeId, dto),
            CREATED
        );
    }

    @Operation(
        summary = "매장에 따른 게임 재고 목록 조회",
        description = "{storeId}에 해당하는 매장에 등록된 게임 재고 목록을 조회한다."
    )
    @GetMapping("/stocks/stores/{storeId}")
    public ResponseEntity<StockListResponse> getStockListByStore(
        @PathVariable Integer storeId
    ) {
        return new ResponseEntity<>(
            gameService.getStockListByStore(storeId),
            OK
        );
    }

    @Operation(
        summary = "게임 재고 조회",
        description = "{stockId}에 해당하는 재고를 조회한다."
    )
    @GetMapping("/stocks/{stockId}")
    public ResponseEntity<StockResponse> getStock(
        @PathVariable Integer stockId
    ) {
        return new ResponseEntity<>(
            gameService.getStock(stockId),
            OK
        );
    }

    @Operation(
        summary = "게임 재고 수정",
        description = "{stockId}에 해당하는 재고를 수정한다. 재고 보유 여부와 위치만 수정 가능."
    )
    @PatchMapping("/stocks/{stockId}")
    public ResponseEntity<Void> updateStock(
        @PathVariable Integer stockId,
        @Valid @RequestBody UpdateStockRequest dto
    ) {
        return new ResponseEntity<>(
            gameService.updateStock(stockId, dto),
            NO_CONTENT
        );
    }

    @Operation(
        summary = "게임 재고 삭제",
        description = "{stodckId}에 해당하는 재고를 삭제한다. soft delete."
    )
    @DeleteMapping("/stocks/{stockId}")
    public ResponseEntity<Void> deleteStock(
        @PathVariable Integer stockId
    ) {
        return new ResponseEntity<>(
            gameService.deleteStock(stockId),
            NO_CONTENT
        );
    }

}
