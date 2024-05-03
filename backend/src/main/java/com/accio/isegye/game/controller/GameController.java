package com.accio.isegye.game.controller;

import com.accio.isegye.game.dto.CreateThemeRequest;
import com.accio.isegye.game.dto.GameResponse;
import com.accio.isegye.game.dto.StockResponse;
import com.accio.isegye.game.dto.ThemeListResponse;
import com.accio.isegye.game.dto.ThemeResponse;
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


    @GetMapping("/{id}/stock-list")
    public ResponseEntity<List<StockResponse>> getStockList(@PathVariable int id){
        return new ResponseEntity<>(gameService.getStockList(id), HttpStatus.OK);
    }

    @GetMapping("/{id}")
    public ResponseEntity<List<GameResponse>> getGameList(@PathVariable int id){
        return new ResponseEntity<>(gameService.getGameList(id), HttpStatus.OK);
    }
}
