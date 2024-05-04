package com.accio.isegye.game.service;

import com.accio.isegye.game.dto.CreateGameRequest;
import com.accio.isegye.game.dto.CreateThemeRequest;
import com.accio.isegye.game.dto.GameListResponse;
import com.accio.isegye.game.dto.GameResponse;
import com.accio.isegye.game.dto.StockResponse;
import com.accio.isegye.game.dto.ThemeListResponse;
import com.accio.isegye.game.dto.ThemeResponse;
import com.accio.isegye.game.dto.UpdateGameRequest;
import java.io.IOException;
import java.util.List;
import org.springframework.web.multipart.MultipartFile;

public interface GameService {

    // 테마 등록
    ThemeResponse createTheme(MultipartFile themeVideo, CreateThemeRequest dto);

    // 테마 목록 조회
    ThemeListResponse getThemeList();

    // 테마 조회
    ThemeResponse getTheme(int themeId);

    // 테마 수정
    Void updateTheme(int themeId, MultipartFile themeVideo);

    // 테마 삭제
    Void deleteTheme(int themeId);

    // 게임 등록
    GameResponse createGame(MultipartFile gameImg, CreateGameRequest dto);

    // 게임 목록 조회
    GameListResponse getGameList();

    // 게임 조회
    GameResponse getGame(Integer gameId);

    // 게임 수정
    Void updateGame(Integer gameId, UpdateGameRequest dto);

    // 게임 삭제
    Void deleteGame(Integer gameId);

    List<StockResponse> getStockList(int storeId);
}
