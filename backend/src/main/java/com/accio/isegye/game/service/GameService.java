package com.accio.isegye.game.service;

import com.accio.isegye.game.dto.CreateThemeRequest;
import com.accio.isegye.game.dto.GameResponse;
import com.accio.isegye.game.dto.StockResponse;
import com.accio.isegye.game.dto.ThemeListResponse;
import com.accio.isegye.game.dto.ThemeResponse;
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

    List<StockResponse> getStockList(int storeId);

    List<GameResponse> getGameList(int gameId);
}
