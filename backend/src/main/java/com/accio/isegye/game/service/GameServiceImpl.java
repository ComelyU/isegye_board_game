package com.accio.isegye.game.service;

import com.accio.isegye.common.entity.CodeItem;
import com.accio.isegye.common.repository.CodeItemRepository;
import com.accio.isegye.common.service.S3Service;
import com.accio.isegye.exception.CustomException;
import com.accio.isegye.exception.ErrorCode;
import com.accio.isegye.game.dto.CreateGameRequest;
import com.accio.isegye.game.dto.CreateThemeRequest;
import com.accio.isegye.game.dto.GameListResponse;
import com.accio.isegye.game.dto.GameResponse;
import com.accio.isegye.game.dto.StockResponse;
import com.accio.isegye.game.dto.ThemeListResponse;
import com.accio.isegye.game.dto.ThemeResponse;
import com.accio.isegye.game.dto.UpdateGameRequest;
import com.accio.isegye.game.entity.Game;
import com.accio.isegye.game.entity.GameTagCategory;
import com.accio.isegye.game.entity.Stock;
import com.accio.isegye.game.entity.Theme;
import com.accio.isegye.game.repository.GameRepository;
import com.accio.isegye.game.repository.GameTagCategoryRepository;
import com.accio.isegye.game.repository.OrderGameRepository;
import com.accio.isegye.game.repository.OrderGameStatusLogRepository;
import com.accio.isegye.game.repository.StockRepository;
import com.accio.isegye.game.repository.ThemeRepository;
import com.accio.isegye.store.dto.StoreResponse;
import com.accio.isegye.store.entity.Store;
import java.io.IOException;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.modelmapper.ModelMapper;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.multipart.MultipartFile;

@Slf4j
@Service
@RequiredArgsConstructor
public class GameServiceImpl implements GameService{

    private final GameRepository gameRepository;
    private final GameTagCategoryRepository tagCategoryRepository;
    private final OrderGameRepository orderRepository;
    private final OrderGameStatusLogRepository logRepository;
    private final StockRepository stockRepository;
    private final ThemeRepository themeRepository;
    private final CodeItemRepository codeItemRepository;
    private final S3Service s3Service;
    private final ModelMapper modelMapper;

    // Stock과 StockResponse 매핑
    private StockResponse getStockResponse(Stock stock){
        return modelMapper.map(stock, StockResponse.class);
    }

    // 파일 S3 업로드
    private String uploadFileToS3(MultipartFile file, String mimeTye, String dirName) {
        String fileContentType = file.getContentType();
        if(fileContentType == null || !fileContentType.startsWith(mimeTye)) {
            throw new CustomException(ErrorCode.BAD_REQUEST_ERROR, "Mime type is not supported in this request");
        }

        try {
            return s3Service.upload(file, dirName);
        } catch (IOException e) {
            throw new CustomException(ErrorCode.IO_ERROR, "I/O Exception while saving file to S3");
        }
    }

    // 테마 등록
    @Override
    public ThemeResponse createTheme(
        MultipartFile themeVideo, CreateThemeRequest dto
    ) {
        String themeVideoUrl = uploadFileToS3(themeVideo, "video/", "gameTheme/" + dto.getThemeType());

        return new ThemeResponse(
            themeRepository.save(
                Theme.builder()
                    .themeType(dto.getThemeType())
                    .themeVideoUrl(themeVideoUrl)
                    .build()
            )
        );
    }

    // 테마 목록 조회
    @Override
    public ThemeListResponse getThemeList() {
        return new ThemeListResponse(
            themeRepository.findAll()
                .stream()
                .map(ThemeResponse::new)
                .toList()
        );
    }

    // 테마 조회
    @Override
    public ThemeResponse getTheme(int themeId) {
        Theme theme = themeRepository.findById(themeId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 테마를 찾을 수 없습니다"));

        return new ThemeResponse(theme);
    }

    // 테마 수정
    @Override
    @Transactional
    public Void updateTheme(int themeId, MultipartFile themeVideo) {
        Theme theme = themeRepository.findById(themeId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 테마를 찾을 수 없습니다"));

        String themeVideoUrl = uploadFileToS3(themeVideo, "video/", "gameTheme/" + theme.getThemeType());

        theme.updateThemeVideoUrl(themeVideoUrl);

        return null;
    }

    // 테마 삭제
    @Override
    @Transactional
    public Void deleteTheme(int themeId) {
        Theme theme = themeRepository.findById(themeId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 테마를 찾을 수 없습니다"));

        theme.softDelete();

        return null;
    }

    // 게임 등록
    @Override
    @Transactional
    public GameResponse createGame(MultipartFile gameImg, CreateGameRequest dto) {
        String gameImgUrl = uploadFileToS3(gameImg, "image/", "gameImg/" + dto.getGameName());

        Game game = gameRepository.save(
            Game.builder()
                .gameName(dto.getGameName())
                .gameDetail(dto.getGameDetail())
                .minPlayer(dto.getMinPlayer())
                .maxPlayer(dto.getMaxPlayer())
                .minPlaytime(dto.getMinPlayTime())
                .maxPlaytime(dto.getMaxPlayTime())
                .gameDifficulty(dto.getGameDifficulty())
                .gameImgUrl(gameImgUrl)
                .theme(dto.getThemeId() != null ?
                    themeRepository.findById(dto.getThemeId()).orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 테마를 찾을 수 없습니다"))
                    : null)
                .build()
        );

        if(dto.getTagCategoryIdList() != null && !dto.getTagCategoryIdList().isEmpty()) {
            dto.getTagCategoryIdList().forEach(tagCategoryItemId -> {
                CodeItem codeItem = codeItemRepository.findById(tagCategoryItemId)
                    .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "공통 코드에서 해당하는 태그 또는 카테고리를 찾을 수 없습니다"));

                tagCategoryRepository.save(
                    GameTagCategory.builder()
                        .game(game)
                        .codeGroup(codeItem.getCodeGroup())
                        .codeItem(codeItem)
                        .build()
                );
            });
        }

//        return new GameResponse(
//            gameRepository.findById(game.getId())
//                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 게임을 찾을 수 없습니다"))
//        );
        return new GameResponse(game);
    }

    // 게임 목록 조회
    @Override
    public GameListResponse getGameList() {
        return new GameListResponse(
            gameRepository.findAll()
                .stream()
                .map(GameResponse::new)
                .toList()
        );
    }

    // 게임 조회
    @Override
    public GameResponse getGame(Integer gameId) {
        return new GameResponse(
            gameRepository.findById(gameId)
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 게임을 찾을 수 없습니다"))
        );
    }

    // 게임 수정
    @Override
    @Transactional
    public Void updateGame(Integer gameId, UpdateGameRequest dto) {
        Game game = gameRepository.findById(gameId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 게임을 찾을 수 없습니다"));

        Theme theme = themeRepository.findById(dto.getThemeId())
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 테마를 찾을 수 없습니다"));

        game.updateGameDetailAndTheme(dto.getGameDetail(), theme);

        return null;
    }

    // 게임 삭제
    @Override
    @Transactional
    public Void deleteGame(Integer gameId) {
        Game game = gameRepository.findById(gameId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 게임을 찾을 수 없습니다"));

        game.softDelete();

        return null;
    }

    @Override
    public List<StockResponse> getStockList(int storeId) {

        List<StockResponse> stockResponse = stockRepository.findAllByStoreId (storeId)
            .stream()
            .map(this::getStockResponse)
            .toList();

        return stockResponse;
    }


}
