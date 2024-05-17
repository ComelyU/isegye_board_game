package com.accio.isegye.game.service;

import com.accio.isegye.common.entity.CodeItem;
import com.accio.isegye.common.repository.CodeItemRepository;
import com.accio.isegye.common.service.KafkaProducerService;
import com.accio.isegye.common.service.S3Service;
import com.accio.isegye.customer.entity.Customer;
import com.accio.isegye.customer.repository.CustomerRepository;
import com.accio.isegye.exception.CustomException;
import com.accio.isegye.exception.ErrorCode;
import com.accio.isegye.game.dto.CreateGameRequest;
import com.accio.isegye.game.dto.CreateOrderGameRequest;
import com.accio.isegye.game.dto.CreateStockRequest;
import com.accio.isegye.game.dto.CreateThemeRequest;
import com.accio.isegye.game.dto.GameListResponse;
import com.accio.isegye.game.dto.GameResponse;
import com.accio.isegye.game.dto.OrderGameListResponse;
import com.accio.isegye.game.dto.OrderGameResponse;
import com.accio.isegye.game.dto.StockListResponse;
import com.accio.isegye.game.dto.StockResponse;
import com.accio.isegye.game.dto.ThemeListResponse;
import com.accio.isegye.game.dto.ThemeResponse;
import com.accio.isegye.game.dto.UpdateGameRequest;
import com.accio.isegye.game.dto.UpdateOrderGameRequest;
import com.accio.isegye.game.dto.UpdateStockRequest;
import com.accio.isegye.game.entity.Game;
import com.accio.isegye.game.entity.GameTagCategory;
import com.accio.isegye.game.entity.OrderGame;
import com.accio.isegye.game.entity.OrderGameStatusLog;
import com.accio.isegye.game.entity.Stock;
import com.accio.isegye.game.entity.Theme;
import com.accio.isegye.game.repository.GameRepository;
import com.accio.isegye.game.repository.GameTagCategoryRepository;
import com.accio.isegye.game.repository.OrderGameRepository;
import com.accio.isegye.game.repository.OrderGameStatusLogRepository;
import com.accio.isegye.game.repository.StockRepository;
import com.accio.isegye.game.repository.ThemeRepository;
import com.accio.isegye.store.entity.Store;
import com.accio.isegye.store.repository.StoreRepository;
import java.io.IOException;
import java.time.LocalDateTime;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;
import org.springframework.web.multipart.MultipartFile;

@Slf4j
@Service
@RequiredArgsConstructor
public class GameServiceImpl implements GameService{

    private final GameRepository gameRepository;
    private final GameTagCategoryRepository tagCategoryRepository;
    private final OrderGameRepository orderGameRepository;
    private final OrderGameStatusLogRepository orderGameStatusLogRepository;
    private final StockRepository stockRepository;
    private final ThemeRepository themeRepository;
    private final CodeItemRepository codeItemRepository;
    private final StoreRepository storeRepository;
    private final CustomerRepository customerRepository;
    private final S3Service s3Service;
    private final KafkaProducerService kafkaProducerService;

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

    // 게임 재고 등록
    @Override
    public StockResponse createStockToStore(Integer gameId, Integer storeId,
        CreateStockRequest dto) {
        Store store = storeRepository.findById(storeId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 매장을 찾을 수 없습니다"));

        Game game = gameRepository.findById(gameId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 게임을 찾을 수 없습니다"));

        return new StockResponse(
            stockRepository.save(
                Stock.builder()
                    .store(store)
                    .game(game)
                    .isAvailable(1) // 재고 보유 여부는 true 상태로
                    .stockLocation(dto.getStockLocation())
                    .build()
            )
        );
    }

    // 매장에 따른 게임 재고 목록 확인
    @Override
    public StockListResponse getStockListByStore(Integer storeId) {
        return new StockListResponse(
            stockRepository.findAllByStoreIdAndDeletedAtIsNull(storeId)
                .stream()
                .map(StockResponse::new)
                .toList()
        );
    }

    // 게임 재고 확인
    @Override
    public StockResponse getStock(Integer stockId) {
        return new StockResponse(
            stockRepository.findById(stockId)
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 게임 재고를 찾을 수 없습니다"))
        );
    }

    // 게임 재고 수정
    @Override
    @Transactional
    public Void updateStock(Integer stockId, UpdateStockRequest dto) {
        Stock stock = stockRepository.findById(stockId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 게임 재고를 찾을 수 없습니다"));

        stock.updateIsAvailableAndStockLocation(dto.getIsAvailable(), dto.getStockLocation());

        return null;
    }

    // 게임 재고 삭제
    @Override
    @Transactional
    public Void deleteStock(Integer stockId) {
        Stock stock = stockRepository.findById(stockId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 게임 재고를 찾을 수 없습니다"));

        stock.softDelete();

        return null;
    }

    // 게임 주문 등록
    @Override
    @Transactional
    public OrderGameResponse createOrderGame(Integer customerId, Integer stockId,
        CreateOrderGameRequest dto) {
        Customer customer = customerRepository.findById(customerId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 고객을 찾을 수 없습니다"));
        Stock stock = stockRepository.findById(stockId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 게임 재고를 찾을 수 없습니다"));
        if(customer.getEndTime() != null) {
            throw new CustomException(ErrorCode.BAD_REQUEST_ERROR, "이미 사용 종료된 고객입니다");
        }
        if(customer.getRoom().getStore().getId() != stock.getStore().getId()) {
            throw new CustomException(ErrorCode.BAD_REQUEST_ERROR, "매장의 고객이 아닙니다");
        }
        if(dto.getOrderType() == 0 && stock.getIsAvailable() == 0) {
            throw new CustomException(ErrorCode.BAD_REQUEST_ERROR, "주문 요청이 불가능한 게임입니다");
        }
        if(dto.getOrderType() == 1 && stock.getIsAvailable() == 1) {
            throw new CustomException(ErrorCode.BAD_REQUEST_ERROR, "회수 요청이 불가능한 게임입니다");
        }

        OrderGame orderGame = orderGameRepository.save(
            OrderGame.builder()
                .customer(customer)
                .stock(stock)
                .orderType(dto.getOrderType())
                .orderStatus(0)
                .build()
        );

        orderGameStatusLogRepository.save(
            OrderGameStatusLog.builder()
                .orderGame(orderGame)
                .beforeStatus(0)
                .afterStatus(0)
                .build()
        );

        // 주문이면 주문 접수 즉시 재고 보유 떨어야 함. 회수인 경우에는 배달 완료가 되었을 때 재고 보유 처리해야 함.
        if(dto.getOrderType() == 0) {
            stock.updateIsAvailableAndStockLocation(0, stock.getStockLocation());
        }

        kafkaProducerService.send(
            "OrderGame",
            String.format(
                "[게임 %s 요청] 고객 ID: %d, 방 번호: %d, 게임명: %s, 재고 위치: %s",
                orderGame.getOrderType() == 0 ? "주문" : "회수",
                orderGame.getCustomer().getId(), orderGame.getCustomer().getRoom().getRoomNumber(),
                orderGame.getStock().getGame().getGameName(), orderGame.getStock().getStockLocation()
            )
        );

        return new OrderGameResponse(orderGame);
    }

    // 고객에 따른 게임 주문 목록 조회
    @Override
    public OrderGameListResponse getOrderGameListByCustomer(Integer customerId) {
        return new OrderGameListResponse(
            orderGameRepository.findAllByCustomerId(customerId)
                .stream()
                .map(OrderGameResponse::new)
                .toList()
        );
    }

    // 매장에 따른 게임 주문 목록 조회
    @Override
    public OrderGameListResponse getOrderGameListByStore(Integer storeId) {
        return new OrderGameListResponse(
            orderGameRepository.findAllByStock_StoreId(storeId)
                .stream()
                .map(OrderGameResponse::new)
                .toList()
        );
    }

    // 게임 주문 조회
    @Override
    public OrderGameResponse getOrderGame(Long orderGameId) {
        return new OrderGameResponse(
            orderGameRepository.findById(orderGameId)
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 게임 주문을 찾을 수 없습니다"))
        );
    }

    // 게임 주문의 상태 변경
    @Override
    @Transactional
    public Void updateOrderGameStatus(Long orderGameId, UpdateOrderGameRequest dto) {
        OrderGame orderGame = orderGameRepository.findById(orderGameId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 게임 주문을 찾을 수 없습니다"));

        // 현재 주문 상태와 변경하려는 주문 상태가 같은 경우
        if(orderGame.getOrderStatus() == dto.getOrderStatus()) {
            throw new CustomException(ErrorCode.BAD_REQUEST_ERROR, "현재 주문 상태와 변경 주문 상태가 같습니다");
        }

        // 배달 완료 처리된 주문에 대해서 상태를 변경하려는 경우
        if(orderGame.getOrderStatus() == 2) {
            throw new CustomException(ErrorCode.BAD_REQUEST_ERROR, "배달 완료된 주문입니다");
        }

        // 취소 처리된 주문에 대해서 상태를 변경하려는 경우
        if(orderGame.getOrderStatus() == 3) {
            throw new CustomException(ErrorCode.BAD_REQUEST_ERROR, "이미 취소된 주문입니다");
        }

        orderGameStatusLogRepository.save(
            OrderGameStatusLog.builder()
                .orderGame(orderGame)
                .beforeStatus(orderGame.getOrderStatus())
                .afterStatus(dto.getOrderStatus())
                .build()
        );

        if(dto.getOrderStatus() == 2) { // 배달 완료로 상태를 변경하려는 경우
            orderGame.updateOrderStatusAndDelieveredAt(dto.getOrderStatus(), LocalDateTime.now());

            if(orderGame.getOrderType() == 1) { // 회수 요청인 경우 재고 보유 여부 true
                orderGame.getStock().updateIsAvailableAndStockLocation(1, orderGame.getStock().getStockLocation());
            }
        } else { // 다른 상태로 변경하는 경우
            orderGame.updateOrderStatusAndDelieveredAt(dto.getOrderStatus(), null);

            if(orderGame.getOrderType() == 0 && dto.getOrderStatus() == 3) { // 주문 요청이면서 변경하려는 상태가 주문 취소 상태인 경우 재고 보유 여부 true
                orderGame.getStock().updateIsAvailableAndStockLocation(1, orderGame.getStock().getStockLocation());
            }
        }

        return null;
    }

    @Override
    @Transactional
    public void updateOrderGameStatus(Long orderGameId, int orderStatus) {
        OrderGame orderGame = orderGameRepository.findById(orderGameId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 게임 주문을 찾을 수 없습니다"));

        // 현재 주문 상태와 변경하려는 주문 상태가 같은 경우
        if(orderGame.getOrderStatus() == orderStatus) {
            throw new CustomException(ErrorCode.BAD_REQUEST_ERROR, "현재 주문 상태와 변경 주문 상태가 같습니다");
        }

        // 배달 완료 처리된 주문에 대해서 상태를 변경하려는 경우
        if(orderGame.getOrderStatus() == 2) {
            throw new CustomException(ErrorCode.BAD_REQUEST_ERROR, "배달 완료된 주문입니다");
        }

        // 취소 처리된 주문에 대해서 상태를 변경하려는 경우
        if(orderGame.getOrderStatus() == 3) {
            throw new CustomException(ErrorCode.BAD_REQUEST_ERROR, "이미 취소된 주문입니다");
        }

        orderGameStatusLogRepository.save(
            OrderGameStatusLog.builder()
                .orderGame(orderGame)
                .beforeStatus(orderGame.getOrderStatus())
                .afterStatus(orderStatus)
                .build()
        );

        if(orderStatus == 2) { // 배달 완료로 상태를 변경하려는 경우
            orderGame.updateOrderStatusAndDelieveredAt(orderStatus, LocalDateTime.now());

            if(orderGame.getOrderType() == 1) { // 회수 요청인 경우 재고 보유 여부 true
                orderGame.getStock().updateIsAvailableAndStockLocation(1, orderGame.getStock().getStockLocation());
            }
        } else { // 다른 상태로 변경하는 경우
            orderGame.updateOrderStatusAndDelieveredAt(orderStatus, null);

            if(orderGame.getOrderType() == 0 && orderStatus == 3) { // 주문 요청이면서 변경하려는 상태가 주문 취소 상태인 경우 재고 보유 여부 true
                orderGame.getStock().updateIsAvailableAndStockLocation(1, orderGame.getStock().getStockLocation());
            }
        }

        return;
    }


    // 게임 주문 삭제
    @Override
    @Transactional
    public Void deleteOrderGame(Long orderGameId) {
        OrderGame orderGame = orderGameRepository.findById(orderGameId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "해당하는 게임 주문을 찾을 수 없습니다"));

        orderGameStatusLogRepository.save(
            OrderGameStatusLog.builder()
                .orderGame(orderGame)
                .beforeStatus(orderGame.getOrderStatus())
                .afterStatus(3)
                .build()
        );

        orderGame.softDelete();

        if(orderGame.getOrderType() == 0) { // 주문 요청에 대해서 처리하는 경우 재고 보유 여부 true
            orderGame.getStock().updateIsAvailableAndStockLocation(1, orderGame.getStock().getStockLocation());
        }

        return null;
    }
}
