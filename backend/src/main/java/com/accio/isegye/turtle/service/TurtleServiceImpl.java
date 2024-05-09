package com.accio.isegye.turtle.service;

import com.accio.isegye.config.MqttConfig.MqttGateway;
import com.accio.isegye.exception.CustomException;
import com.accio.isegye.exception.ErrorCode;
import com.accio.isegye.game.entity.OrderGame;
import com.accio.isegye.game.repository.OrderGameRepository;
import com.accio.isegye.game.service.GameService;
import com.accio.isegye.menu.entity.OrderMenu;
import com.accio.isegye.menu.repository.OrderMenuRepository;
import com.accio.isegye.menu.service.MenuService;
import com.accio.isegye.store.entity.Room;
import com.accio.isegye.store.repository.RoomRepository;
import com.accio.isegye.store.repository.StoreRepository;
import com.accio.isegye.turtle.dto.StartOrderDto;
import com.accio.isegye.turtle.dto.StartTurtleOrderRequest;
import com.accio.isegye.turtle.dto.TurtleIdResponse;
import com.accio.isegye.turtle.dto.UpdateTurtleRequest;
import com.accio.isegye.turtle.entity.Turtle;
import com.accio.isegye.turtle.entity.TurtleLog;
import com.accio.isegye.turtle.repository.TurtleLogRepository;
import com.accio.isegye.turtle.repository.TurtleRepository;
import com.google.gson.Gson;
import com.google.gson.JsonObject;
import java.time.LocalDateTime;
import java.util.List;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.modelmapper.ModelMapper;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Slf4j
@Service
@RequiredArgsConstructor
public class TurtleServiceImpl implements TurtleService{
    private final TurtleRepository turtleRepository;
    private final TurtleLogRepository turtleLogRepository;
    private final OrderMenuRepository orderMenuRepository;
    private final OrderGameRepository orderGameRepository;
    private final StoreRepository storeRepository;
    private final RoomRepository roomRepository;
    private final MqttGateway mqttGateway;
    private final MenuService menuService;
    private final GameService gameService;
    private final ModelMapper modelMapper;


    @Override
    @Transactional
    public Integer createTurtle(int storeId) {
        Turtle turtle = Turtle.builder()
            .store(storeRepository.findById(storeId)
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "Store does not exist: " + storeId)))
            .isWorking(0) // 대기중
            .build();

        Turtle save = turtleRepository.save(turtle);
        return save.getId();
    }

    @Override
    @Transactional
    public void updateTurtle(int turtleId, UpdateTurtleRequest request) {
        Turtle turtle = turtleRepository.findById(turtleId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR,
                "Turtle does not exist: " + turtleId));

        if(request.getStoreId() != null){
            turtle.updateStore(storeRepository.findById(request.getStoreId())
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR,
                    "Store does not exist: " + request.getStoreId())));
        }
        if(request.getIsWorking() != null){
            turtle.updateIsWorking(request.getIsWorking());
        }

        turtleRepository.save(turtle);
    }

    @Override
    @Transactional
    public void deleteTurtle(int turtleId) {
        Turtle turtle = turtleRepository.findById(turtleId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR,
                "Turtle does not exist: " + turtleId));
        turtle.softDelete();

        turtleRepository.save(turtle);
    }

    @Override
    @Transactional(readOnly = true)
    public List<TurtleIdResponse> getAvailableTurtleList(int storeId) {
        return turtleRepository.findByStoreIdAndIsWorking(storeId,0)
            .stream()
            .map(id -> modelMapper.map(id, TurtleIdResponse.class))
            .toList();
    }

    @Override
    @Transactional
    public Long createTurtleLog(int turtleId, Long orderMenuId, Long orderGameId, Long receiveGameId, int commandType) {
        TurtleLog turtleLog = TurtleLog.builder()
            .turtle(turtleRepository.findById(turtleId)
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR,
                    "Turtle does not exist: " + turtleId)))
            .commandType(commandType)// 점주 위치로
            .orderMenu(orderMenuId != null ?
                orderMenuRepository.findById(orderMenuId)
                    .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR,
                        "OrderMenu does not exist: " + orderMenuId))
                : null)
            .orderGame(orderGameId != null ?
                orderGameRepository.findById(orderGameId)
                    .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR,
                        "OrderGame does not exist: " + orderGameId))
                : null
            )
            .build();

        TurtleLog save = turtleLogRepository.save(turtleLog);

        return save.getId();
    }

    @Override
    @Transactional
    public void sendTurtleToCounter(int turtleId, Long turtleLogId) {
        Turtle turtle = turtleRepository.findById(turtleId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "Turtle does not exist: " + turtleId));

        Room room = roomRepository.findCounterByTurtleId(turtleId);

        StartOrderDto dto = StartOrderDto.builder()
            .turtleLogId(turtleLogId)
            .coordinateX(room.getCoordinateX())
            .coordinateY(room.getCoordinateY())
            .build();

        //StartOrderDto를 turtleId에 해당하는 로봇에게 보낸다
        mqttGateway.sendToMqtt(dto.toString(), "ros_test/"+turtleId);

        //로봇은 이제 대기상태가 아니다
        turtle.updateIsWorking(1);
        turtleRepository.save(turtle);

    }

    @Override
    public void sendOrderToTurtle(int turtleId, Long orderMenuId, Long orderGameId, Long turtleLogId) {
        //Room에서 필요한 정보를 가져온다
        Room room = null;
        if(orderMenuRepository.existsById(orderMenuId)){
            room = roomRepository.findRoomByOrderMenuId(orderMenuId);
        }else if(orderGameRepository.existsById(orderGameId)){
            room = roomRepository.findRoomByOrderGameId(orderGameId);
        }else{
            throw new RuntimeException(new CustomException(ErrorCode.NOT_FOUND_ERROR, "Room does not exist"));
        }

        //StartOrderDto를 조립한다
        StartOrderDto dto = StartOrderDto.builder()
            .turtleLogId(turtleLogId)
            .coordinateX(room.getCoordinateX())
            .coordinateY(room.getCoordinateY())
            .build();

        //StartOrderDto를 turtleId에 해당하는 로봇에게 보낸다
        mqttGateway.sendToMqtt(dto.toString(), "ros_test/"+turtleId);

        //끝
    }

    //로그 갱신
    //1. 배송 출발
    @Override
    @Transactional
    public void startOrder(StartTurtleOrderRequest request){
        TurtleLog turtleLog = turtleLogRepository.findById(request.getTurtleLogId())
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR,
                "TurtleLog does not exist: " + request.getTurtleId()));

        if(turtleLog.getTurtle().getId() != request.getTurtleId()){
            throw new CustomException(ErrorCode.BAD_REQUEST_ERROR, "해당 주문은 해당 로봇의 작업이 아닙니다");
        }

        //기존의 turtleLog를 갱신한다
        updateTurtleLogFromMessage(request.toString(), turtleLog);

        //배송중으로 변경
        if(turtleLog.getOrderGame() != null){
            //********************************게임 주문 테이블 갱신 추가 ***********************************
        }
        if(turtleLog.getOrderMenu() != null){
            menuService.updateOrderMenu(turtleLog.getOrderMenu().getId(), 2);
        }

        //새로운 turtlelog를 만들어서 해당 id를 전송한다
        Long orderMenuId = turtleLog.getOrderMenu()==null ? null : turtleLog.getOrderMenu().getId();
        Long orderGameId = turtleLog.getOrderGame()==null ? null : turtleLog.getOrderGame().getId();

        Long newTurtleLogId = createTurtleLog(request.getTurtleId(),
            orderMenuId,
            orderGameId,
            null
        , 1);

        sendOrderToTurtle(request.getTurtleId(), orderMenuId, orderGameId, newTurtleLogId);

    }

    @Transactional
    public void updateTurtleLogFromMessage(String message, TurtleLog turtleLog){
        //해당 터틀로그의 명령 완료 시간을 갱신
        turtleLog.setCommandEndTime(LocalDateTime.now());
        //해당 터틀로그의 성공 여부를 갱신
        turtleLog.setIsSuccess(1);
        //해당 터틀로그에 로그 메시지를 입력
        turtleLog.setLogMessage(message);
        //로그 갱신
        turtleLogRepository.save(turtleLog);
    }

    //로그 갱신
    //2. 수령 완료
    @Override
    @Transactional
    public void receiveOrder(String message){
        log.info("Received receiveOrder message: {}", message);
        JsonObject convertObject = new Gson().fromJson(message, JsonObject.class);

        long turtleLogId = convertObject.get("turtleLogId").getAsLong();
        TurtleLog turtleLog = turtleLogRepository.findById(turtleLogId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR,
                "TurtleLog does not exist: " + turtleLogId));

        //기존의 turtleLog를 갱신한다
        updateTurtleLogFromMessage(message, turtleLog);

        //메뉴, 게임 주문 배송 완료 갱신
        updateOrder(turtleLog);

    }

    @Transactional
    private void updateOrder(TurtleLog turtleLog){
        OrderMenu orderMenu = turtleLog.getOrderMenu();
        OrderGame orderGame = turtleLog.getOrderGame();

        if(orderMenu != null){
            orderMenu.updateOrderStatusAndDelieveredAt(3, LocalDateTime.now());
        }
        if(orderGame != null){
            orderGame.updateOrderStatusAndDelieveredAt(3, LocalDateTime.now());
        }
    }

    //반납 요청


}
