package com.accio.isegye.turtle.service;

import com.accio.isegye.config.MqttConfig.MqttGateway;
import com.accio.isegye.customer.entity.Customer;
import com.accio.isegye.customer.repository.CustomerRepository;
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
import com.accio.isegye.turtle.dto.TurtleOrderRequest;
import com.accio.isegye.turtle.dto.TurtleIdResponse;
import com.accio.isegye.turtle.dto.TurtleOrderResponse;
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
    private final CustomerRepository customerRepository;
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
    public Long createTurtleLog(int turtleId, Long orderMenuId, Long orderGameId, int commandType) {

        if(orderMenuId == null && orderGameId == null){
            return -1L; // 로그를 만들지 않았다.
        }

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
    public TurtleOrderResponse sendTurtleToCounter(int turtleId, Long turtleOrderLogId, Long turtleReceiveLogId) {
        Turtle turtle = turtleRepository.findById(turtleId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "Turtle does not exist: " + turtleId));

        Room room = roomRepository.findCounterByTurtleId(turtleId);

        StartOrderDto dto = StartOrderDto.builder()
            .turtleOrderLogId(turtleOrderLogId)
            .turtleReceiveLogId(turtleReceiveLogId)
            .coordinateX(room.getCoordinateX())
            .coordinateY(room.getCoordinateY())
            .build();

        //StartOrderDto를 turtleId에 해당하는 로봇에게 보낸다
        mqttGateway.sendToMqtt(dto.toString(), "turtlebot"+turtleId);

        //로봇은 이제 대기상태가 아니다
        turtle.updateIsWorking(1);
        turtleRepository.save(turtle);

        return TurtleOrderResponse.builder()
            .turtleId(turtleId)
            .turtleOrderLogId(turtleOrderLogId)
            .turtleReceiveLogId(turtleReceiveLogId)
            .build();

    }

    @Override
    public void sendOrderToTurtle(int turtleId, Room room, Long turtleOrderLogId, Long turtleReceiveOrderId) {

        //StartOrderDto를 조립한다
        StartOrderDto dto = StartOrderDto.builder()
            .turtleOrderLogId(turtleOrderLogId)
            .turtleReceiveLogId(turtleReceiveOrderId)
            .coordinateX(room.getCoordinateX())
            .coordinateY(room.getCoordinateY())
            .build();

        //StartOrderDto를 turtleId에 해당하는 로봇에게 보낸다
        mqttGateway.sendToMqtt(dto.toString(), "turtlebot"+turtleId);

        //끝
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

    @Override
    @Transactional
    public void updateTurtleOrder(String message) { // turtlebot으로부터 메세지를 받았을 경우
        log.info("Received updateTurtleOrder message: {}", message);
        TurtleOrderRequest orderRequest = new Gson().fromJson(message, TurtleOrderRequest.class);

        log.info("orderRequest : {}", orderRequest.toString());

        //모든 명령을 완수 후 제자리로 돌아갔을 경우 turtle.isWorking을 0으로 바꾼다
        TurtleLog turtleOrderLog = TurtleLog.builder().build();
        TurtleLog turtleReceiveLog = TurtleLog.builder().build();

        boolean orderReturn = false;
        boolean receiveReturn = false;
        if(orderRequest.getTurtleOrderLogId() > 0){
            turtleOrderLog = turtleLogRepository.findById(orderRequest.getTurtleOrderLogId())
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR,
                    "TurtleOrderLog does not exist: " + orderRequest.getTurtleOrderLogId()));
            if(turtleOrderLog.getCommandType() == 255) orderReturn = true;
        }
        if(orderRequest.getTurtleReceiveLogId() > 0){
            turtleReceiveLog = turtleLogRepository.findById(orderRequest.getTurtleReceiveLogId())
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR,
                    "TurtleReceiveLog does not exist: " + orderRequest.getTurtleReceiveLogId()));
            if(turtleReceiveLog.getCommandType() == 255) receiveReturn = true;
        }
        if(orderReturn && receiveReturn){
            updateTurtleLogFromMessage(orderRequest.toString(), turtleOrderLog);
            updateTurtleLogFromMessage(orderRequest.toString(), turtleReceiveLog);
            Turtle turtle = turtleRepository.findById(orderRequest.getTurtleId())
                    .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR ,"Turtle does not exist: " + orderRequest.getTurtleId()));
            turtle.updateIsWorking(0);
            turtleRepository.save(turtle);
            return;
        }

        Long newTurtleOrderLogId = -1L;
        Long newTurtleReceiveLogId = -1L;
        Room room = roomRepository.findByRoomNumber(255);
        int commandType = room.getRoomNumber();

        /*
        * 배송
        * */
        if(orderRequest.getTurtleOrderLogId() > 0){
            turtleOrderLog = turtleLogRepository.findById(orderRequest.getTurtleOrderLogId())
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR,
                    "TurtleOrderLog does not exist: " + orderRequest.getTurtleOrderLogId()));

            log.info("turtleOrderLog : {}", turtleOrderLog.getId());

            if(turtleOrderLog.getCommandType() == 0){// 점주 위치로
                //목표 방 검색
                if(turtleOrderLog.getOrderMenu() != null){ //메뉴 배송
                    room = roomRepository.findRoomByOrderMenuId(turtleOrderLog.getOrderMenu().getId());
                    //배송 정보 갱신
                    OrderMenu orderMenu = turtleOrderLog.getOrderMenu();
                    log.info("orderMenu.id : {}", orderMenu.getId());
                    log.info("orderMenu.orderStatus : {}", orderMenu.getOrderStatus());
                    if(orderMenu.getOrderStatus() == 1){
                        menuService.updateOrderMenu(orderMenu.getId(), 2);
                    }
                }
                if(turtleOrderLog.getOrderGame() != null){ // 게임 배송
                    if(room.getRoomNumber()==255){ // 메뉴 배송에서 바뀌지 않았을 경우
                        room = roomRepository.findRoomByOrderGameId(turtleOrderLog.getOrderGame().getId());    
                    }
                    if(turtleOrderLog.getOrderGame().getOrderStatus() == 0){
                        gameService.updateOrderGameStatus(turtleOrderLog.getOrderGame().getId(), 1);
                    }
                }

                commandType = room.getRoomNumber(); // 목표 갱신
            }else{ // 목표 방인 경우
                if(turtleOrderLog.getOrderMenu() != null) { //메뉴 배송
                    //배송 정보 갱신
                    menuService.updateOrderMenu(turtleOrderLog.getOrderMenu().getId(), 3);
                }
                if(turtleOrderLog.getOrderGame() != null){ // 게임 배송
                    if(turtleOrderLog.getOrderGame().getOrderStatus() != 2){
                        gameService.updateOrderGameStatus(turtleOrderLog.getOrderGame().getId(), 2);
                    }
                    Customer customer = customerRepository.findCustomerByOrderGameId(turtleOrderLog.getOrderGame().getId());

                    if(customer.getIsTheme() == 1){ // 테마를 사용한다
                        Room themeRoom = roomRepository.findRoomByOrderGameId(turtleOrderLog.getOrderGame().getId());
                        String theme = turtleOrderLog.getOrderGame().getStock().getGame().getTheme().getThemeType();
                        mqttGateway.sendToMqtt(theme, "display/"+themeRoom.getId());
                    }

                }

            }

            //기존의 turtleLog를 갱신한다
            updateTurtleLogFromMessage(orderRequest.toString(), turtleOrderLog);

            //새로운 turtleOrderLog 생성
            Long orderMenuId = turtleOrderLog.getOrderMenu()==null ? null : turtleOrderLog.getOrderMenu().getId();
            Long orderGameId = turtleOrderLog.getOrderGame()==null ? null : turtleOrderLog.getOrderGame().getId();
            newTurtleOrderLogId = createTurtleLog(orderRequest.getTurtleId(), orderMenuId, orderGameId,commandType);
            
        }

        /*
        * 회수
        * */
        if(orderRequest.getTurtleReceiveLogId() > 0){
            turtleReceiveLog = turtleLogRepository.findById(orderRequest.getTurtleReceiveLogId())
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR,
                    "TurtleReceiveLog does not exist: " + orderRequest.getTurtleReceiveLogId()));

            if(turtleReceiveLog.getCommandType() == 0) {// 현재 점주 위치
                //목표 방 검색
                if(turtleReceiveLog.getOrderGame().getOrderStatus() == 0){ // 회수하러 출발
                    room = roomRepository.findRoomByOrderGameId(
                        turtleReceiveLog.getOrderGame().getId());
                    commandType = room.getRoomNumber(); // 목표 갱신
                    gameService.updateOrderGameStatus(turtleReceiveLog.getOrderGame().getId(), 1);
                }else{ //아닐 경우 대기 장소로
                    gameService.updateOrderGameStatus(turtleReceiveLog.getOrderGame().getId(), 2);
                }

            }else if(turtleReceiveLog.getCommandType() > 0){ // 현재 목표 방
                room = roomRepository.findCounterByTurtleId(orderRequest.getTurtleId());
                commandType = room.getRoomNumber();
            }

            //기존의 turtleLog를 갱신한다
            updateTurtleLogFromMessage(orderRequest.toString(), turtleReceiveLog);

            //새로운 turtleLog를 생성한다
            Long orderGameId = turtleReceiveLog.getOrderGame().getId();
            newTurtleReceiveLogId = createTurtleLog(orderRequest.getTurtleId(), null, orderGameId, commandType);

        }

        sendOrderToTurtle(orderRequest.getTurtleId(), room, newTurtleOrderLogId, newTurtleReceiveLogId);

    }

}
