package com.accio.isegye.menu.service;

import com.accio.isegye.common.service.KafkaProducerService;
import com.accio.isegye.customer.entity.Customer;
import com.accio.isegye.customer.repository.CustomerRepository;
import com.accio.isegye.exception.CustomException;
import com.accio.isegye.exception.ErrorCode;
import com.accio.isegye.menu.dto.CreateMenuRequest;
import com.accio.isegye.menu.dto.CreateOrderMenuRequest;
import com.accio.isegye.menu.dto.MenuResponse;
import com.accio.isegye.menu.dto.OrderMenuResponse;
import com.accio.isegye.menu.dto.UpdateMenuRequest;
import com.accio.isegye.menu.entity.Menu;
import com.accio.isegye.menu.entity.OrderMenu;
import com.accio.isegye.menu.entity.OrderMenuDetail;
import com.accio.isegye.menu.entity.OrderMenuStatusLog;
import com.accio.isegye.menu.repository.MenuRepository;
import com.accio.isegye.menu.repository.OrderMenuDetailRepository;
import com.accio.isegye.menu.repository.OrderMenuRepository;
import com.accio.isegye.menu.repository.OrderMenuStatusLogRepository;
import com.accio.isegye.store.repository.StoreRepository;
import com.accio.isegye.turtle.service.TurtleService;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import lombok.RequiredArgsConstructor;
import org.modelmapper.ModelMapper;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Service
@RequiredArgsConstructor
public class MenuServiceImpl implements MenuService{

    private final MenuRepository menuRepository;
    private final StoreRepository storeRepository;
    private final OrderMenuDetailRepository detailRepository;
    private final OrderMenuRepository orderMenuRepository;
    private final OrderMenuStatusLogRepository logRepository;
    private final CustomerRepository customerRepository;
    private final ModelMapper modelMapper;
    private final KafkaProducerService kafkaProducerService;

    //Menu와 MenuResponse 매핑
    private MenuResponse getMenuResponse(Menu menu){
        return modelMapper.map(menu, MenuResponse.class);
    }

    //OrderMenu와 OrderMenuResponse 매핑
    private OrderMenuResponse getOrderMenuResponse(OrderMenu orderMenu){
        return modelMapper.map(orderMenu, OrderMenuResponse.class);
    }


    @Override
    @Transactional
    public MenuResponse createMenu(int storeId, CreateMenuRequest request) {
        Menu menu = Menu.builder()
            .store(storeRepository.findById(storeId)
                .orElseThrow(()
                    -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "Store does not exist : " + storeId)))
            .menuName(request.getMenuName())
            .menuType(request.getMenuType())
            .menuPrice(request.getMenuPrice())
            .isAvailable(request.getIsAvailable())
            .menuImgUrl(request.getMenuImgUrl())
            .build();

        Menu save = menuRepository.save(menu);

        return getMenuResponse(save);
    }

    //특정 매장의 모든 메뉴 불러오기
    @Override
    @Transactional(readOnly = true)
    public List<MenuResponse> getMenuList(int storeId) {
        return menuRepository.findByStoreIdAndDeletedAtIsNull(storeId)
            .stream()
            .map(this::getMenuResponse)
            .toList();
    }

    @Override
    @Transactional
    public MenuResponse updateMenu(int menuId, UpdateMenuRequest request) {
        Menu menu = menuRepository.findById(menuId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "Menu does not exist: " + menuId));

        menu.setMenuName(request.getMenuName());
        if(request.getMenuType() != null) menu.setMenuType(request.getMenuType());
        if(request.getMenuPrice() != null) menu.setMenuPrice(request.getMenuPrice());
        if(request.getIsAvailable() != null) menu.setIsAvailable(request.getIsAvailable());
        if(request.getMenuImgUrl() != null) menu.setMenuImgUrl(request.getMenuImgUrl());

        Menu update = menuRepository.save(menu);

        return getMenuResponse(update);
    }

    @Override
    @Transactional
    public void deleteMenu(int menuId) {
        Menu menu = menuRepository.findById(menuId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "Menu does not exist: " + menuId));

        menu.setDeletedAt(LocalDateTime.now());
        menuRepository.save(menu);
    }

    @Override
    @Transactional
    public OrderMenuResponse createOrderMenu(List<CreateOrderMenuRequest> orderMenuRequest, int customerId) {
        //1. 메뉴 주문 생성
        OrderMenu orderMenu = OrderMenu.builder()
            .customer(customerRepository.findById(customerId)
                .orElseThrow(()
                    -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "Customer does not exist: " + customerId)))
            .orderStatus(0)
            .build();

        //2. 메뉴 세부 사항 생성
        List<OrderMenuDetail> orderMenuDetailList = orderMenu.getOrderMenuDetailList();
        for(CreateOrderMenuRequest order : orderMenuRequest){
            Menu menu = menuRepository.findById(order.getMenuId())
                .orElseThrow(()
                    -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "Menu does not exist: " + order.getMenuId()));

            OrderMenuDetail orderMenuDetail = OrderMenuDetail.builder()
                .menu(menu)
                .orderMenu(orderMenu)
                .quantity(order.getQuantity())
                .unitPrice(menu.getMenuPrice())
                .totalPrice(order.getQuantity() * menu.getMenuPrice())
                .build();

            orderMenuDetailList.add(orderMenuDetail);

        }

        //3. 메뉴 주문 상태 변경 로그 생성
        OrderMenuStatusLog statusLog = OrderMenuStatusLog.builder()
            .orderMenu(orderMenu)
            .beforeStatus(0)
            .afterStatus(0)
            .build();

        orderMenu.getStatusLogList().add(statusLog);

        OrderMenu save = orderMenuRepository.save(orderMenu);
        detailRepository.saveAll(orderMenuDetailList);
        logRepository.save(statusLog);

        // + Kafka
        kafkaProducerService.send(
            "OrderMenu",
            String.format(
                "[메뉴 주문] 고객 ID: %d, 방 번호: %d",
                save.getCustomer().getId(), save.getCustomer().getRoom().getRoomNumber()
            )
        );

        //4. return OrderMenuResponse // 주문 ID를 반환한다
        return getOrderMenuResponse(save);

    }

    @Override
    @Transactional(readOnly = true)
    public List<OrderMenuResponse> getOrderMenu(int customerId) {
        return orderMenuRepository.findAllByCustomerId(customerId)
            .stream()
            .map(this::getOrderMenuResponse)
            .toList();
    }

    @Override
    @Transactional(readOnly = true)
    public List<OrderMenuResponse> getStoreOrderMenu(int storeId) {
        //특정 매장, 주문 접수(0), 메뉴 준비중(1)
        return orderMenuRepository.findAllByCustomerRoomStoreIdAndOrderStatusLessThanEqual(storeId, 2)
            .stream()
            .map(this::getOrderMenuResponse)
            .toList();
    }

    @Override
    @Transactional
    public void readyOrderMenu(long orderMenuId) {
        //1. order menu 테이블 정보 가져오기
        OrderMenu orderMenu = findOrderMenuById(orderMenuId);

        //2. 새로운 메뉴 주문 상태 변경 로그 생성
        OrderMenuStatusLog statusLog = updateStatusLog(orderMenu, 1);

        //3. order menu 테이블의 order status 컬럼을 갱신
        orderMenu.setOrderStatus(1); // 메뉴 준비중

        //4. 갱신 적용
        orderMenuRepository.save(orderMenu);
        logRepository.save(statusLog);
    }

    @Override
    @Transactional
    public void turtleOrderMenu(long orderMenuId) {
        //1. order menu 테이블의 정보를 가져온다
        OrderMenu orderMenu = findOrderMenuById(orderMenuId);

        //2. 새로운 메뉴 주문 상태 변경 로그 생성
        OrderMenuStatusLog statusLog = updateStatusLog(orderMenu, 2);

        //3. order menu 테이블의 order status 컬럼을 갱신
        orderMenu.setOrderStatus(2);

        //4. 갱신 적용
        orderMenuRepository.save(orderMenu);
        logRepository.save(statusLog);

    }

    @Override
    @Transactional
    public void completeOrderMenu(long orderMenuId) {
        //1. order menu 테이블의 delivered at 컬럼을 갱신
        OrderMenu orderMenu = findOrderMenuById(orderMenuId);
        orderMenu.setDeliveredAt(LocalDateTime.now());

        //2. 새로운 메뉴 주문 상태 변경 로그 생성
        OrderMenuStatusLog statusLog = updateStatusLog(orderMenu, 3);

        //3. order menu 테이블의 order status 컬럼을 갱신
        orderMenu.setOrderStatus(3);

        //4. 갱신 적용
        orderMenuRepository.save(orderMenu);
        logRepository.save(statusLog);

    }

    private OrderMenuStatusLog updateStatusLog(OrderMenu orderMenu, int orderStatus){
        return OrderMenuStatusLog.builder()
            .orderMenu(orderMenu)
            .beforeStatus(orderMenu.getOrderStatus())
            .afterStatus(orderStatus) // 배송완료
            .build();
    }

    private OrderMenu findOrderMenuById(long orderMenuId){
        return orderMenuRepository.findById(orderMenuId)
            .orElseThrow(()
                -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "OrderMenu does not exist: " + orderMenuId));
    }
}
