package com.accio.isegye.customer.service;

import com.accio.isegye.customer.dto.CreateCustomerRequest;
import com.accio.isegye.customer.dto.CustomerResponse;
import com.accio.isegye.customer.entity.Customer;
import com.accio.isegye.customer.repository.CustomerRepository;
import com.accio.isegye.exception.CustomException;
import com.accio.isegye.exception.ErrorCode;
import com.accio.isegye.game.entity.Game;
import com.accio.isegye.store.dto.RoomResponse;
import com.accio.isegye.store.repository.RoomRepository;
import com.accio.isegye.store.repository.StoreRepository;
import jakarta.persistence.criteria.CriteriaBuilder.In;
import java.time.Duration;
import java.time.LocalDateTime;
import lombok.RequiredArgsConstructor;
import org.modelmapper.ModelMapper;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Service
@RequiredArgsConstructor
public class CustomerServiceImpl implements CustomerService{

    private final CustomerRepository customerRepository;
    private final RoomRepository roomRepository;
    private final StoreRepository storeRepository;
    private final ModelMapper modelMapper;

    private CustomerResponse getCustomerResponse(Customer customer){
        return modelMapper.map(customer, CustomerResponse.class);
    }

    @Override
    @Transactional
    public CustomerResponse createCustomer(int roomId, CreateCustomerRequest request) {
        Customer customer = Customer.builder()
            .room(roomRepository.findById(roomId)
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "No Room matching: " + roomId)))
            .isTheme(request.getIsTheme())
            .peopleNum(request.getPeopleNum())
            .build();

        Customer save = customerRepository.save(customer);

        return getCustomerResponse(save);
    }

    @Override
    @Transactional
    public int endCustomer(int customerId) {
        Customer customer = customerRepository.findById(customerId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "No Customer matching: " + customerId));

        LocalDateTime startTime = customer.getStartTime();
        LocalDateTime endTime = LocalDateTime.now();
        Duration duration = Duration.between(startTime, endTime);
        long hourDifference = duration.toHours(); // 올림

        int hourFee = storeRepository.findHourFeeById(customer.getRoom().getStore().getId());

        customer.setEndTime(endTime);
        customer.setRoomFee((int)hourDifference * hourFee * customer.getPeopleNum());
        Customer update = customerRepository.save(customer);

        Integer menuFee = customerRepository.getMenuFeeByCustomerId(customerId);
        if(menuFee == null){
            menuFee = 0;
        }

        return update.getRoomFee() + menuFee;
    }

    @Override
    @Transactional
    public Integer toggleTheme(int customerId) {
        Customer customer = customerRepository.findById(customerId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "No Customer matching: " + customerId));
        int isTheme = customer.getIsTheme();
        customer.setIsTheme( (isTheme+1)%2 ); // 테마 사용 여부 전환

        Customer update = customerRepository.save(customer);

        return update.getIsTheme();
    }

    @Override
    public Integer findRoom(int customerId) {
        return roomRepository.findByCustomerId(customerId).getId();
    }

    @Override
    public String getTheme(int customerId) {
        Customer customer = customerRepository.findById(customerId)
            .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "No Customer matching: " + customerId));

        if (customer.getOrderGameList().isEmpty()
            || customer.getOrderGameList().get(customer.getOrderGameList().size()-1).getOrderStatus()==1){
            //주문한 적이 없거나 마지막 주문이 회수 상태이면
            return "Default";
        }

        Game game = customer.getOrderGameList().get(customer.getOrderGameList().size()-1).getStock()
            .getGame();

        return game.getTheme().getThemeType();
    }
}