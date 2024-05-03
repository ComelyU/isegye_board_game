package com.accio.isegye.turtle.service;

import com.accio.isegye.exception.CustomException;
import com.accio.isegye.exception.ErrorCode;
import com.accio.isegye.menu.repository.OrderMenuRepository;
import com.accio.isegye.turtle.entity.TurtleLog;
import com.accio.isegye.turtle.repository.TurtleLogRepository;
import com.accio.isegye.turtle.repository.TurtleRepository;
import java.util.List;
import lombok.RequiredArgsConstructor;
import org.modelmapper.ModelMapper;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Service
@RequiredArgsConstructor
public class TurtleServiceImpl implements TurtleService{
    private final TurtleRepository turtleRepository;
    private final TurtleLogRepository turtleLogRepository;
    private final OrderMenuRepository orderMenuRepository;
    private final ModelMapper modelMapper;


    @Override
    @Transactional(readOnly = true)
    public List<Integer> getAvailableTurtleList() {
        return turtleRepository.findIdByIsWorking(1);
    }

    @Override
    @Transactional
    public Integer createMenuLog(int turtleId, long orderMenuId) {
        TurtleLog turtleLog = TurtleLog.builder()
            .turtle(turtleRepository.findById(turtleId)
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "Turtle does not exist: " + turtleId)))
            .commandType(0) // 점주 위치로
            .orderMenu(orderMenuRepository.findById(orderMenuId)
                .orElseThrow(() -> new CustomException(ErrorCode.NOT_FOUND_ERROR, "OrderMenu does not exist: " + orderMenuId)))
            .build();

        TurtleLog save = turtleLogRepository.save(turtleLog);

        return save.getId();
    }
}
