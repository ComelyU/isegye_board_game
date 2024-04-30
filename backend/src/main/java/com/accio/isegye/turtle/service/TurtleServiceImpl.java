package com.accio.isegye.turtle.service;

import com.accio.isegye.turtle.repository.TurtleLogRepository;
import com.accio.isegye.turtle.repository.TurtleRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class TurtleServiceImpl implements TurtleService{
    private TurtleRepository turtleRepository;
    private TurtleLogRepository turtleLogRepository;
}
