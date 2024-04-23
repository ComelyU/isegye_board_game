package com.accio.isegye.menu.service;

import com.accio.isegye.menu.repository.MenuRepository;
import com.accio.isegye.menu.repository.OrderMenuDetailRepository;
import com.accio.isegye.menu.repository.OrderMenuRepository;
import com.accio.isegye.menu.repository.OrderMenuStatusLogRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class MenuServiceImpl implements MenuService{

    private final MenuRepository menuRepository;
    private final OrderMenuDetailRepository detailRepository;
    private final OrderMenuRepository orderMenuRepository;
    private final OrderMenuStatusLogRepository logRepository;
}
