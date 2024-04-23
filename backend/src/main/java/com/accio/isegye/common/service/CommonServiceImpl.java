package com.accio.isegye.common.service;

import com.accio.isegye.common.repository.CodeGroupRepository;
import com.accio.isegye.common.repository.CodeItemRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class CommonServiceImpl implements CommonService{

    private final CodeGroupRepository groupRepository;
    private final CodeItemRepository itemRepository;
}
