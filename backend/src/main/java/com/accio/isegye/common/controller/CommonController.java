package com.accio.isegye.common.controller;

import com.accio.isegye.common.service.CommonService;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/common")
@Tag(name = "Common", description = "Common API")
public class CommonController {

    private final CommonService commonService;
}
