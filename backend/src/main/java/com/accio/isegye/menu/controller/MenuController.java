package com.accio.isegye.menu.controller;

import com.accio.isegye.menu.service.MenuService;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/menu")
@Tag(name = "Menu", description = "Menu API")
public class MenuController {

    private final MenuService menuService;
}
