package com.accio.isegye.customer.controller;

import com.accio.isegye.customer.service.CustomerService;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/api/customer")
@Tag(name = "Customer", description = "Customer API")
public class CustomerController {

    private final CustomerService customerService;
}
