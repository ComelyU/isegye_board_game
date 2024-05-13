package com.accio.isegye.customer.service;

import com.accio.isegye.customer.dto.CreateCustomerRequest;
import com.accio.isegye.customer.dto.CustomerResponse;

public interface CustomerService {

    CustomerResponse createCustomer(int roomId, CreateCustomerRequest createCustomerRequest);

    int endCustomer(int customerId);

    Integer toggleTheme(int customerId);

    Integer findRoom(int customerId);

    String getTheme(int customerId);
}
