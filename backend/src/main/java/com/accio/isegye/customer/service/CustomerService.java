package com.accio.isegye.customer.service;

import com.accio.isegye.customer.dto.CreateCustomerRequest;
import com.accio.isegye.customer.dto.CustomerResponse;

public interface CustomerService {

    CustomerResponse createCustomer(CreateCustomerRequest createCustomerRequest);

    int endCustomer(int customerId);
}
