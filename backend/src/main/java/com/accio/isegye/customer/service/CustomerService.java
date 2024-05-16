package com.accio.isegye.customer.service;

import com.accio.isegye.customer.dto.CreateCustomerRequest;
import com.accio.isegye.customer.dto.CustomerResponse;
import com.accio.isegye.game.dto.GameResponse;
import java.util.List;
import org.springframework.web.multipart.MultipartFile;

public interface CustomerService {

    CustomerResponse createCustomer(int roomId, CreateCustomerRequest createCustomerRequest);

    int endCustomer(int customerId);

    Integer toggleTheme(int customerId);

    Integer findRoom(int customerId);

    String getTheme(int customerId);

    String swapFace(int customerId, MultipartFile sourceFile);

    List<GameResponse> getGameRecommendation(int gameId);
}
