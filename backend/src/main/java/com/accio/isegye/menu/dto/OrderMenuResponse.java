package com.accio.isegye.menu.dto;

import com.accio.isegye.menu.entity.OrderMenuDetail;
import jakarta.validation.constraints.NotNull;
import java.util.ArrayList;
import java.util.List;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
@AllArgsConstructor
public class OrderMenuResponse {

    @NotNull
    private long id;

    @NotNull
    private int customerId;

    private int orderStatus;

    private List<OrderMenuDetailResponse> orderMenuDetail = new ArrayList<>();

}
