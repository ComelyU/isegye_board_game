package com.example.domain.model

data class OrderClass(
    val orderId: Int,
    val customerId: Int,
    val orderStatus: Int,
    val orderDetail: List<OrderDetailClass>
)
