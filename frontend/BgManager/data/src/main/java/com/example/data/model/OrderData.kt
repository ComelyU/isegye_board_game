package com.example.data.model

data class OrderData(
    val orderId: Int,
    val customerId: Int,
    val orderStatus: Int,
    val roomNumber: Int,
    val orderDetailData: List<OrderDetailData>
)