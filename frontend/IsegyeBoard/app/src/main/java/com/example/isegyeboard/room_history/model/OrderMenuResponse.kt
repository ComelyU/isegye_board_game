package com.example.isegyeboard.room_history.model

data class OrderMenuResponse(
    val id: Int,
    val customerId: Int,
    val orderStatus: Int,
    val orderMenuDetail: List<OrderMenuDetail>
)

