package com.example.isegyeboard.room_history

data class OrderMenuResponse(
    val id: Int,
    val customerId: Int,
    val orderStatus: Int,
    val orderMenuDetail: List<OrderMenuDetail>
)

data class OrderMenuDetail(
    val id: Int,
    val menuName: String,
    val quantity: Int,
    val totalPrice: Int
)