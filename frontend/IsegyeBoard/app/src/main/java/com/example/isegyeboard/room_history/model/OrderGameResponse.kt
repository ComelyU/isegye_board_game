package com.example.isegyeboard.room_history.model

data class OrderGameResponse(
    val id: Int,
    val stockId: Int,
    val customerId: Int,
    val gameName: String,
    val orderType: Int,
    val orderStatus: Int,
)
