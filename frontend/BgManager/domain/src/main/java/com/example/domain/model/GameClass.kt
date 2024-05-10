package com.example.domain.model

data class GameClass(
    val gameOrderId: Int,
    val customerId: Int,
    val gameName: String,
    val stockLocation: String,
    val orderType: Int,
    val orderStatus: Int,
    val roomNumber: Int,
)
