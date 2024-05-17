package com.example.data.model

data class GameData(
    val gameOrderId: Int,
    val customerId: Int,
    val gameName: String,
    val stockLocation: String,
    val orderType: Int,
    val orderStatus: Int,
    val roomNumber: Int,
)
