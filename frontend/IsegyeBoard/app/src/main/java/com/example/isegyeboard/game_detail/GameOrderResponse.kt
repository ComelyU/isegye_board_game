package com.example.isegyeboard.game_detail


data class GameOrderResponse(
    val id : Long,
    val customerId : Int,
    val stockId : Int,
    val gameName : String,
    val stockLocation : String,
    val orderType : Int,
    val orderStatus : Int,
)
