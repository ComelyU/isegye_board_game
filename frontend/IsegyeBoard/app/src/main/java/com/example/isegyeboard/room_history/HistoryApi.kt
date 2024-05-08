package com.example.isegyeboard.room_history

import com.example.isegyeboard.room_history.model.OrderGameList
import com.example.isegyeboard.room_history.model.OrderGameResponse
import com.example.isegyeboard.room_history.model.OrderMenuResponse
import retrofit2.http.GET
import retrofit2.http.Path
interface HistoryApi {
    @GET("menu/order/{customerId}")
    suspend fun getMenuHistoryList(
        @Path("customerId") customerId: String,
    ) : List<OrderMenuResponse>

    @GET("game/order/customers/{customerId}")
    suspend fun getGameHistoryList(
        @Path("customerId") customerId: String,
    ) : OrderGameList
}