package com.example.isegyeboard.game_list

import com.example.isegyeboard.game_detail.GameOrderResponse
import com.example.isegyeboard.game_list.model.GameResponse
import com.example.isegyeboard.game_list.model.StockList
import retrofit2.Call
import retrofit2.http.GET
import retrofit2.http.Path

interface GameApi {

    @GET("game/stocks/stores/{storeId}")
    suspend fun getGameList(
        @Path("storeId") storeId: String,
    ) : StockList
}