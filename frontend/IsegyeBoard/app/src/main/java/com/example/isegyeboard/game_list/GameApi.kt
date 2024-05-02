package com.example.isegyeboard.game_list

import retrofit2.http.GET
import retrofit2.http.Path

interface GameApi {

    @GET("game/{storeId}/stock-list")
    suspend fun getGameList(
        @Path("storeId") storeId: String,
    ) : List<GameClass>
}