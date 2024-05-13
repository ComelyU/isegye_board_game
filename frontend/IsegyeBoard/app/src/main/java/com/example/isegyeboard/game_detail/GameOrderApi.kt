package com.example.isegyeboard.game_detail

import com.example.isegyeboard.baseapi.BasicResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.GET
import retrofit2.http.PATCH
import retrofit2.http.POST
import retrofit2.http.PUT
import retrofit2.http.Path
import retrofit2.http.Query

interface GameOrderApi {

    // 게임 배달 요청
    @POST("game/order/{customerId}/stocks/{stockId}")
    fun orderGame(
        @Path("stockId") gameId: String,
        @Path("customerId") customerId: String,
        @Body requestBody: Map<String, Int>
    ): Call<GameOrderResponse>

    //테마 토글
    @PATCH("customer/{customerId}/theme")
    fun sendThemeToggle(
        @Path("customerId") customerId: String
    ): Call<Int>

    @POST("customer/{customerId}/sound")
    fun sendVolume(
        @Path("customerId") customerId: String,
        @Query("volume") volume: String
    ): Call<Void>
}